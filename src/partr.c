// This file is a part of Julia. License is MIT: https://julialang.org/license

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>

#include "julia.h"
#include "julia_internal.h"
#include "threading.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef JULIA_ENABLE_THREADING
#ifdef JULIA_ENABLE_PARTR

/* the `start` task */
jl_ptask_t *start_task;

/* sticky task queues need to be visible to all threads */
jl_ptask_t ***all_taskqs;
int8_t     **all_taskq_locks;

/* forward declarations */
static int run_next();

/* internally used to indicate a yield occurred in the runtime itself */
static const int64_t yield_from_sync = 1;


// initialize the threading infrastructure
void jl_init_threadinginfra(void)
{
    /* initialize the synchronization trees pool and the multiqueue */
    synctreepool_init();
    multiq_init();

    /* allocate per-thread task queues, for sticky tasks */
    all_taskqs = (jl_ptask_t ***)jl_malloc_aligned(jl_n_threads * sizeof(jl_ptask_t **), 64);
    all_taskq_locks = (int8_t **)jl_malloc_aligned(jl_n_threads * sizeof(int8_t *), 64);
}


// initialize the thread function argument
void jl_init_threadarg(jl_threadarg_t *targ) { }


// helper for final thread initialization
static void init_started_thread()
{
    jl_ptls_t ptls = jl_get_ptls_states();

    /* allocate this thread's sticky task queue pointer and initialize the lock */
    seed_cong(&ptls->rngseed);
    ptls->sticky_taskq_lock = (int8_t *)
            jl_malloc_aligned(sizeof(int8_t) + sizeof(jl_ptask_t *), 64);
    ptls->sticky_taskq = (jl_ptask_t **)(ptls->sticky_taskq_lock + sizeof(int8_t));
    jl_atomic_clear(ptls->sticky_taskq_lock);
    *ptls->sticky_taskq = NULL;
    all_taskqs[ptls->tid] = ptls->sticky_taskq;
    all_taskq_locks[ptls->tid] = ptls->sticky_taskq_lock;
}


// once the threads are started, perform any final initializations
void jl_init_started_threads(jl_threadarg_t **targs)
{
    // master thread final initialization
    init_started_thread();
}


// thread function: used by all except the main thread
void jl_threadfun(void *arg)
{
    jl_threadarg_t *targ = (jl_threadarg_t *)arg;

    // initialize this thread (set tid, create heap, etc.)
    jl_init_threadtls(targ->tid);
    jl_init_stack_limits(0);

    jl_ptls_t ptls = jl_get_ptls_states();

    // set up tasking
    jl_init_root_task(ptls->stack_lo, ptls->stack_hi - ptls->stack_lo);
#ifdef COPY_STACKS
    jl_set_base_ctx((char*)&arg);
#endif

    init_started_thread();

    // Assuming the functions called below doesn't contain unprotected GC
    // critical region. In general, the following part of this function
    // shouldn't call any managed code without calling `jl_gc_unsafe_enter`
    // first.
    jl_gc_state_set(ptls, JL_GC_STATE_SAFE, 0);
    uv_barrier_wait(targ->barrier);

    // free the thread argument here
    free(targ);

    /* get the highest priority task and run it */
    while (run_next() == 0)
        ;

    /* free the sticky task queue pointer (and its lock) */
    jl_free_aligned(ptls->sticky_taskq_lock);
}


// old threading interface: run specified function in all threads. partr created
// jl_n_threads tasks and enqueues them; these may not actually run in all the
// threads.
JL_DLLEXPORT jl_value_t *jl_threading_run(jl_value_t *_args)
{
    // TODO.
    return NULL;
}


// coroutine entry point
static void partr_coro(void *ctx)
{
    jl_ptask_t *task = (jl_ptask_t *)ctx; // TODO. ctx_get_user_ptr(ctx);
    task->result = task->f(task->arg, task->start, task->end);

    /* grain tasks must synchronize */
    if (task->grain_num >= 0) {
        int was_last = 0;

        /* reduce... */
        if (task->red) {
            task->result = reduce(task->arr, task->red, task->rf,
                                  task->result, task->grain_num);
            /*  if this task is last, set the result in the parent task */
            if (task->result) {
                task->parent->red_result = task->result;
                was_last = 1;
            }
        }
        /* ... or just sync */
        else {
            if (last_arriver(task->arr, task->grain_num))
                was_last = 1;
        }

        /* the last task to finish needs to finish up the loop */
        if (was_last) {
            /* a non-parent task must wake up the parent */
            if (task->grain_num > 0) {
                multiq_insert(task->parent, 0);
            }
            /* the parent task was last; it can just end */
        }
        else {
            /* the parent task needs to wait */
            if (task->grain_num == 0) {
                // TODO. yield_value(task->ctx, (void *)yield_from_sync);
            }
        }
    }
}


// allocate and initialize a task
static jl_ptask_t *setup_task(void *(*f)(void *, int64_t, int64_t), void *arg,
        int64_t start, int64_t end)
{
    jl_ptask_t *task = NULL; // TODO. task_alloc();
    if (task == NULL)
        return NULL;

    // TODO. ctx_construct(task->ctx, task->stack, TASK_STACK_SIZE, partr_coro, task);
    task->f = f;
    task->arg = arg;
    task->start = start;
    task->end = end;
    task->sticky_tid = -1;
    task->grain_num = -1;

    return task;
}


// free a task
static void *release_task(jl_ptask_t *task)
{
    void *result = task->result;
    // TODO. ctx_destruct(task->ctx);
    if (task->grain_num == 0  &&  task->red)
        reducer_free(task->red);
    if (task->grain_num == 0  &&  task->arr)
        arriver_free(task->arr);
    task->f = NULL;
    task->arg = task->result = task->red_result = NULL;
    task->start = task->end = 0;
    task->rf = NULL;
    task->parent = task->cq = NULL;
    task->arr = NULL;
    task->red = NULL;
    // TODO. task_free(task);
    return result;
}


// add the specified task to the sticky task queue
static void add_to_taskq(jl_ptask_t *task)
{
    assert(task->sticky_tid != -1);

    jl_ptask_t **q = all_taskqs[task->sticky_tid];
    int8_t *lock = all_taskq_locks[task->sticky_tid];

    while (jl_atomic_test_and_set(lock))
        jl_cpu_pause();

    if (*q == NULL)
        *q = task;
    else {
        jl_ptask_t *pt = *q;
        while (pt->next)
            pt = pt->next;
        pt->next = task;
    }

    jl_atomic_clear(lock);
}


// pop the first task off the sticky task queue
static jl_ptask_t *get_from_taskq()
{
    jl_ptls_t ptls = jl_get_ptls_states();

    /* racy check for quick path */
    if (*ptls->sticky_taskq == NULL)
        return NULL;

    while (jl_atomic_test_and_set(ptls->sticky_taskq_lock))
        jl_cpu_pause();

    jl_ptask_t *task = *ptls->sticky_taskq;
    if (task) {
        *ptls->sticky_taskq = task->next;
        task->next = NULL;
    }

    jl_atomic_clear(ptls->sticky_taskq_lock);

    return task;
}


// get the next available task and run it
static int run_next()
{
#if 0
            // TODO: before we support getting return value from
            //       the work, and after we have proper GC transition
            //       support in the codegen and runtime we don't need to
            //       enter GC unsafe region when starting the work.
            int8_t gc_state = jl_gc_unsafe_enter(ptls);
            // This is probably always NULL for now
            jl_module_t *last_m = ptls->current_module;
            size_t last_age = ptls->world_age;
            JL_GC_PUSH1(&last_m);
            ptls->current_module = work->current_module;
            ptls->world_age = work->world_age;
            jl_thread_run_fun(&work->fptr, work->mfunc, work->args, work->nargs);
            ptls->current_module = last_m;
            ptls->world_age = last_age;
            JL_GC_POP();
            jl_gc_unsafe_leave(ptls, gc_state);
#endif

    jl_ptls_t ptls = jl_get_ptls_states();

    /* first check for sticky tasks */
    jl_ptask_t *task = get_from_taskq();

    /* no sticky tasks, go to the multiq */
    if (task == NULL) {
        task = multiq_deletemin();
        if (task == NULL)
            return 0;

        /* a sticky task will only come out of the multiq if it has not been run */
        if (task->settings & TASK_IS_STICKY) {
            assert(task->sticky_tid == -1);
            task->sticky_tid = ptls->tid;
        }
    }

    /* run/resume the task */
    ptls->curr_task = task;
    int64_t y = 0; // TODO. (int64_t)resume(task->ctx);
    ptls->curr_task = NULL;

    /* if the task isn't done, it is either in a CQ, or must be re-queued */
    if (0 /* TODO. !ctx_is_done(task->ctx) */) {
        /* the yield value tells us if the task is in a CQ */
        if (y != yield_from_sync) {
            /* sticky tasks go to the thread's sticky queue */
            if (task->settings & TASK_IS_STICKY)
                add_to_taskq(task);
            /* all others go back into the multiq */
            else
                multiq_insert(task, task->prio);
        }
        return 0;
    }

    /* The task completed. As detached tasks cannot be synced, clean
       those up here.
     */
    if (task->settings & TASK_IS_DETACHED) {
        release_task(task);
        return 0;
    }

    /* add back all the tasks in this one's completion queue */
    while (__atomic_test_and_set(&task->cq_lock, __ATOMIC_ACQUIRE))
        jl_cpu_pause();
    jl_ptask_t *cqtask, *cqnext;
    cqtask = task->cq;
    task->cq = NULL;
    while (cqtask) {
        cqnext = cqtask->next;
        cqtask->next = NULL;
        if (cqtask->settings & TASK_IS_STICKY)
            add_to_taskq(cqtask);
        else
            multiq_insert(cqtask, cqtask->prio);
        cqtask = cqnext;
    }
    __atomic_clear(&task->cq_lock, __ATOMIC_RELEASE);

    return 0;
}


/*  partr_start() -- the runtime entry point

    To be called from thread 0, before creating any tasks. Wraps into
    a task and invokes `f(arg)`; tasks should only be spawned/synced
    from within tasks.
 */
int partr_start(void **ret, void *(*f)(void *, int64_t, int64_t),
        void *arg, int64_t start, int64_t end)
{
    assert(tid == 0);

    jl_ptls_t ptls = jl_get_ptls_states();

    start_task = setup_task(f, arg, start, end);
    if (start_task == NULL)
        return -1;
    start_task->settings |= TASK_IS_STICKY;
    start_task->sticky_tid = ptls->tid;

    ptls->curr_task = start_task;
    int64_t y = 0; // TODO. (int64_t)resume(start_task->ctx);
    ptls->curr_task = NULL;

    if (0 /* TODO. !ctx_is_done(start_task->ctx) */) {
        if (y != yield_from_sync) {
            add_to_taskq(start_task);
        }
        while (run_next() == 0)
            if (0 /* TODO. ctx_is_done(start_task->ctx) */)
                break;
    }

    void *r = release_task(start_task);
    if (ret)
        *ret = r;

    return 0;
}


/*  partr_spawn() -- create a task for `f(arg)` and enqueue it for execution

    Implicitly asserts that `f(arg)` can run concurrently with everything
    else that's currently running. If `detach` is set, the spawned task
    will not be returned (and cannot be synced). Yields.
 */
int partr_spawn(partr_t *t, void *(*f)(void *, int64_t, int64_t),
        void *arg, int64_t start, int64_t end, int8_t sticky, int8_t detach)
{
    jl_ptls_t ptls = jl_get_ptls_states();

    jl_ptask_t *task = setup_task(f, arg, start, end);
    if (task == NULL)
        return -1;
    if (sticky)
        task->settings |= TASK_IS_STICKY;
    if (detach)
        task->settings |= TASK_IS_DETACHED;

    if (multiq_insert(task, ptls->tid) != 0) {
        release_task(task);
        return -2;
    }

    *t = detach ? NULL : (partr_t)task;

    /* only yield if we're running a non-sticky task */
    if (!(ptls->curr_task->settings & TASK_IS_STICKY))
        // TODO. yield(ptls->curr_task->ctx);
        ;

    return 0;
}


/*  partr_sync() -- get the return value of task `t`

    Returns only when task `t` has completed.
 */
int partr_sync(void **r, partr_t t, int done_with_task)
{
    jl_ptask_t *task = (jl_ptask_t *)t;

    jl_ptls_t ptls = jl_get_ptls_states();

    /* if the target task has not finished, add the current task to its
       completion queue; the thread that runs the target task will add
       this task back to the ready queue
     */
    if (0 /* TODO. !ctx_is_done(task->ctx) */) {
        ptls->curr_task->next = NULL;
        while (jl_atomic_test_and_set(&task->cq_lock))
            jl_cpu_pause();

        /* ensure the task didn't finish before we got the lock */
        if (0 /* TODO. !ctx_is_done(task->ctx) */) {
            /* add the current task to the CQ */
            if (task->cq == NULL)
                task->cq = ptls->curr_task;
            else {
                jl_ptask_t *pt = task->cq;
                while (pt->next)
                    pt = pt->next;
                pt->next = ptls->curr_task;
            }

            /* unlock the CQ and yield the current task */
            jl_atomic_clear(&task->cq_lock);
            // TODO. yield_value(ptls->curr_task->ctx, (void *)yield_from_sync);
        }

        /* the task finished before we could add to its CQ */
        else
            jl_atomic_clear(&task->cq_lock);
    }

    if (r)
        *r = task->grain_num >= 0 && task->red ?
                task->red_result : task->result;

    if (done_with_task)
        release_task(task);

    return 0;
}


/*  partr_parfor() -- spawn multiple tasks for a parallel loop

    Spawn tasks that invoke `f(arg, start, end)` such that the sum of `end-start`
    for all tasks is `count`. Uses `rf()`, if provided, to reduce the return
    values from the tasks, and returns the result. Yields.
 */
int partr_parfor(partr_t *t, void *(*f)(void *, int64_t, int64_t),
        void *arg, int64_t count, void *(*rf)(void *, void *))
{
    jl_ptls_t ptls = jl_get_ptls_states();

    int64_t n = GRAIN_K * jl_n_threads;
    lldiv_t each = lldiv(count, n);

    /* allocate synchronization tree(s) */
    arriver_t *arr = arriver_alloc();
    if (arr == NULL)
        return -1;
    reducer_t *red = NULL;
    if (rf != NULL) {
        red = reducer_alloc();
        if (red == NULL) {
            arriver_free(arr);
            return -2;
        }
    }

    /* allocate and enqueue (GRAIN_K * nthreads) tasks */
    *t = NULL;
    int64_t start = 0, end;
    for (int64_t i = 0;  i < n;  ++i) {
        end = start + each.quot + (i < each.rem ? 1 : 0);
        jl_ptask_t *task = setup_task(f, arg, start, end);
        if (task == NULL)
            return -1;

        /* The first task is the parent (root) task of the parfor, thus only
           this can be synced. So, we create the remaining tasks detached.
         */
        if (*t == NULL) *t = task;
        else task->settings = TASK_IS_DETACHED;

        task->parent = *t;
        task->grain_num = i;
        task->rf = rf;
        task->arr = arr;
        task->red = red;

        if (multiq_insert(task, ptls->tid) != 0) {
            release_task(task);
            return -3;
        }

        start = end;
    }

    /* only yield if we're running a non-sticky task */
    if (!(ptls->curr_task->settings & TASK_IS_STICKY))
        // TODO. yield(curr_task->ctx);
        ;

    return 0;
}


#endif // JULIA_ENABLE_PARTR
#endif // JULIA_ENABLE_THREADING

#ifdef __cplusplus
}
#endif
