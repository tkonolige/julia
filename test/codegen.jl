# This file is a part of Julia. License is MIT: https://julialang.org/license

# tests for codegen and optimizations

const opt_level = Base.JLOptions().opt_level
const coverage = (Base.JLOptions().code_coverage > 0) || (Base.JLOptions().malloc_log > 0)
const Iptr = sizeof(Int) == 8 ? "i64" : "i32"

# `_dump_function` might be more efficient but it doesn't really matter here...
get_llvm(f::ANY, t::ANY, strip_ir_metadata=true, dump_module=false) =
    sprint(code_llvm, f, t, strip_ir_metadata, dump_module)

if opt_level > 0
    # Make sure getptls call is removed at IR level with optimization on
    @test !contains(get_llvm(identity, Tuple{String}), " call ")
end

jl_string_ptr(s::String) = ccall(:jl_string_ptr, Ptr{UInt8}, (Any,), s)
core_sizeof(o) = Core.sizeof(o)
function test_loads_no_call(ir, load_types)
    in_function = false
    load_idx = 1
    for line in eachline(IOBuffer(ir))
        if !in_function
            if startswith(line, "define ")
                in_function = true
            end
            continue
        end
        @test !contains(line, " call ")
        load_split = split(line, " load ", limit=2)
        if !coverage && length(load_split) >= 2
            @test load_idx <= length(load_types)
            if load_idx <= length(load_types)
                @test startswith(load_split[2], "$(load_types[load_idx]),")
            end
            load_idx += 1
        end
        if startswith(line, "}")
            break
        end
    end
    if !coverage
        @test load_idx == length(load_types) + 1
    end
end

# This function tests if functions are output when compiled if jl_dump_compiles is enabled.
# Have to go through pains with recursive function (eval probably not required) to make sure
# that inlining won't happen.
function test_jl_dump_compiles()
    tfile = tempname()
    io = open(tfile, "w")
    ccall(:jl_dump_compiles, Void, (Ptr{Void},), io.handle)
    eval(@noinline function test_jl_dump_compiles_internal(x)
        if x > 0
            test_jl_dump_compiles_internal(x-1)
        end
        end)
    test_jl_dump_compiles_internal(1)
    ccall(:jl_dump_compiles, Void, (Ptr{Void},), C_NULL)
    close(io)
    tstats = stat(tfile)
    tempty = tstats.size == 0
    rm(tfile)
    @test tempty == false
end

# This function tests if a toplevel thunk is output if jl_dump_compiles is enabled.
# The eval statement creates the toplevel thunk.
function test_jl_dump_compiles_toplevel_thunks()
    tfile = tempname()
    io = open(tfile, "w")
    ccall(:jl_dump_compiles, Void, (Ptr{Void},), io.handle)
    eval(expand(Main, :(for i in 1:10 end)))
    ccall(:jl_dump_compiles, Void, (Ptr{Void},), C_NULL)
    close(io)
    tstats = stat(tfile)
    tempty = tstats.size == 0
    rm(tfile)
    @test tempty == true
end

if opt_level > 0
    # Make sure `jl_string_ptr` is inlined
    @test !contains(get_llvm(jl_string_ptr, Tuple{String}), " call ")
    s = "aaa"
    @test jl_string_ptr(s) == pointer_from_objref(s) + sizeof(Int)
    # String
    test_loads_no_call(get_llvm(core_sizeof, Tuple{String}), [Iptr])
    # String
    test_loads_no_call(get_llvm(core_sizeof, Tuple{SimpleVector}), [Iptr])
    # Array
    test_loads_no_call(get_llvm(core_sizeof, Tuple{Vector{Int}}), [Iptr])
    # As long as the eltype is known we don't need to load the elsize
    test_loads_no_call(get_llvm(core_sizeof, Tuple{Array{Any}}), [Iptr])
    # Check that we load the elsize
    test_loads_no_call(get_llvm(core_sizeof, Tuple{Vector}), [Iptr, "i16"])

    test_jl_dump_compiles()
    test_jl_dump_compiles_toplevel_thunks()
end

@test !contains(get_llvm(isequal, Tuple{Nullable{BigFloat}, Nullable{BigFloat}}), "%gcframe")