## MPFR ##

ifeq ($(USE_SYSTEM_GMP), 0)
$(BUILDDIR)/mpfr-$(MPFR_VER)/build-configured: | $(build_prefix)/manifest/gmp
endif

ifeq ($(USE_SYSTEM_MPFR), 0)
ifeq ($(USE_SYSTEM_GMP), 0)
MPFR_OPTS := --with-gmp-include=$(abspath $(build_includedir)) --with-gmp-lib=$(abspath $(build_shlibdir))
endif
endif
ifeq ($(BUILD_OS),WINNT)
ifeq ($(OS),WINNT)
MPFR_OPTS += --disable-thread-safe CFLAGS="$(CFLAGS) -DNPRINTF_L -DNPRINTF_T -DNPRINTF_J"
endif
endif


ifeq ($(OS),Darwin)
MPFR_CHECK_MFLAGS := LDFLAGS="$(LDFLAGS) -Wl,-rpath,'$(build_libdir)'"
endif

ifeq ($(SANITIZE),1)
# Force generic C build
MPFR_OPTS += --host=none-unknown-linux
endif

$(SRCDIR)/srccache/mpfr-$(MPFR_VER).tar.bz2: | $(SRCDIR)/srccache
	$(JLDOWNLOAD) $@ http://www.mpfr.org/mpfr-$(MPFR_VER)/$(notdir $@)
$(SRCDIR)/srccache/mpfr-$(MPFR_VER)/source-extracted: $(SRCDIR)/srccache/mpfr-$(MPFR_VER).tar.bz2
	$(JLCHECKSUM) $<
	cd $(dir $<) && $(TAR) -jxf $<
	touch -c $(SRCDIR)/srccache/mpfr-$(MPFR_VER)/configure # old target
	echo 1 > $@

$(SRCDIR)/srccache/mpfr-$(MPFR_VER)/build-patched: $(SRCDIR)/srccache/mpfr-$(MPFR_VER)/source-extracted
	cd $(dir $@) && patch -p0 < $(SRCDIR)/patches/mpfr-sincos.patch
	echo 1 > $@

$(BUILDDIR)/mpfr-$(MPFR_VER)/build-configured: $(SRCDIR)/srccache/mpfr-$(MPFR_VER)/source-extracted $(SRCDIR)/srccache/mpfr-$(MPFR_VER)/build-patched
	mkdir -p $(dir $@)
	cd $(dir $@) && \
	$(dir $<)/configure $(CONFIGURE_COMMON) $(MPFR_OPTS) F77= --enable-shared --disable-static
	echo 1 > $@

$(BUILDDIR)/mpfr-$(MPFR_VER)/build-compiled: $(BUILDDIR)/mpfr-$(MPFR_VER)/build-configured
	$(MAKE) -C $(dir $<) $(LIBTOOL_CCLD)
	echo 1 > $@

$(BUILDDIR)/mpfr-$(MPFR_VER)/build-checked: $(BUILDDIR)/mpfr-$(MPFR_VER)/build-compiled
ifeq ($(OS),$(BUILD_OS))
	$(MAKE) -C $(dir $@) $(LIBTOOL_CCLD) check $(MPFR_CHECK_MFLAGS)
endif
	echo 1 > $@

$(eval $(call staged-install, \
	mpfr,mpfr-$(MPFR_VER), \
	MAKE_INSTALL,$$(LIBTOOL_CCLD),, \
	$$(INSTALL_NAME_CMD)libmpfr.$$(SHLIB_EXT) $$(build_shlibdir)/libmpfr.$$(SHLIB_EXT)))

clean-mpfr:
	-rm $(BUILDDIR)/mpfr-$(MPFR_VER)/build-configured $(BUILDDIR)/mpfr-$(MPFR_VER)/build-compiled
	-$(MAKE) -C $(BUILDDIR)/mpfr-$(MPFR_VER) clean

distclean-mpfr:
	-rm -rf $(SRCDIR)/srccache/mpfr-$(MPFR_VER).tar.bz2 \
		$(SRCDIR)/srccache/mpfr-$(MPFR_VER) \
		$(BUILDDIR)/mpfr-$(MPFR_VER)

get-mpfr: $(SRCDIR)/srccache/mpfr-$(MPFR_VER).tar.bz2
extract-mpfr: $(SRCDIR)/srccache/mpfr-$(MPFR_VER)/source-extracted
configure-mpfr: $(BUILDDIR)/mpfr-$(MPFR_VER)/build-configured
compile-mpfr: $(BUILDDIR)/mpfr-$(MPFR_VER)/build-compiled
fastcheck-mpfr: check-mpfr
check-mpfr: $(BUILDDIR)/mpfr-$(MPFR_VER)/build-checked
