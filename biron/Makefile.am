if ! HAVE_XCF
  BUILT_SOURCES=xcf_config_error
endif
CPPFLAGS+= @XCF_CPPFLAGS@ -I@abs_top_builddir@/include
noinst_LTLIBRARIES=  libbiron.la
libbiron_la_SOURCES= Biron.cpp

xcf_config_error:
	@echo "ERROR in configuration: should not build in this subdir"
	@echo "  unless --with-xcf has been given to the configure script"
	@echo "  this is a bug in the automake files, please inform the"
	@echo "  maintainers listed in the README file"
	exit 42
