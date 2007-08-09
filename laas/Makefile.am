CPPFLAGS= @NPM_CPPFLAGS@
CFLAGS=   @NPM_CFLAGS@
CXXFLAGS= @NPM_CXXFLAGS@
LDFLAGS=  @NPM_LDFLAGS@

if ! NPM_HAVE_GENOM
  BUILT_SOURCES= genom_config_error
endif

noinst_PROGRAMS= pwrite pread
pwrite_SOURCES=  pwrite.cpp
pwrite_LDADD=    liblaas.la
pread_SOURCES=   pread.cpp
pread_LDADD=     liblaas.la

noinst_LTLIBRARIES= liblaas.la
liblaas_la_SOURCES= LAAS.cpp GenomBridge.cpp phelp.cpp
include_HEADERS=    GenomBridge.hpp LAAS.hpp phelp.hpp
includedir=         @includedir@/npm/laas

genom_config_error:
	@echo "ERROR in configuration: should not build in this subdir"
	@echo "  unless --with-genom has been given to the configure script"
	@echo "  this is a bug in the automake files, please inform the"
	@echo "  maintainers listed in the README file"
	exit 42
