CPPFLAGS= @NPM_CPPFLAGS@ @NPM_ESTAR_CPPFLAGS@
CFLAGS=   @NPM_CFLAGS@
CXXFLAGS= @NPM_CXXFLAGS@
LDFLAGS=  @NPM_LDFLAGS@

if ! NPM_HAVE_ESTAR
  BUILT_SOURCES= estar_config_error
endif

noinst_LTLIBRARIES=  libestar.la
libestar_la_SOURCES= CarrotDrawing.cpp Esbot.cpp PNF.cpp \
                     PNFCamera.cpp PNFDrawing.cpp EstarDrawing.cpp
include_HEADERS=     CarrotDrawing.hpp Esbot.hpp EstarDrawing.hpp \
                     PNF.hpp PNFCamera.hpp PNFDrawing.hpp
includedir=          @includedir@/npm/estar

estar_config_error:
	@echo "ERROR in configuration: should not build in this subdir"
	@echo "  unless --with-estar has been given to the configure script"
	@echo "  this is a bug in the automake files, please inform the"
	@echo "  maintainers listed in the README file"
	exit 42
