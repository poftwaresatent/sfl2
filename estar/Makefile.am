if ! HAVE_ESTAR
  BUILT_SOURCES= estar_config_error
endif
CPPFLAGS+= @ESTAR_CPPFLAGS@
noinst_LTLIBRARIES= libestar.la
libestar_la_SOURCES= CarrotDrawing.cpp Esbot.cpp PNF.cpp \
                     PNFCamera.cpp PNFDrawing.cpp EstarDrawing.cpp

estar_config_error:
	@echo "ERROR in configuration: should not build in this subdir"
	@echo "  unless --with-estar has been given to the configure script"
	@echo "  this is a bug in the automake files, please inform the"
	@echo "  maintainers listed in the README file"
	exit 42
