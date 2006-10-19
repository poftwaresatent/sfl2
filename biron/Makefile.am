if ! BUILD_BIRON
  BUILT_SOURCES=xcf_config_error
endif
CPPFLAGS=          @CPPFLAGS@ @XCF_CPPFLAGS@ -I@prefix@/include
noinst_LIBRARIES=  libbiron.a
libbiron_a_SOURCES= Biron.cpp

xcf_config_error:
	@echo "ERROR in configuration: should not build in this subdir"
	@echo "  unless --with-xcf has been given to the configure script"
	@echo "  this is a bug in the automake files, please inform the"
	@echo "  maintainers listed in the README file"
	exit 42
