CPPFLAGS= @NPM_CPPFLAGS@ @NPM_ROS_CPPFLAGS@
CFLAGS=   @NPM_CFLAGS@
CXXFLAGS= @NPM_CXXFLAGS@
LDFLAGS=  @NPM_LDFLAGS@

if ! NPM_HAVE_ROS
  BUILT_SOURCES= ros_config_error
endif

noinst_LTLIBRARIES=  libros.la
libros_la_SOURCES=   ROSbot.cpp
include_HEADERS=     ROSbot.hpp

includedir=          @includedir@/npm/ros

bin_PROGRAMS=        test

test_SOURCES=        test.cpp
test_LDADD=          -lros

ros_config_error:
	@echo "ERROR in configuration: should not build in this subdir"
	@echo "  unless --with-ros has been given to the configure script"
	@echo "  this is a bug in the automake files, please inform the"
	@echo "  maintainers listed in the README file"
	exit 42
