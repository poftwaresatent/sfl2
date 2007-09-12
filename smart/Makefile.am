CPPFLAGS= @NPM_CPPFLAGS@ @NPM_ESTAR_CPPFLAGS@
CFLAGS=   @NPM_CFLAGS@
CXXFLAGS= @NPM_CXXFLAGS@
LDFLAGS=  @NPM_LDFLAGS@

if NPM_CARMEN_ELROB
SMART_CARMEN_CPP=SmartCarmen.cpp wrap_carmen.cpp	
SMART_CARMEN_HPP=SmartCarmen.hpp wrap_carmen.hpp
else
SMART_CARMEN_CPP=
SMART_CARMEN_HPP=
endif

noinst_LTLIBRARIES=  libsmart.la
libsmart_la_SOURCES= Smart.cpp PathDrawing.cpp ArcDrawing.cpp \
                     $(SMART_CARMEN_CPP)
include_HEADERS=     ArcDrawing.hpp PathDrawing.hpp Smart.hpp \
                     ThreadDrawing.hpp $(SMART_CARMEN_HPP)
includedir=          @includedir@/npm/smart
