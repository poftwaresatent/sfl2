CPPFLAGS= @NPM_CPPFLAGS@ @NPM_ESTAR_CPPFLAGS@
CFLAGS=   @NPM_CFLAGS@
CXXFLAGS= @NPM_CXXFLAGS@
LDFLAGS=  @NPM_LDFLAGS@

noinst_LTLIBRARIES=  libsmart.la
libsmart_la_SOURCES= Smart.cpp PathDrawing.cpp ArcDrawing.cpp \
                     SmartCarmen.cpp wrap_carmen.cpp
include_HEADERS=     ArcDrawing.hpp PathDrawing.hpp Smart.hpp \
                     SmartCarmen.hpp ThreadDrawing.hpp wrap_carmen.hpp
includedir=          @includedir@/npm/smart
