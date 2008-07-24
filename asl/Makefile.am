CPPFLAGS= @NPM_CPPFLAGS@ @NPM_ESTAR_CPPFLAGS@
CFLAGS=   @NPM_CFLAGS@
CXXFLAGS= @NPM_CXXFLAGS@
LDFLAGS=  @NPM_LDFLAGS@

noinst_LTLIBRARIES=  libasl.la
libasl_la_SOURCES=   AslBot.cpp Smart.cpp Borox.cpp \
                     PathDrawing.cpp ArcDrawing.cpp \
                     SmartCarmen.cpp wrap_carmen.cpp smart_params.cpp
include_HEADERS=     AslBot.hpp Smart.hpp Borox.hpp \
                     ArcDrawing.hpp PathDrawing.hpp \
                     SmartCarmen.hpp ThreadDrawing.hpp wrap_carmen.hpp smart_params.hpp
includedir=          @includedir@/npm/asl
