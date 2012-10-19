AM_CPPFLAGS= -I@abs_top_srcdir@ @SFL2_CPPFLAGS@
AM_CFLAGS=   @SFL2_CFLAGS@
AM_CXXFLAGS= @SFL2_CXXFLAGS@
AM_LDFLAGS=  @SFL2_LDFLAGS@

noinst_LTLIBRARIES=    libgplan.la

libgplan_la_SOURCES=   GridFrame.cpp \
                       NF1Wave.cpp \
                       NF1.cpp \
                       TraversabilityMap.cpp \
                       Mapper2d.cpp

include_HEADERS=       GridFrame.hpp \
                       NF1Wave.hpp \
                       NF1.hpp \
                       TraversabilityMap.hpp \
                       RWTravmap.hpp \
                       Mapper2d.hpp

includedir=            @includedir@/sfl/gplan
