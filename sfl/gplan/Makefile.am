if HAVE_ESTAR
  OPTSRC=              TraversabilityMapCS.cpp \
											 Mapper2d.cpp
  OPTHDR=              TraversabilityMapCS.hpp \
											 Mapper2d.hpp
endif

noinst_LTLIBRARIES=    libgplan.la

libgplan_la_SOURCES=   GridFrame.cpp \
                       NF1Wave.cpp \
                       NF1.cpp \
                       TraversabilityMap.cpp \
                       $(OPTSRC)

include_HEADERS=       GridFrame.hpp \
                       NF1Wave.hpp \
                       NF1.hpp \
                       TraversabilityMap.hpp \
                       $(OPTHDR)

includedir=            @includedir@/sfl/gplan

###CPPFLAGS+=             -I@top_builddir@/include
