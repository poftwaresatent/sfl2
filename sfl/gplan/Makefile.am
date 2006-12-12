if HAVE_ESTAR
  OPTSRC=              TraversabilityMapCS.cpp
  OPTHDR=              TraversabilityMapCS.hpp
endif

noinst_LTLIBRARIES=    libgplan.la

libgplan_la_SOURCES=   GridFrame.cpp \
                       NF1Wave.cpp \
                       NF1.cpp \
                       TraversabilityMap.cpp \
                       $(OPTSRC)

include_HEADERS=       GridFrame.cpp \
                       NF1Wave.cpp \
                       NF1.cpp \
                       TraversabilityMap.cpp \
                       $(OPTHDR)

includedir=            @includedir@/sfl/gplan
