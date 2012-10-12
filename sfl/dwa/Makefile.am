AM_CPPFLAGS= @SFL_CPPFLAGS@
AM_CFLAGS=   @SFL_CFLAGS@
AM_CXXFLAGS= @SFL_CXXFLAGS@
AM_LDFLAGS=  @SFL_LDFLAGS@

noinst_LTLIBRARIES= libdwa.la
libdwa_la_SOURCES=  SpeedObjective.cpp Objective.cpp Lookup.cpp \
                    DynamicWindow.cpp HeadingObjective.cpp \
                    DistanceObjective.cpp DistanceObjectiveToBI.cpp
include_HEADERS=    SpeedObjective.hpp Lookup.hpp Objective.hpp \
                    HeadingObjective.hpp DistanceObjective.hpp DistanceObjectiveToBI.hpp \
                    DynamicWindow.hpp
includedir=         @includedir@/sfl/dwa
