CPPFLAGS= @SFL_CPPFLAGS@
CFLAGS=   @SFL_CFLAGS@
CXXFLAGS= @SFL_CXXFLAGS@
LDFLAGS=  @SFL_LDFLAGS@

noinst_LTLIBRARIES= libexpo.la
libexpo_la_SOURCES= MotionPlannerState.cpp MotionPlanner.cpp \
                    MotionController.cpp expo_parameters.cpp \
                    Robox.cpp
include_HEADERS=    MotionPlanner.hpp MotionPlannerState.hpp \
                    MotionController.hpp expo_parameters.h \
                    Robox.hpp
includedir=         @includedir@/sfl/expo
