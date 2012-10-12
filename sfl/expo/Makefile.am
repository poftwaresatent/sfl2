AM_CPPFLAGS= @SFL_CPPFLAGS@
AM_CFLAGS=   @SFL_CFLAGS@
AM_CXXFLAGS= @SFL_CXXFLAGS@
AM_LDFLAGS=  @SFL_LDFLAGS@

noinst_LTLIBRARIES= libexpo.la
libexpo_la_SOURCES= MotionPlannerState.cpp MotionPlanner.cpp \
                    MotionController.cpp expo_parameters.cpp \
                    Robox.cpp
include_HEADERS=    MotionPlanner.hpp MotionPlannerState.hpp \
                    MotionController.hpp expo_parameters.h \
                    Robox.hpp
includedir=         @includedir@/sfl/expo
