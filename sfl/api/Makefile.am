AM_CPPFLAGS= @SFL_CPPFLAGS@
AM_CFLAGS=   @SFL_CFLAGS@
AM_CXXFLAGS= @SFL_CXXFLAGS@
AM_LDFLAGS=  @SFL_LDFLAGS@

noinst_LTLIBRARIES= libapi.la
libapi_la_SOURCES=  Scan.cpp Goal.cpp Timestamp.cpp MotionController.cpp \
                    Pose.cpp RobotModel.cpp Scanner.cpp Odometry.cpp \
                    Multiscanner.cpp FrameFusion.cpp
include_HEADERS=    Pose.hpp Scan.hpp Multiscanner.hpp RobotModel.hpp \
                    Goal.hpp Scanner.hpp MotionPlanner.hpp \
                    MotionController.hpp Odometry.hpp Timestamp.hpp HAL.hpp \
                    FrameFusion.hpp
includedir=         @includedir@/sfl/api

check_PROGRAMS=          testFrameFusion testTimestamp
LDADD=                   ../libsunflower.la
testFrameFusion_SOURCES= testFrameFusion.cpp
testTimestamp_SOURCES=   testTimestamp.cpp

../libsunflower.la:
	$(MAKE) -C ..
