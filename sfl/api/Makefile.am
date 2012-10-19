# AM_CPPFLAGS= @SFL2_CPPFLAGS@
# AM_CFLAGS=   @SFL2_CFLAGS@
# AM_CXXFLAGS= @SFL2_CXXFLAGS@
# AM_LDFLAGS=  @SFL2_LDFLAGS@

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
LDADD=                   ../libsfl2.la
testFrameFusion_SOURCES= testFrameFusion.cpp
testTimestamp_SOURCES=   testTimestamp.cpp

../libsfl2.la:
	$(MAKE) -C ..
