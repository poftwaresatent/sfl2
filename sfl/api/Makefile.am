AM_CPPFLAGS= -I@abs_top_srcdir@ @SFL2_CPPFLAGS@
AM_CFLAGS=   @SFL2_CFLAGS@
AM_CXXFLAGS= @SFL2_CXXFLAGS@
AM_LDFLAGS=  @SFL2_LDFLAGS@

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
