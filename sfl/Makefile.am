AM_CPPFLAGS= -I@abs_top_srcdir@ @SFL2_CPPFLAGS@
AM_CFLAGS=   @SFL2_CFLAGS@
AM_CXXFLAGS= @SFL2_CXXFLAGS@
AM_LDFLAGS=  @SFL2_LDFLAGS@

SUBDIRS=                 api bband dwa expo gplan util

lib_LTLIBRARIES=         libsfl2.la

libsfl2_la_SOURCES= api/FrameFusion.cpp \
                    api/Goal.cpp \
                    api/MotionController.cpp \
                    api/Multiscanner.cpp \
                    api/Odometry.cpp \
                    api/Pose.cpp \
                    api/RobotModel.cpp \
                    api/Scan.cpp \
                    api/Scanner.cpp \
                    api/Timestamp.cpp \
                    bband/Bubble.cpp \
                    bband/BubbleBand.cpp \
                    bband/BubbleFactory.cpp \
                    bband/BubbleList.cpp \
                    bband/ReplanHandler.cpp \
                    dwa/DistanceObjective.cpp \
                    dwa/DistanceObjectiveToBI.cpp \
                    dwa/DynamicWindow.cpp \
                    dwa/HeadingObjective.cpp \
                    dwa/Lookup.cpp \
                    dwa/Objective.cpp \
                    dwa/SpeedObjective.cpp \
                    expo/expo_parameters.cpp \
                    expo/expoMotionController.cpp \
                    expo/MotionPlanner.cpp \
                    expo/MotionPlannerState.cpp \
                    expo/Robox.cpp \
                    gplan/GridFrame.cpp \
                    gplan/Mapper2d.cpp \
                    gplan/NF1.cpp \
                    gplan/NF1Wave.cpp \
                    gplan/TraversabilityMap.cpp \
                    util/Frame.cpp \
                    util/GoalManager.cpp \
                    util/Hull.cpp \
                    util/Line.cpp \
                    util/numeric.cpp \
                    util/OptionDictionary.cpp \
                    util/Point.cpp \
                    util/Polygon.cpp \
                    util/Pthread.cpp \
                    util/Random.cpp \
                    util/Ray.cpp \
                    util/strutil.cpp

