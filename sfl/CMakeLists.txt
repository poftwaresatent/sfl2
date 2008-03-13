### Simplified CMake build file for Sunflower.
###
### Works only if the top-level directory is called 'sfl' and if the
### E* code lives side-by-side with sfl in a directory called 'estar'.

PROJECT (Sunflower)
SET (CMAKE_VERBOSE_MAKEFILE ON)

MESSAGE ("System: ${CMAKE_SYSTEM_NAME}")



### Some compiler flags are not understood by all...

INCLUDE (CheckCXXCompilerFlag)

CHECK_CXX_COMPILER_FLAG (-O0 SFL_CXX_FLAG_O)
IF (SFL_CXX_FLAG_O)
  ADD_DEFINITIONS (-O0)
ENDIF (SFL_CXX_FLAG_O)

CHECK_CXX_COMPILER_FLAG (-g SFL_CXX_FLAG_g)
IF (SFL_CXX_FLAG_g)
  ADD_DEFINITIONS (-g)
ENDIF (SFL_CXX_FLAG_g)

CHECK_CXX_COMPILER_FLAG (-Wall SFL_CXX_FLAG_Wall)
IF (SFL_CXX_FLAG_Wall)
  ADD_DEFINITIONS (-Wall)
ENDIF (SFL_CXX_FLAG_Wall)

CHECK_CXX_COMPILER_FLAG (-Wno-long-double SFL_CXX_FLAG_Wnld)
IF (SFL_CXX_FLAG_Wnld)
  ADD_DEFINITIONS (-Wno-long-double)
ENDIF (SFL_CXX_FLAG_Wnld)

CHECK_CXX_COMPILER_FLAG (-Wundef SFL_CXX_FLAG_Wundef)
IF (SFL_CXX_FLAG_Wundef)
  ADD_DEFINITIONS (-Wundef)
ENDIF (SFL_CXX_FLAG_Wundef)


### Find required Boost libraries: Smart Pointers and Graph.

SET (PREFIX /usr/local          CACHE PATH "project installation prefix")
SET (BOOST_PATH ${PREFIX}/boost CACHE PATH "Boost installation prefix")

SET (CMAKE_REQUIRED_INCLUDES ${BOOST_PATH}/include ${PREFIX}/include)
INCLUDE_DIRECTORIES (${BOOST_PATH}/include ${PREFIX}/include)

INCLUDE (CheckIncludeFileCXX)

CHECK_INCLUDE_FILE_CXX (boost/shared_ptr.hpp SFL_HAVE_SHARED_PTR)
IF (${SFL_HAVE_SHARED_PTR})
ELSE (${SFL_HAVE_SHARED_PTR})
      MESSAGE(FATAL_ERROR "boost/shared_ptr.hpp not found")
ENDIF (${SFL_HAVE_SHARED_PTR})



### Some OS-dependent flags.

IF (WIN32)
  MESSAGE ("Detected Microsoft Windows")
  ADD_DEFINITIONS (-DWIN32)
ELSE (WIN32)
  IF (APPLE)
    MESSAGE ("Detected Mac OS X")
    ADD_DEFINITIONS (-DOSX)
  ENDIF (APPLE)
  
  IF (CMAKE_SYSTEM_NAME MATCHES Linux)
    MESSAGE ("Detected Linux")
    ADD_DEFINITIONS (-DLINUX)
  ENDIF (CMAKE_SYSTEM_NAME MATCHES Linux)
  
  IF (CMAKE_SYSTEM_NAME MATCHES OpenBSD)
    MESSAGE ("Detected OpenBSD")
    ADD_DEFINITIONS (-DOPENBSD)
  ENDIF (CMAKE_SYSTEM_NAME MATCHES OpenBSD)
ENDIF(WIN32)



ADD_LIBRARY (sunflower
             gplan/GridFrame.cpp
             gplan/NF1Wave.cpp
             gplan/NF1.cpp
             gplan/TraversabilityMap.cpp
             gplan/Mapper2d.cpp
             util/Point.cpp
             util/Ray.cpp
             util/Frame.cpp
             util/Line.cpp
             util/Pthread.cpp
             util/Hull.cpp
             util/numeric.cpp
             util/Polygon.cpp
             util/GoalManager.cpp
             dwa/SpeedObjective.cpp
             dwa/Objective.cpp
             dwa/Lookup.cpp
             dwa/DynamicWindow.cpp
             dwa/HeadingObjective.cpp
             dwa/DistanceObjective.cpp
             bband/ReplanHandler.cpp
             bband/BubbleBand.cpp
             bband/Bubble.cpp
             bband/BubbleFactory.cpp
             bband/BubbleList.cpp
             expo/MotionPlannerState.cpp
             expo/MotionPlanner.cpp
             expo/MotionController.cpp
             expo/expo_parameters.cpp
             api/Scan.cpp
             api/Goal.cpp
             api/Timestamp.cpp
             api/MotionController.cpp
             api/Pose.cpp
             api/RobotModel.cpp
             api/Scanner.cpp
             api/Odometry.cpp
             api/Multiscanner.cpp)

INCLUDE_DIRECTORIES (..
                     ../estar)
