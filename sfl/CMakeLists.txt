### Simplified CMake build file for Sunflower.
###
### Works only if the top-level directory is called 'sfl' and if the
### E* code lives side-by-side with sfl in a directory called 'estar'.

PROJECT (Sunflower)
SET (CMAKE_VERBOSE_MAKEFILE ON)

MESSAGE (STATUS "System: ${CMAKE_SYSTEM_NAME}")



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

### -Wundef can sometimes help you find errors when porting files from
### Windows, but it can be *very* noisy in Boost headers
# CHECK_CXX_COMPILER_FLAG (-Wundef SFL_CXX_FLAG_Wundef)
# IF (SFL_CXX_FLAG_Wundef)
#   ADD_DEFINITIONS (-Wundef)
# ENDIF (SFL_CXX_FLAG_Wundef)



### Find required Boost library

INCLUDE (FindBoost)

IF (Boost_FOUND)
  MESSAGE (STATUS "Found Boost in ${Boost_INCLUDE_DIRS}")
  INCLUDE_DIRECTORIES (${Boost_INCLUDE_DIRS})
ELSE (Boost_FOUND)
  MESSAGE (FATAL_ERROR "Boost libraries not found")
ENDIF (Boost_FOUND)



### Some OS-dependent flags.

IF (WIN32)
  MESSAGE (STATUS "Detected Microsoft Windows")
  ADD_DEFINITIONS (-DWIN32)
ELSE (WIN32)
  IF (APPLE)
    MESSAGE (STATUS "Detected Mac OS X")
    ADD_DEFINITIONS (-DOSX)
  ENDIF (APPLE)
  
  IF (CMAKE_SYSTEM_NAME MATCHES Linux)
    MESSAGE (STATUS "Detected Linux")
    ADD_DEFINITIONS (-DLINUX)
  ENDIF (CMAKE_SYSTEM_NAME MATCHES Linux)
  
  IF (CMAKE_SYSTEM_NAME MATCHES OpenBSD)
    MESSAGE (STATUS "Detected OpenBSD")
    ADD_DEFINITIONS (-DOPENBSD)
  ENDIF (CMAKE_SYSTEM_NAME MATCHES OpenBSD)
ENDIF(WIN32)



SET (PREFIX /usr/local CACHE PATH "project installation prefix")



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
                     ../estar
                     ${PREFIX}/include)
