cmake_minimum_required (VERSION 2.6)
if (COMMAND cmake_policy)
  cmake_policy (SET CMP0003 NEW)
  cmake_policy (SET CMP0005 NEW)
  if (POLICY CMP0011)
    cmake_policy (SET CMP0011 NEW)
  endif (POLICY CMP0011)
endif (COMMAND cmake_policy)

##################################################
# basic setup

project (SFL2)
set (CMAKE_VERBOSE_MAKEFILE ON)
if (NOT BUILD_SHARED_LIBS)
  set (BUILD_SHARED_LIBS True)
endif (NOT BUILD_SHARED_LIBS)
if (NOT CMAKE_BUILD_TYPE)
  SET (CMAKE_BUILD_TYPE Debug)
endif (NOT CMAKE_BUILD_TYPE)

### XXXX to do: some leftovers from GNU Automake setup
# for Debug we should -DSFL2_DEBUG
# for PNG we should -DSFL2_HAVE_PNG
# the doc/Doxyfile.in should be (manually?) handled

##################################################
# OS detection

if (WIN32)
  add_definitions (-DWIN32)
else (WIN32)
  if (APPLE)
    add_definitions (-DOSX)
    include_directories (
      /opt/local/include
      )
    list (APPEND CMAKE_REQUIRED_INCLUDES
      /opt/local/include
      )
    link_directories (
      /opt/local/lib
      )
  endif (APPLE)
  if (CMAKE_SYSTEM_NAME MATCHES Linux)
    add_definitions (-DLINUX)
  endif (CMAKE_SYSTEM_NAME MATCHES Linux)
  if (CMAKE_SYSTEM_NAME MATCHES OpenBSD)
    add_definitions (-DOPENBSD)
    include_directories (
      /usr/local/include
      )
    link_directories (
      /usr/local/lib
      )
  endif (CMAKE_SYSTEM_NAME MATCHES OpenBSD)
endif(WIN32)

##################################################
# compiler flags (other than header and library paths)

include (CheckCXXCompilerFlag)

check_cxx_compiler_flag (-pipe CXX_FLAG_pipe)
if (CXX_FLAG_pipe)
  add_definitions (-pipe)
endif (CXX_FLAG_pipe)
check_cxx_compiler_flag (-Wall CXX_FLAG_Wall)
if (CXX_FLAG_Wall)
  add_definitions (-Wall)
endif (CXX_FLAG_Wall)

if (${CMAKE_BUILD_TYPE} STREQUAL "Debug")
  check_cxx_compiler_flag (-O0 CXX_FLAG_O0)
  if (CXX_FLAG_O0)
    add_definitions (-O0)
  endif (CXX_FLAG_O0)
endif (${CMAKE_BUILD_TYPE} STREQUAL "Debug")

##################################################
# configure-time checks

include (CheckIncludeFileCXX)

# find Boost (required, just the smart pointers though)

if (BOOST_DIR)
  list (APPEND CMAKE_REQUIRED_INCLUDES
    ${BOOST_DIR}/include
    ${BOOST_DIR}
    ${BOOST_DIR}/include/boost
    ${BOOST_DIR}/boost
    )
  include_directories (
    ${BOOST_DIR}/include
    ${BOOST_DIR}
    ${BOOST_DIR}/include/boost
    ${BOOST_DIR}/boost
    )
endif (BOOST_DIR)
check_include_file_cxx (boost/shared_ptr.hpp HAVE_BOOST)
if (HAVE_BOOST)
  message ("*** found Boost (required)")
else (HAVE_BOOST)
  message (FATAL_ERROR "Please install the Boost smart_ptr library, e.g. `sudo apt-get install libboost-dev' and/or tell me where to find it by passing -DBOOST_DIR:path=/path/to/boost to cmake next time (you probably need to clear its cache first).")
endif (HAVE_BOOST)

include (FindOpenGL)
if (OPENGL_FOUND)
  message ("*** found OpenGL (required)")
else (OPENGL_FOUND)
  message (FATAL_ERROR "Please install OpenGL.")
endif (OPENGL_FOUND)
if (OPENGL_GLU_FOUND)
  message ("*** found GLU (required)")
else (OPENGL_GLU_FOUND)
  message (FATAL_ERROR "Please install GLU.")
endif (OPENGL_GLU_FOUND)

include (FindGLUT)
if (GLUT_FOUND)
  message ("*** found GLUT (required)")
else (GLUT_FOUND)
  message (FATAL_ERROR "Please install GLUT.")
endif (GLUT_FOUND)

include_directories (
  .
  3rdparty/fpplib/include
  3rdparty/fpplib/yaml-cpp-0.2.7/include
  ${OPENGL_INCLUDE_DIR}
  ${GLUT_INCLUDE_DIR}
  )

##################################################
### Build 3rdparty stuff

subdirs (
  3rdparty/fpplib
  )

##################################################
### The actual sources

list (APPEND SFL2_SRCS
  sfl/api/FrameFusion.cpp
  sfl/api/Goal.cpp
  sfl/api/MotionController.cpp
  sfl/api/Multiscanner.cpp
  sfl/api/Odometry.cpp
  sfl/api/Pose.cpp
  sfl/api/RobotModel.cpp
  sfl/api/Scan.cpp
  sfl/api/Scanner.cpp
  sfl/api/Timestamp.cpp
  sfl/bband/Bubble.cpp
  sfl/bband/BubbleBand.cpp
  sfl/bband/BubbleFactory.cpp
  sfl/bband/BubbleList.cpp
  sfl/bband/ReplanHandler.cpp
  sfl/dwa/DistanceObjective.cpp
  sfl/dwa/DistanceObjectiveToBI.cpp
  sfl/dwa/DynamicWindow.cpp
  sfl/dwa/HeadingObjective.cpp
  sfl/dwa/Lookup.cpp
  sfl/dwa/Objective.cpp
  sfl/dwa/SpeedObjective.cpp
  sfl/expo/expo_parameters.cpp
  sfl/expo/expoMotionController.cpp
  sfl/expo/MotionPlanner.cpp
  sfl/expo/MotionPlannerState.cpp
  sfl/expo/Robox.cpp
  sfl/gplan/GridFrame.cpp
  sfl/gplan/Mapper2d.cpp
  sfl/gplan/NF1.cpp
  sfl/gplan/NF1Wave.cpp
  sfl/gplan/TraversabilityMap.cpp
  sfl/util/Frame.cpp
  sfl/util/GoalManager.cpp
  sfl/util/Hull.cpp
  sfl/util/Line.cpp
  sfl/util/numeric.cpp
  sfl/util/OptionDictionary.cpp
  sfl/util/Point.cpp
  sfl/util/Polygon.cpp
  sfl/util/Pthread.cpp
  sfl/util/Random.cpp
  sfl/util/Ray.cpp
  sfl/util/strutil.cpp
  npm/BBox.cpp
  npm/BicycleDrive.cpp
  npm/CheatSheet.cpp
  npm/DiffDrive.cpp
  npm/Drive.cpp
  npm/ext/RobotFactory.cpp
  npm/ext/Zombie.cpp
  npm/gfx/BicycleDriveDrawing.cpp
  npm/gfx/Camera.cpp
  npm/gfx/DiffDriveDrawing.cpp
  npm/gfx/Drawing.cpp
  npm/gfx/GoalInstanceDrawing.cpp
  npm/gfx/HoloDriveDrawing.cpp
  npm/gfx/MapperUpdateDrawing.cpp
  npm/gfx/OdometryDrawing.cpp
  npm/gfx/PNGImage.cpp
  npm/gfx/RobotDrawing.cpp
  npm/gfx/RobotZoomCamera.cpp
  npm/gfx/ScannerDrawing.cpp
  npm/gfx/SharpDrawing.cpp
  npm/gfx/StillCamera.cpp
  npm/gfx/TrajectoryDrawing.cpp
  npm/gfx/TraversabilityDrawing.cpp
  npm/gfx/View.cpp
  npm/gfx/WorldCamera.cpp
  npm/gfx/WorldDrawing.cpp
  npm/gfx/wrap_glu.cpp
  npm/HAL.cpp
  npm/HoloDrive.cpp
  npm/Lidar.cpp
  npm/NoiseModel.cpp
  npm/Object.cpp
  npm/RobotClient.cpp
  npm/RobotDescriptor.cpp
  npm/RobotServer.cpp
  npm/Sensor.cpp
  npm/Sharp.cpp
  npm/World.cpp
  apps/Argtool.cpp
  apps/Simulator.cpp
  )

add_library (sfl2 SHARED ${SFL2_SRCS})
target_link_libraries (sfl2 fpp yaml-cpp ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})

add_executable (nepumuk apps/nepumuk.cpp)
target_link_libraries (nepumuk sfl2)
