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
  set (CMAKE_BUILD_TYPE Debug)
endif (NOT CMAKE_BUILD_TYPE)

if (CMAKE_BUILD_TYPE STREQUAL Debug)
  add_definitions (-DSFL2_DEBUG)
endif (CMAKE_BUILD_TYPE STREQUAL Debug)

### XXXX to do: some leftovers from GNU Automake setup
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
#
# Workaround for yaml-cpp shared lib issues: control everything from here...
#
#subdirs (
#  3rdparty/fpplib
#  )

##################################################
### The actual sources

file (GLOB         SFL2_YAMLCPP_SRCS "3rdparty/fpplib/yaml-cpp-0.2.7/src/*.cpp")
file (GLOB         SFL2_FPPLIB_SRCS  "3rdparty/fpplib/src/*.cpp")
file (GLOB_RECURSE SFL2_SFL_SRCS     "sfl/*.cpp")
file (GLOB_RECURSE SFL2_NPM_SRCS     "npm/*.cpp")

add_library (yamlcpp SHARED ${SFL2_YAMLCPP_SRCS})
add_library (fpplib  SHARED ${SFL2_FPPLIB_SRCS})
add_library (sfl     SHARED ${SFL2_SFL_SRCS})
add_library (npm     SHARED ${SFL2_NPM_SRCS})
target_link_libraries (fpplib yamlcpp)
target_link_libraries (sfl    pthread)
target_link_libraries (npm    fpplib sfl ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})

add_executable (nepumuk apps/nepumuk.cpp)
target_link_libraries (nepumuk npm)
