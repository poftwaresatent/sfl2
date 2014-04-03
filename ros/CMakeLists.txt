cmake_minimum_required(VERSION 2.8.3)
project(sfl2)

set (CMAKE_VERBOSE_MAKEFILE ON)

find_package(catkin REQUIRED COMPONENTS
  cargo_ants_msgs
  roscpp
  message_generation
)

add_message_files(
  DIRECTORY msg
  FILES
  Scan.msg
)

generate_messages(
  DEPENDENCIES
  cargo_ants_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
  ..
  ../3rdparty/fpplib/include
  ../3rdparty/fpplib/yaml-cpp-0.2.7/include
  LIBRARIES npm
  CATKIN_DEPENDS roscpp
  #  DEPENDS system_lib
)

set (NPM_PLUGIN_PATH ${sfl2ros_BINARY_DIR}/plugins)
subdirs (..)

include_directories(
  ..
  ../3rdparty/fpplib/include
  ../3rdparty/fpplib/yaml-cpp-0.2.7/include
  ${catkin_INCLUDE_DIRS}
)

add_library (cargo-ants-plugin MODULE src/cargo-ants-plugin.cpp)
target_link_libraries (cargo-ants-plugin npm ${catkin_LIBRARIES})
set_target_properties (cargo-ants-plugin PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${NPM_PLUGIN_PATH})
