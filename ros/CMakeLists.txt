cmake_minimum_required(VERSION 2.8.3)
project(sfl2)

set (CMAKE_VERBOSE_MAKEFILE ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
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
  ###  INCLUDE_DIRS ..
  LIBRARIES sfl
  CATKIN_DEPENDS roscpp
  #  DEPENDS system_lib
)

subdirs (..)

include_directories(.. ${catkin_INCLUDE_DIRS})

add_executable(foo foo.cpp)
target_link_libraries(foo sfl ${catkin_LIBRARIES})
