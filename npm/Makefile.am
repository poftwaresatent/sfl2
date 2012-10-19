AM_CPPFLAGS= -I@abs_top_srcdir@ @SFL2_CPPFLAGS@
AM_CFLAGS=   @SFL2_CFLAGS@
AM_CXXFLAGS= @SFL2_CXXFLAGS@
AM_LDFLAGS=  @SFL2_LDFLAGS@

SUBDIRS=              ext \
                      gfx

lib_LTLIBRARIES=      libnpm.la
libnpm_la_LIBADD=     ext/libext.la
libnpm_la_SOURCES=    BBox.cpp \
                      BicycleDrive.cpp \
                      CheatSheet.cpp \
                      DiffDrive.cpp \
                      Drive.cpp \
                      HAL.cpp \
                      HoloDrive.cpp \
                      Lidar.cpp \
                      Manageable.cpp \
                      Manager.cpp \
                      NoiseModel.cpp \
                      Object.cpp \
                      RobotClient.cpp \
                      RobotDescriptor.cpp \
                      RobotServer.cpp \
                      Sensor.cpp \
                      Sharp.cpp \
                      World.cpp \
                      gfx/BicycleDriveDrawing.cpp \
                      gfx/Camera.cpp \
                      gfx/DiffDriveDrawing.cpp \
                      gfx/Drawing.cpp \
                      gfx/GoalInstanceDrawing.cpp \
                      gfx/HoloDriveDrawing.cpp \
                      gfx/MapperUpdateDrawing.cpp \
                      gfx/OdometryDrawing.cpp \
                      gfx/RobotDrawing.cpp \
                      gfx/RobotZoomCamera.cpp \
                      gfx/ScannerDrawing.cpp \
                      gfx/SharpDrawing.cpp \
                      gfx/PNGImage.cpp \
                      gfx/StillCamera.cpp \
                      gfx/TrajectoryDrawing.cpp \
                      gfx/TraversabilityDrawing.cpp \
                      gfx/View.cpp \
                      gfx/WorldCamera.cpp \
                      gfx/WorldDrawing.cpp \
                      gfx/wrap_glu.cpp

include_HEADERS=      BBox.hpp \
                      BicycleDrive.hpp \
                      CheatSheet.hpp \
                      DiffDrive.hpp \
                      Drive.hpp \
                      HAL.hpp \
                      HoloDrive.hpp \
                      Lidar.hpp \
                      Manageable.hpp \
                      Manager.hpp \
                      NoiseModel.hpp \
                      Object.hpp \
                      RobotClient.hpp \
                      RobotDescriptor.hpp \
                      RobotServer.hpp \
                      Sensor.hpp \
                      Sharp.hpp \
                      World.hpp \
                      pdebug.hpp

includedir=           @includedir@/npm
