AM_CPPFLAGS= @NPM_CPPFLAGS@
AM_CFLAGS=   @NPM_CFLAGS@
AM_CXXFLAGS= @NPM_CXXFLAGS@
AM_LDFLAGS=  @NPM_LDFLAGS@

noinst_LTLIBRARIES=  librobox.la
librobox_la_SOURCES= DODrawing.cpp BBDrawing.cpp BLDrawing.cpp OCamera.cpp \
                     GridLayerCamera.cpp DWDrawing.cpp ODrawing.cpp \
                     RHDrawing.cpp GridLayerDrawing.cpp MPDrawing.cpp \
                     Robox.cpp VisualRobox.cpp robox_parameters.cpp

include_HEADERS= BBDrawing.hpp BLDrawing.hpp DODrawing.hpp \
                 DWDrawing.hpp GridLayerCamera.hpp \
                 GridLayerDrawing.hpp MPDrawing.hpp OCamera.hpp \
                 ODrawing.hpp RHDrawing.hpp Robox.hpp VisualRobox.hpp \
                 robox_parameters.hpp

includedir=      @includedir@/npm/robox
