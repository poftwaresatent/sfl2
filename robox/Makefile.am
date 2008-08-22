CPPFLAGS= @NPM_CPPFLAGS@
CFLAGS=   @NPM_CFLAGS@
CXXFLAGS= @NPM_CXXFLAGS@
LDFLAGS=  @NPM_LDFLAGS@

noinst_LTLIBRARIES=  librobox.la
librobox_la_SOURCES= DODrawing.cpp BBDrawing.cpp BLDrawing.cpp OCamera.cpp \
                     GridLayerCamera.cpp DWDrawing.cpp ODrawing.cpp \
                     RHDrawing.cpp GridLayerDrawing.cpp MPDrawing.cpp \
                     Robox.cpp VisualRobox.cpp

include_HEADERS= BBDrawing.hpp BLDrawing.hpp DODrawing.hpp \
                 DWDrawing.hpp GridLayerCamera.hpp \
                 GridLayerDrawing.hpp MPDrawing.hpp OCamera.hpp \
                 ODrawing.hpp RHDrawing.hpp Robox.hpp VisualRobox.hpp

includedir=      @includedir@/npm/robox
