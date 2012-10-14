AM_CPPFLAGS= -I@abs_top_srcdir@ @NPM_CPPFLAGS@
AM_CFLAGS=   @NPM_CFLAGS@
AM_CXXFLAGS= @NPM_CXXFLAGS@
AM_LDFLAGS=  @NPM_LDFLAGS@ @NPM_GLUTLIBS@

noinst_LTLIBRARIES=   libnpmbin.la
libnpmbin_la_SOURCES= RobotFactory.cpp Simulator.cpp

bin_PROGRAMS=         nepumuk dumpworld xfig2world

nepumuk_SOURCES=      nepumuk.cpp Interlock.cpp
nepumuk_LDADD=        libnpmbin.la \
                      ../npm/libnpm.la \
                      -lsunflower

dumpworld_SOURCES=    dumpworld.cpp Interlock.cpp
dumpworld_LDADD=      libnpmbin.la \
                      ../npm/libnpm.la \
                      -lsunflower

xfig2world_SOURCES=   xfig2world.cpp

## dist_bin_SCRIPTS= estardemo.sh expodemo.sh ...
