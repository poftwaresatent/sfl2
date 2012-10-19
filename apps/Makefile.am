AM_CPPFLAGS= -I@abs_top_srcdir@ @SFL2_CPPFLAGS@
AM_CFLAGS=   @SFL2_CFLAGS@
AM_CXXFLAGS= @SFL2_CXXFLAGS@
AM_LDFLAGS=  @SFL2_LDFLAGS@ @SFL2_GLUTLIBS@

noinst_LTLIBRARIES=   libnpmbin.la
libnpmbin_la_SOURCES= Simulator.cpp

bin_PROGRAMS=         nepumuk dumpworld xfig2world

nepumuk_SOURCES=      nepumuk.cpp Argtool.cpp
nepumuk_LDADD=        libnpmbin.la \
                      ../sfl/libsfl2.la \
                      ../npm/libnpm.la

dumpworld_SOURCES=    dumpworld.cpp Argtool.cpp
dumpworld_LDADD=      libnpmbin.la \
                      ../sfl/libsfl2.la \
                      ../npm/libnpm.la

xfig2world_SOURCES=   xfig2world.cpp

dist_bin_SCRIPTS=     test0.sh layout.test0 robots.test0 \
                      test1.sh layout.test1 robots.test1
