CPPFLAGS= @NPM_CPPFLAGS@ @NPM_XCF_CPPFLAGS@ @NPM_ESTAR_CPPFLAGS@
CFLAGS=   @NPM_CFLAGS@
CXXFLAGS= @NPM_CXXFLAGS@
LDFLAGS=  @NPM_LDFLAGS@ @NPM_GLUTLIBS@

noinst_LTLIBRARIES=  libsimul.la
libsimul_la_SOURCES= RobotFactory.cpp Simulator.cpp

bin_PROGRAMS=      nepumuk dumpworld
nepumuk_SOURCES=   nepumuk.cpp Interlock.cpp
dumpworld_SOURCES= dumpworld.cpp Interlock.cpp

include_HEADERS=   Interlock.hpp RobotFactory.hpp Simulator.hpp
includedir=        @includedir@/npm/simul

# ===> see dist_bin_SCRIPTS for more stuff

if NPM_HAVE_ESTAR

if NPM_SMARTFLOWER
  nepumuk_LDADD=   libsimul.la \
                   ../libnepumuk.la \
                   ../../asl-mplan/libasl-mplan.la \
                   ../../asl-mcontrol/libasl-mcontrol.la \
                   ../../sunflower/libsunflower.la \
                   ../../estar/libestar.la
  dumpworld_LDADD= libsimul.la \
                   ../libnepumuk.la \
                   ../../sunflower/libsunflower.la
else
# ! NPM_SMARTFLOWER
  nepumuk_LDADD=   libsimul.la \
                   ../libnepumuk.la \
                   -lsunflower \
                   -lestar
  dumpworld_LDADD= libsimul.la \
                   ../libnepumuk.la \
                   -lsunflower
endif
# NPM_SMARTFLOWER

else
# ! NPM_HAVE_ESTAR

if NPM_SMARTFLOWER
  nepumuk_LDADD=   libsimul.la \
                   ../libnepumuk.la \
                   ../../sunflower/libsunflower.la
  dumpworld_LDADD= libsimul.la \
                   ../libnepumuk.la \
                   ../../sunflower/libsunflower.la
else
# ! NPM_SMARTFLOWER
  nepumuk_LDADD=   libsimul.la \
                   ../libnepumuk.la \
                   -lsunflower
  dumpworld_LDADD= libsimul.la \
                   ../libnepumuk.la \
                   -lsunflower
endif
# NPM_SMARTFLOWER

endif
# NPM_HAVE_ESTAR

# ===> scripts and config files

dist_bin_SCRIPTS= birondemo.sh \
                  estardemo.sh \
                  expodemo.sh \
                  testestar.sh \
                  layout.biron \
                  layout.laas \
                  layout.estar \
                  layout.expo \
                  robots.biron \
                  robots.laas \
                  robots.estar \
                  robots.expo \
                  smart0.sh \
                    layout.smart0 \
                    robots.smart0 \
                    travmap.smart0 \
                    goals.smart0 \
                    world.smart0 \
                  smart1.sh \
                    layout.smart1 \
                    robots.smart1 \
                    travmap.smart1 \
                    goals.smart1 \
                    world.smart1 \
                  smart2.sh \
                    layout.smart2 \
                    robots.smart2 \
                    travmap.smart2 \
                    goals.smart2 \
                    world.smart2 \
                  smart3.sh \
                    layout.smart3 \
                    robots.smart3 \
                    travmap.smart3 \
                    goals.smart3 \
                    world.smart3 \
                  smart4.sh \
                    layout.smart4 \
                    robots.smart4 \
                    travmap.smart4 \
                    goals.smart4 \
                    world.smart4 \
                  smart5.sh \
                    layout.smart5 \
                    robots.smart5 \
                    travmap.smart5 \
                    goals.smart5 \
                    world.smart5
