CPPFLAGS+= @XCF_CPPFLAGS@ @ESTAR_CPPFLAGS@

noinst_LTLIBRARIES=  libsimul.la
libsimul_la_SOURCES= RobotFactory.cpp Simulator.cpp

bin_PROGRAMS=      nepumuk dumpworld
nepumuk_SOURCES=   nepumuk.cpp Interlock.cpp
dumpworld_SOURCES= dumpworld.cpp Interlock.cpp

# ===> see dist_bin_SCRIPTS for more stuff

# ===> external libraries, depending on the build setup
# three cases: SMARTFLOWER, OPTPROGNAV, and all others

if SMARTFLOWER
  LDFLAGS+= @GLUTLIBS@
else

if OPTPROGNAV
  LDFLAGS+= @GLUTLIBS@
else
  LDFLAGS+= @GLUTLIBS@ @XCF_LDFLAGS@ @ESTAR_LDFLAGS@
endif

endif

# ===> internal libraries, depending on the build setup
# six cases for nepumuk: the same as for external libraries, but
#                        multiplied by two because of HAVE_ESTAR
# two cases for dumpworld: it only depends on libsunflower, which
#                          might be external or part of a meta-build

if HAVE_ESTAR

if SMARTFLOWER
  nepumuk_LDADD=   libsimul.la \
                   ../libnepumuk.la \
                   ../../smartsfl/libsmartsfl.la \
                   ../../asl-mcontrol/libasl-mcontrol.la \
                   ../../sunflower/libsunflower.la \
                   ../../estar/libestar.la
  dumpworld_LDADD= libsimul.la \
                   ../libnepumuk.la \
                   ../../sunflower/libsunflower.la
else
if OPTPROGNAV
  nepumuk_LDADD=   libsimul.la \
                   ../libnepumuk.la \
                   ../../sunflower/libsunflower.la \
                   ../../src/liboptprognav.la \
                   ../../estar/libestar.la
  dumpworld_LDADD= libsimul.la \
                   ../libnepumuk.la \
                   ../../sunflower/libsunflower.la
else
  nepumuk_LDADD=   libsimul.la \
                   ../libnepumuk.la \
                   -lasl-mcontrol \
                   -lsmartsfl \
                   -lsunflower \
                   -lestar
  dumpworld_LDADD= libsimul.la \
                   ../libnepumuk.la \
                   -lsunflower
endif

endif

else

if SMARTFLOWER
  nepumuk_LDADD=   libsimul.la \
                   ../libnepumuk.la \
                   ../../sunflower/libsunflower.la
  dumpworld_LDADD= libsimul.la \
                   ../libnepumuk.la \
                   ../../sunflower/libsunflower.la
else
if OPTPROGNAV
  nepumuk_LDADD=   libsimul.la \
                   ../libnepumuk.la \
                   ../../sunflower/libsunflower.la \
                   ../../src/liboptprognav.la
  dumpworld_LDADD= libsimul.la \
                   ../libnepumuk.la \
                   ../../sunflower/libsunflower.la
else
  nepumuk_LDADD=   libsimul.la \
                   ../libnepumuk.la \
                   -lsunflower
  dumpworld_LDADD= libsimul.la \
                   ../libnepumuk.la \
                   -lsunflower
endif

endif

endif

# ===> scripts and config files

dist_bin_SCRIPTS= birondemo.sh \
                  estardemo.sh \
                  expodemo.sh \
                  testestar.sh \
                  layout.biron \
                  layout.config \
                  layout.estar \
                  layout.expo \
                  robots.biron \
                  robots.config \
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
                    world.smart4
