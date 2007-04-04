###CPPFLAGS+= @XCF_CPPFLAGS@ @ESTAR_CPPFLAGS@ -I@abs_top_builddir@/include
CPPFLAGS+= @XCF_CPPFLAGS@ @ESTAR_CPPFLAGS@

if SMARTFLOWER
  LDFLAGS+= @GLUTLIBS@
else

if OPTPROGNAV
  LDFLAGS+= @GLUTLIBS@
else
  LDFLAGS+= @GLUTLIBS@ @XCF_LDFLAGS@ @ESTAR_LDFLAGS@
endif

endif

bin_PROGRAMS=      nepumuk dumpworld
nepumuk_SOURCES=   nepumuk.cpp RobotFactory.cpp Interlock.cpp Simulator.cpp

if HAVE_ESTAR

if SMARTFLOWER
  nepumuk_LDADD= ../libnepumuk.la \
                 ../../asl-mcontrol/libasl-mcontrol.la \
                 ../../smartsfl/libsmartsfl.la \
                 ../../sunflower/libsunflower.la \
                 ../../estar/libestar.la
else
if OPTPROGNAV
  nepumuk_LDADD= ../libnepumuk.la \
                 ../../sunflower/libsunflower.la \
                 ../../src/liboptprognav.la \
                 ../../estar/libestar.la
else
  nepumuk_LDADD=   ../libnepumuk.la -lasl-mcontrol -lsmartsfl -lsunflower -lestar
endif

endif

else

if SMARTFLOWER
  nepumuk_LDADD= ../libnepumuk.la \
                 ../../sunflower/libsunflower.la
else
if OPTPROGNAV
  nepumuk_LDADD= ../libnepumuk.la \
                 ../../sunflower/libsunflower.la \
                 ../../src/liboptprognav.la
else
  nepumuk_LDADD=   ../libnepumuk.la -lsunflower
endif

endif

endif

dumpworld_SOURCES= dumpworld.cpp Interlock.cpp

if SMARTFLOWER
  dumpworld_LDADD= ../libnepumuk.la \
                   ../../sunflower/libsunflower.la
else
if OPTPROGNAV
  dumpworld_LDADD= ../libnepumuk.la \
                   ../../sunflower/libsunflower.la
else
  dumpworld_LDADD= ../libnepumuk.la -lsunflower
endif

endif

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
                  smart1.sh \
                    layout.smart1 \
                    robots.smart1 \
                    travmap.smart1 \
                  smart2.sh \
                    layout.smart2 \
                    robots.smart2 \
                    travmap.smart2 \
                  smart3.sh \
                    layout.smart3 \
                    robots.smart3 \
                    travmap.smart3 \
                    goals.smart3 \
                    world.smart3
