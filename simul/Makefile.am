###CPPFLAGS+= @XCF_CPPFLAGS@ @ESTAR_CPPFLAGS@ -I@abs_top_builddir@/include
CPPFLAGS+= @XCF_CPPFLAGS@ @ESTAR_CPPFLAGS@

if SMARTFLOWER
  LDFLAGS+= @GLUTLIBS@
else
  LDFLAGS+= @GLUTLIBS@ @XCF_LDFLAGS@ @ESTAR_LDFLAGS@
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
  nepumuk_LDADD=   ../libnepumuk.la -lasl-mcontrol -lsmartsfl -lsunflower -lestar
endif

else

if SMARTFLOWER
  nepumuk_LDADD= ../libnepumuk.la \
                 ../../sunflower/libsunflower.la
else
  nepumuk_LDADD=   ../libnepumuk.la -lsunflower
endif

endif

dumpworld_SOURCES= dumpworld.cpp Interlock.cpp

if SMARTFLOWER
  dumpworld_LDADD= ../libnepumuk.la \
                   ../../sunflower/libsunflower.la
else
  dumpworld_LDADD= ../libnepumuk.la -lsunflower
endif

dist_bin_SCRIPTS= birondemo.sh \
                  estardemo.sh \
                  expodemo.sh \
                  smartdemo.sh \
		  smartphysical.sh \
                  smarttest.sh \
                  testestar.sh \
                  layout.biron \
                  layout.config \
                  layout.estar \
                  layout.expo \
                  layout.smart \
                  layout.smarttest \
                  robots.biron \
                  robots.config \
                  robots.estar \
                  robots.expo \
                  robots.smart \
                  robots.smarttest \
                  robots.physical \
                  smart.trm \
                  trav.map \
                  travCS.map \
                  travmap-example.txt
