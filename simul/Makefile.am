CPPFLAGS+= @XCF_CPPFLAGS@ @ESTAR_CPPFLAGS@
LDFLAGS+= @GLUTLIBS@ @XCF_LDFLAGS@ @ESTAR_LDFLAGS@

bin_PROGRAMS=      nepumuk dumpworld
nepumuk_SOURCES=   nepumuk.cpp RobotFactory.cpp Interlock.cpp Simulator.cpp
if HAVE_ESTAR
  nepumuk_LDADD=   ../libnepumuk.la -lasl-mcontrol
else
  nepumuk_LDADD=   ../libnepumuk.la
endif

dumpworld_SOURCES= dumpworld.cpp Interlock.cpp
dumpworld_LDADD=   ../libnepumuk.la

dist_bin_SCRIPTS= birondemo.sh \
                  estardemo.sh \
                  expodemo.sh \
                  smartdemo.sh \
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
                  smart.trm \
                  trav.map \
                  travCS.map \
                  travmap-example.txt
