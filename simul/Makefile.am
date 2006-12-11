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
