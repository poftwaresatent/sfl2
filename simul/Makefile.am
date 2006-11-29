CPPFLAGS+= @XCF_CPPFLAGS@ @ESTAR_CPPFLAGS@
LDFLAGS+= @GLUTLIBS@ @XCF_LDFLAGS@ @ESTAR_LDFLAGS@

bin_PROGRAMS=      nepumuk dumpworld
nepumuk_SOURCES=   nepumuk.cpp RobotFactory.cpp Interlock.cpp Simulator.cpp
nepumuk_LDADD=     ../libnepumuk.la

dumpworld_SOURCES= dumpworld.cpp Interlock.cpp
dumpworld_LDADD=   ../libnepumuk.la
