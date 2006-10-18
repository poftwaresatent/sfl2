GLUTLIBS=          @GLUTLIBS@
bin_PROGRAMS=      nepumuk dumpworld
nepumuk_SOURCES=   nepumuk.cpp RobotFactory.cpp Interlock.cpp Simulator.cpp
nepumuk_LDADD=     ../common/libcommon.a $(GLUTLIBS) -lsunflower
dumpworld_SOURCES= dumpworld.cpp Interlock.cpp
dumpworld_LDADD=   ../common/libcommon.a -lsunflower

../common/common/libcommon.a:
	$(MAKE) -C ../common
