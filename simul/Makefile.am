GLUTLIBS=          @GLUTLIBS@
bin_PROGRAMS=      nepumuk dumpworld
nepumuk_SOURCES=   nepumuk.cpp RobotFactory.cpp Interlock.cpp Simulator.cpp
nepumuk_LDADD=     ../robox/librobox.a \
                   ../visitor/libvisitor.a \
                   ../laas/liblaas.a \
                   ../common/libcommon.a \
                   $(GLUTLIBS) -lsunflower
dumpworld_SOURCES= dumpworld.cpp Interlock.cpp
dumpworld_LDADD=   ../robox/librobox.a \
                   ../visitor/libvisitor.a \
                   ../laas/liblaas.a \
                   ../common/libcommon.a \
                   $(GLUTLIBS) -lsunflower

../common/libcommon.a:
	$(MAKE) -C ../common

../robox/librobox.a:
	$(MAKE) -C ../robox

../visitor/libvisitor.a:
	$(MAKE) -C ../visitor

../laas/liblaas.a:
	$(MAKE) -C ../laas
