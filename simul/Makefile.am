GLUTLIBS=          @GLUTLIBS@
bin_PROGRAMS=      nepumuk dumpworld
nepumuk_SOURCES=   nepumuk.cpp RobotFactory.cpp Interlock.cpp Simulator.cpp
nepumuk_LDADD=     ../common/libcommon.a \
                   ../robox/librobox.a \
                   ../visitor/libvisitor.a \
                   ../laas/liblaas.a \
                   $(GLUTLIBS) -lsunflower
dumpworld_SOURCES= dumpworld.cpp Interlock.cpp
dumpworld_LDADD=   ../common/libcommon.a \
                   ../robox/librobox.a \
                   ../visitor/libvisitor.a \
                   ../laas/liblaas.a \
                   -lsunflower

../common/libcommon.a:
	$(MAKE) -C ../common

../robox/librobox.a:
	$(MAKE) -C ../robox

../visitor/libvisitor.a:
	$(MAKE) -C ../visitor

../laas/liblaas.a:
	$(MAKE) -C ../laas
