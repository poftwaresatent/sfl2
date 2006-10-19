CPPFLAGS=          @CPPFLAGS@ -I@prefix@/include
LDFLAGS=           @LDFLAGS@ -L@prefix@/lib
GLUTLIBS=          @GLUTLIBS@
FOOLIBS=           @FOOLIBS@
XCF_LDFLAGS=       @XCF_LDFLAGS@

if BUILD_BIRON
  BIRON_LIB=../biron/libbiron.a $(XCF_LDFLAGS)
endif

bin_PROGRAMS=      nepumuk dumpworld
nepumuk_SOURCES=   nepumuk.cpp RobotFactory.cpp Interlock.cpp Simulator.cpp
nepumuk_LDADD=     ../robox/librobox.a \
                   ../visitor/libvisitor.a \
                   ../laas/liblaas.a \
                   $(BIRON_LIB) \
                   ../common/libcommon.a \
                   $(GLUTLIBS) $(FOOLIBS)
dumpworld_SOURCES= dumpworld.cpp Interlock.cpp
dumpworld_LDADD=   ../robox/librobox.a \
                   ../visitor/libvisitor.a \
                   ../laas/liblaas.a \
                   $(BIRON_LIB) \
                   ../common/libcommon.a \
                   $(GLUTLIBS) $(FOOLIBS)

../common/libcommon.a:
	$(MAKE) -C ../common

../robox/librobox.a:
	$(MAKE) -C ../robox

../visitor/libvisitor.a:
	$(MAKE) -C ../visitor

../laas/liblaas.a:
	$(MAKE) -C ../laas

../biron/libbiron.a:
	$(MAKE) -C ../biron
