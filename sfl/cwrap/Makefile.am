noinst_LTLIBRARIES=  libcwrap.la
libcwrap_la_SOURCES= Handlemap.cpp cwrapHAL.cpp sfl.cpp expo.cpp
include_HEADERS=     expo.h Handlemap.hpp hal.h cwrapHAL.hpp sfl.h
includedir=          @includedir@/cwrap

check_PROGRAMS=      testcwrap
testcwrap_SOURCES=   testcwrap.cpp
testcwrap_LDADD=     ../libsunflower.la

../libsunflower.la:
	$(MAKE) -C ..
