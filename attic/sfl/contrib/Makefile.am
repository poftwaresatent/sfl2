AM_CPPFLAGS= @SFL_CPPFLAGS@
AM_CFLAGS=   @SFL_CFLAGS@
AM_CXXFLAGS= @SFL_CXXFLAGS@
AM_LDFLAGS=  @SFL_LDFLAGS@

noinst_LTLIBRARIES=    libcontrib.la
libcontrib_la_SOURCES= DummyHAL.cpp
include_HEADERS=       DummyHAL.hpp
includedir=            @includedir@/sfl/contrib

if SFL_HAVE_ROBOX
libcontrib_la_SOURCES+= asl_hal.cpp
include_HEADERS+=       asl_hal.hpp
endif

# can Automake recognise default C++ sources?
check_PROGRAMS=                  xcomp-test-c++-iostream tpthread \
                                 xcomp-test-c++-boost xcomp-test-c
xcomp_test_c___iostream_SOURCES= xcomp-test-c++-iostream.cpp
tpthread_SOURCES=                tpthread.cpp
xcomp_test_c___boost_SOURCES=    xcomp-test-c++-boost.cpp
LDADD=                           ../libsunflower.la
xcomp_test_c_LDADD=              -lstdc++ -lm

../libsunflower.la:
	$(MAKE) -C ..
