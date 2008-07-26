CPPFLAGS= @SFL_CPPFLAGS@
CFLAGS=   @SFL_CFLAGS@
CXXFLAGS= @SFL_CXXFLAGS@
LDFLAGS=  @SFL_LDFLAGS@

noinst_LTLIBRARIES= libutil.la
libutil_la_SOURCES= Point.cpp Ray.cpp Frame.cpp Line.cpp Pthread.cpp \
                    Hull.cpp numeric.cpp Polygon.cpp GoalManager.cpp \
                    OptionDictionary.cpp strutil.cpp Random.cpp
include_HEADERS=    Point.hpp array2d.hpp functors.hpp Frame.hpp \
                    vec2d.hpp Pthread.hpp Ray.hpp Line.hpp pdebug.hpp \
                    Polygon.hpp Hull.hpp numeric.hpp GoalManager.hpp \
                    ringbuf.hpp flexgrid.hpp OptionDictionary.hpp strutil.hpp \
                    Random.hpp
includedir=         @includedir@/sfl/util

check_PROGRAMS=       test-goalmgr test-ringbuf test-pthread
LDADD=                ../libsunflower.la
test_goalmgr_SOURCES= test-goalmgr.cpp
test_ringbuf_SOURCES= test-ringbuf.cpp
test_pthread_SOURCES= test-pthread.cpp

../libsunflower.la:
	$(MAKE) -C ..
