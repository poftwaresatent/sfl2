AM_CPPFLAGS= -I@abs_top_srcdir@ @SFL2_CPPFLAGS@
AM_CFLAGS=   @SFL2_CFLAGS@
AM_CXXFLAGS= @SFL2_CXXFLAGS@
AM_LDFLAGS=  @SFL2_LDFLAGS@

include_HEADERS=    Point.hpp array2d.hpp functors.hpp Frame.hpp \
                    vec2d.hpp Pthread.hpp Ray.hpp Line.hpp pdebug.hpp \
                    Polygon.hpp Hull.hpp numeric.hpp GoalManager.hpp \
                    ringbuf.hpp flexgrid.hpp OptionDictionary.hpp strutil.hpp \
                    Random.hpp
includedir=         @includedir@/sfl/util

check_PROGRAMS=       test-goalmgr test-ringbuf
LDADD=                ../libsfl2.la
test_goalmgr_SOURCES= test-goalmgr.cpp
test_ringbuf_SOURCES= test-ringbuf.cpp

../libsfl2.la:
	$(MAKE) -C ..
