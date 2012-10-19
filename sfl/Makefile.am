AM_CPPFLAGS= -I@abs_top_srcdir@ @SFL2_CPPFLAGS@
AM_CFLAGS=   @SFL2_CFLAGS@
AM_CXXFLAGS= @SFL2_CXXFLAGS@
AM_LDFLAGS=  @SFL2_LDFLAGS@

SUBDIRS=                 api bband dwa expo gplan util

lib_LTLIBRARIES=         libsfl2.la
libsfl2_la_SOURCES= 
libsfl2_la_LIBADD=       api/libapi.la \
                         bband/libbband.la \
                         dwa/libdwa.la \
                         expo/libexpo.la \
                         util/libutil.la \
                         gplan/libgplan.la
