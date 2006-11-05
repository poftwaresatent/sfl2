SUBDIRS=                 api bband contrib cwrap dwa expo gplan util

lib_LTLIBRARIES=         libsunflower.la
libsunflower_la_SOURCES= 
libsunflower_la_LDFLAGS= -version-info 3:0:0
libsunflower_la_LIBADD=  api/libapi.la \
                         bband/libbband.la \
                         contrib/libcontrib.la \
                         cwrap/libcwrap.la \
                         dwa/libdwa.la \
                         expo/libexpo.la \
                         gplan/libgplan.la \
                         util/libutil.la

pkgconfigdir=            $(libdir)/pkgconfig
pkgconfig_DATA=          sunflower.pc

BUILT_SOURCES=           sfl

sfl:
	test -L sfl || ln -s $(top_srcdir) sfl
