if BUILD_BIRON
  BIRON_DIR=biron
  BIRON_LIBADD= biron/libbiron.la
endif

SUBDIRS=       common robox visitor laas $(BIRON_DIR) . simul

noinst_LTLIBRARIES=    libnepumuk.la
libnepumuk_la_SOURCES= 
libnepumuk_la_LIBADD=  common/libcommon.la \
                       robox/librobox.la \
                       visitor/libvisitor.la \
                       laas/liblaas.la \
                       $(BIRON_LIBADD)

BUILT_SOURCES= npm

npm:
	test -L npm || ln -s $(top_srcdir) npm
