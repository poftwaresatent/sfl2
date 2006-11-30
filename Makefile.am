if HAVE_XCF
  BIRON_DIR=    biron
  BIRON_LIBADD= biron/libbiron.la
endif

if HAVE_ESTAR
  ESTAR_DIR=    estar
  ESTAR_LIBADD= estar/libestar.la
endif 

SUBDIRS= common robox smart visitor laas $(BIRON_DIR) $(ESTAR_DIR) . simul

noinst_LTLIBRARIES=    libnepumuk.la
libnepumuk_la_SOURCES= 
libnepumuk_la_LIBADD=  common/libcommon.la \
                       robox/librobox.la \
                       visitor/libvisitor.la \
                       laas/liblaas.la \
					   smart/libsmart.la \
                       $(BIRON_LIBADD) \
                       $(ESTAR_LIBADD)

BUILT_SOURCES= npm

npm:
	test -L npm || ln -s $(top_srcdir) npm
