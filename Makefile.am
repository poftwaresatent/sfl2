if HAVE_XCF
  BIRON_DIR=    biron
  BIRON_LIBADD= biron/libbiron.la
endif

if HAVE_ESTAR
  ESTAR_DIR=    estar smart
  ESTAR_LIBADD= estar/libestar.la smart/libsmart.la
endif 

if HAVE_GENOM
  LAAS_DIR=    laas
  LAAS_LIBADD= laas/liblaas.la
endif

if ENABLE_QT
  QT_DIR= qt
else
  QT_DIR= qt
endif

SUBDIRS= common robox visitor \
         $(LAAS_DIR) $(BIRON_DIR) $(ESTAR_DIR) \
         . simul \
         $(QT_DIR)

noinst_LTLIBRARIES=    libnepumuk.la
libnepumuk_la_SOURCES= 
libnepumuk_la_LIBADD=  common/libcommon.la \
                       robox/librobox.la \
                       visitor/libvisitor.la \
                       $(LAAS_LIBADD) \
                       $(BIRON_LIBADD) \
                       $(ESTAR_LIBADD)

SRCDIR=                @abs_top_srcdir@
BUILDDIR=              @abs_top_builddir@

BUILT_SOURCES=         incsym

incsym:
	test -d $(BUILDDIR)/include || mkdir -p $(BUILDDIR)/include
	test -L $(BUILDDIR)/include/npm || ln -s $(SRCDIR) $(BUILDDIR)/include/npm
