CPPFLAGS= @NPM_CPPFLAGS@
CFLAGS=   @NPM_CFLAGS@
CXXFLAGS= @NPM_CXXFLAGS@
LDFLAGS=  @NPM_LDFLAGS@

if NPM_HAVE_XCF
  BIRON_DIR=    biron
  BIRON_LIBADD= biron/libbiron.la
endif

if NPM_HAVE_ROS
  ROS_DIR=    ros
  ROS_LIBADD= ros/libnpmros.la
endif

if NPM_HAVE_ESTAR
  ESTAR_DIR=    estar
  ESTAR_LIBADD= estar/libestar.la
endif 

if NPM_HAVE_ASL
  ASL_DIR=    asl
  ASL_LIBADD= asl/libasl.la
endif 

if NPM_HAVE_GENOM
  LAAS_DIR=    laas
  LAAS_LIBADD= laas/liblaas.la
endif

if NPM_ENABLE_QT
  QT_DIR= qt
else
  QT_DIR=
endif

SUBDIRS= common robox visitor \
         $(ROS_DIR) $(LAAS_DIR) $(BIRON_DIR) $(ESTAR_DIR) $(ASL_DIR) \
         . simul \
         $(QT_DIR)

lib_LTLIBRARIES=       libnepumuk.la
libnepumuk_la_SOURCES= 
libnepumuk_la_LIBADD=  common/libcommon.la \
                       robox/librobox.la \
                       visitor/libvisitor.la \
                       $(ROS_LIBADD) \
                       $(LAAS_LIBADD) \
                       $(BIRON_LIBADD) \
                       $(ESTAR_LIBADD) \
                       $(ASL_LIBADD)

SRCDIR=                @abs_top_srcdir@
BUILDDIR=              @abs_top_builddir@

BUILT_SOURCES=         incsym

EXTRA_DIST=            bootstrap-buildsystem.sh \
                       build-stage.sh

incsym:
	test -d $(BUILDDIR)/include || mkdir -p $(BUILDDIR)/include
	test -L $(BUILDDIR)/include/npm || ln -s $(SRCDIR) $(BUILDDIR)/include/npm
