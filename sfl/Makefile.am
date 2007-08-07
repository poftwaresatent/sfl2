CPPFLAGS= @SFL_CPPFLAGS@
CFLAGS=   @SFL_CFLAGS@
CXXFLAGS= @SFL_CXXFLAGS@
LDFLAGS=  @SFL_LDFLAGS@

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

EXTRA_DIST=          LICENSE.BSD \
                     bootstrap-buildsystem.sh \
                     build-stage.sh \
                     doxylogo.png \
                     footer.html \
                     mainpage.dox

SRCDIR=                  @abs_top_srcdir@
BUILDDIR=                @abs_top_builddir@

BUILT_SOURCES=           incsym

incsym:
	test -d $(BUILDDIR)/include || mkdir -p $(BUILDDIR)/include
	test -L $(BUILDDIR)/include/sfl || ln -s $(SRCDIR) $(BUILDDIR)/include/sfl

ChangeLog:
	- echo 'see http://libsunflower.svn.sourceforge.net/viewvc/libsunflower/trunk/sunflower/' > ChangeLog
