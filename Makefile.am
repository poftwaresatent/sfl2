CPPFLAGS= @NPM_CPPFLAGS@
CFLAGS=   @NPM_CFLAGS@
CXXFLAGS= @NPM_CXXFLAGS@
LDFLAGS=  @NPM_LDFLAGS@

if NPM_HAVE_ESTAR
  ESTAR_DIR=    estar
  ESTAR_LIBADD= estar/libestar.la
endif 

if NPM_HAVE_ASL
  ASL_DIR=    asl
  ASL_LIBADD= asl/libasl.la
endif 

SUBDIRS= common robox visitor \
         $(ESTAR_DIR) $(ASL_DIR) \
         . simul

lib_LTLIBRARIES=       libnepumuk.la
libnepumuk_la_SOURCES= 
libnepumuk_la_LIBADD=  common/libcommon.la \
                       robox/librobox.la \
                       visitor/libvisitor.la \
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
