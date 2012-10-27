AM_CPPFLAGS= -I@abs_top_srcdir@ @SFL2_CPPFLAGS@
AM_CFLAGS=   @SFL2_CFLAGS@
AM_CXXFLAGS= @SFL2_CXXFLAGS@
AM_LDFLAGS=  @SFL2_LDFLAGS@

ACLOCAL_AMFLAGS=       -I m4

SUBDIRS=               3rdparty \
                       sfl \
                       npm \
                       apps

SRCDIR=                @abs_top_srcdir@
BUILDDIR=              @abs_top_builddir@

EXTRA_DIST=            am-regen.sh \
                       build-stage.sh \
                       sfl/doxylogo.png \
                       sfl/footer.html \
                       sfl/mainpage.dox \
                       npm/mainpage.dox

pkgconfigdir=          $(libdir)/pkgconfig
pkgconfig_DATA=        sunflower2.pc

.PHONY: doc
doc:
	cd doc && doxygen Doxyfile
