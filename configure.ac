AC_REVISION($Id$)
AC_INIT([sunflower2],
        [2.0pre],
        [libsunflower-devel AT lists DOT sourceforge DOT net])
AC_CONFIG_AUX_DIR([.])
AC_CONFIG_MACRO_DIR([m4])
AC_PREREQ(2.57)
AM_INIT_AUTOMAKE([-Wall -Werror foreign])
AM_PROG_AR

AC_LANG(C++)

AC_PROG_CXX
if test "x${GXX}" != "xyes"; then
   AC_MSG_WARN([Looks like a non-GNU C++ compiler, build might break.])
   AC_MSG_WARN([Update configure.ac with compiler-vendor detection.])
fi

SFL2_CPPFLAGS=""
SFL2_CFLAGS="-pipe"
SFL2_LDFLAGS=""

AC_CANONICAL_HOST
case $host_os in
  linux*)  AC_MSG_NOTICE([detected Linux])
           SFL2_CPPFLAGS="$SFL2_CPPFLAGS -DLINUX"
           SFL2_GLUTLIBS="-lglut -lGLU -lGL"
	   oldCPPFLAGS="$CPPFLAGS"
	   if test "$prefix" = "NONE"; then
	     CPPFLAGS="$SFL2_CPPFLAGS"
	   else
	     CPPFLAGS="$SFL2_CPPFLAGS -I$prefix/include"
	   fi
	   AC_CHECK_HEADER([GL/glut.h], [],
	     [ AC_MSG_ERROR([cannot find GL/glut.h]) ])
           CPPFLAGS="$oldCPPFLAGS"
           AM_DISABLE_STATIC;;
  *openbsd*) AC_MSG_NOTICE([detected OpenBSD])
             SFL2_CPPFLAGS="$SFL2_CPPFLAGS -DOPENBSD -I/usr/local/include -I/usr/X11R6/include"
	     SFL2_LDFLAGS="$SFL2_LDFLAGS -L/usr/local/lib -L/usr/X11R6/lib"

             SFL2_GLUTLIBS="-lX11 -lXi -lXmu -lglut -lGLU -lGL"
             oldCPPFLAGS="$CPPFLAGS"
             if test "$prefix" = "NONE"; then
               CPPFLAGS="$SFL2_CPPFLAGS"
             else
               CPPFLAGS="$SFL2_CPPFLAGS -I$prefix/include"
             fi
             AC_CHECK_HEADER([GL/glut.h], [],
               [ AC_MSG_ERROR([Nepumuk requires GLUT development packages (which in turn requires GL and GLU)])])
             CPPFLAGS="$oldCPPFLAGS"
             AM_DISABLE_STATIC;;
  darwin*)  AC_MSG_NOTICE([detected Darwin])
           SFL2_CPPFLAGS="$SFL2_CPPFLAGS -DOSX -I/usr/X11/include"
           SFL2_GLUTLIBS="-framework OpenGL -framework GLUT -lobjc"
	   oldCPPFLAGS="$CPPFLAGS"
	   if test "$prefix" = "NONE"; then
	     CPPFLAGS="$SFL2_CPPFLAGS"
	   else
	     CPPFLAGS="$SFL2_CPPFLAGS -I$prefix/include"
	   fi
	   AC_CHECK_HEADER([GL/glut.h], [],
	     [ AC_MSG_ERROR([cannot find GL/glut.h]) ])
           AM_DISABLE_STATIC;;
  *)       AC_MSG_WARN([Unknown host_os $host_os, build will probably break.])
           AC_MSG_WARN([Please update configure.ac.]);;
esac
AC_PROG_LIBTOOL
AC_PROG_INSTALL
AC_PROG_EGREP

AC_ARG_ENABLE(debug,
  AS_HELP_STRING([--enable-debug], [GCC options -g -O0 (else -O3)]),
  [ SFL2_CPPFLAGS="$SFL2_CPPFLAGS -DSFL2_DEBUG"
    SFL2_CFLAGS="$SFL2_CFLAGS -g -O0" ],
  [ SFL2_CFLAGS="$SFL2_CFLAGS -O3" ])

AC_ARG_ENABLE(pedantic,
  AS_HELP_STRING([--enable-pedantic], [GCC options -pedantic (else -Wall)]),
  [ SFL2_CFLAGS="$SFL2_CFLAGS -pedantic" ],
  [ SFL2_CFLAGS="$SFL2_CFLAGS -Wall" ])

AC_ARG_WITH(boost,
  AS_HELP_STRING([--with-boost=PATH], [specify boost install dir]),
  [ if test "x$withval" != "x" ; then
      SFL2_CPPFLAGS="$SFL2_CPPFLAGS -I${withval}/include -I${withval}"
      SFL2_LDFLAGS="-L${withval}/lib -I${withval} $SFL2_LDFLAGS"
    fi ], [])
oldCPPFLAGS="$CPPFLAGS"
if test "$prefix" = "NONE"; then
   CPPFLAGS="$SFL2_CPPFLAGS"
else
   CPPFLAGS="$SFL2_CPPFLAGS -I$prefix/include"
fi
AC_CHECK_HEADER([boost/shared_ptr.hpp], [], [
  AC_MSG_ERROR([cannot compile boost/shared_ptr.hpp (CPPFLAGS=$CPPFLAGS)])])
CPPFLAGS="$oldCPPFLAGS"

AC_ARG_WITH(png,
  AC_HELP_STRING([--with-png=PATH], [enable libpng (for screenshots)]),
  [ if test "x$with_png" = xno; then
      AC_MSG_NOTICE([png support explicitly disabled])
    fi ],
  [ with_png=check ])
AS_IF([test "x$with_png" != xno],
  [ if test "$prefix" = "NONE"; then
      CPPFLAGS="$SFL2_CPPFLAGS"
    else
      CPPFLAGS="$SFL2_CPPFLAGS -I$prefix/include"
    fi
    AC_CHECK_HEADERS([png.h],
      [ SFL2_CPPFLAGS="$SFL2_CPPFLAGS -DSFL2_HAVE_PNG"
        SFL2_LDFLAGS="-lpng -lz $SFL2_LDFLAGS" ],
      [ if test "x$with_png" != xcheck; then
          AC_MSG_FAILURE([--with-png was given, but cannot find png.h])
        else
          AC_MSG_NOTICE([png.h not found])
        fi ])
    CPPFLAGS="$oldCPPFLAGS" ],
  [])

SFL2_CXXFLAGS="$SFL2_CFLAGS"

AC_SUBST(PACKAGE_VERSION)
AC_SUBST(SFL2_CPPFLAGS)
AC_SUBST(SFL2_CFLAGS)
AC_SUBST(SFL2_CXXFLAGS)
AC_SUBST(SFL2_LDFLAGS)
AC_SUBST(SFL2_GLUTLIBS)
AC_CONFIG_FILES([Makefile
                 sunflower2.pc
                 doc/Doxyfile
                 3rdparty/Makefile
                 sfl/Makefile
                 sfl/gplan/Makefile
                 sfl/expo/Makefile
                 sfl/util/Makefile
                 sfl/bband/Makefile
                 sfl/api/Makefile
                 sfl/dwa/Makefile
                 npm/Makefile
                 npm/gfx/Makefile
                 npm/ext/Makefile
                 apps/Makefile])
AC_OUTPUT
