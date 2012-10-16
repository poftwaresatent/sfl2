AC_REVISION($Id$)
AC_INIT([nepumuk],
        [1.0pre],
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

NPM_CPPFLAGS=""
NPM_CFLAGS="-pipe"
NPM_LDFLAGS=""

AC_CANONICAL_HOST
case $host_os in
  linux*)  AC_MSG_NOTICE([detected Linux])
           NPM_CPPFLAGS="$NPM_CPPFLAGS -DLINUX"
           NPM_GLUTLIBS="-lglut -lGLU -lGL"
	   oldCPPFLAGS="$CPPFLAGS"
	   if test "$prefix" = "NONE"; then
	     CPPFLAGS="$NPM_CPPFLAGS"
	   else
	     CPPFLAGS="$NPM_CPPFLAGS -I$prefix/include"
	   fi
	   AC_CHECK_HEADER([GL/glut.h], [],
	     [ AC_MSG_ERROR([cannot find GL/glut.h]) ])
           CPPFLAGS="$oldCPPFLAGS"
           AM_DISABLE_STATIC;;
  *openbsd*) AC_MSG_NOTICE([detected OpenBSD])
             NPM_CPPFLAGS="$NPM_CPPFLAGS -DOPENBSD -I/usr/local/include -I/usr/X11R6/include"
	     NPM_LDFLAGS="$NPM_LDFLAGS -L/usr/local/lib -L/usr/X11R6/lib"

             NPM_GLUTLIBS="-lX11 -lXi -lXmu -lglut -lGLU -lGL"
             oldCPPFLAGS="$CPPFLAGS"
             if test "$prefix" = "NONE"; then
               CPPFLAGS="$NPM_CPPFLAGS"
             else
               CPPFLAGS="$NPM_CPPFLAGS -I$prefix/include"
             fi
             AC_CHECK_HEADER([GL/glut.h], [],
               [ AC_MSG_ERROR([Nepumuk requires GLUT development packages (which in turn requires GL and GLU)])])
             CPPFLAGS="$oldCPPFLAGS"
             AM_DISABLE_STATIC;;
  darwin*)  AC_MSG_NOTICE([detected Darwin])
           NPM_CPPFLAGS="$NPM_CPPFLAGS -DOSX -I/usr/X11/include"
           NPM_GLUTLIBS="-framework OpenGL -framework GLUT -lobjc"
	   oldCPPFLAGS="$CPPFLAGS"
	   if test "$prefix" = "NONE"; then
	     CPPFLAGS="$NPM_CPPFLAGS"
	   else
	     CPPFLAGS="$NPM_CPPFLAGS -I$prefix/include"
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
  [ NPM_CPPFLAGS="$NPM_CPPFLAGS -DNPM_DEBUG"
    NPM_CFLAGS="$NPM_CFLAGS -g -O0" ],
  [ NPM_CFLAGS="$NPM_CFLAGS -O3" ])

AC_ARG_ENABLE(pedantic,
  AS_HELP_STRING([--enable-pedantic], [GCC options -pedantic (else -Wall)]),
  [ NPM_CFLAGS="$NPM_CFLAGS -pedantic" ],
  [ NPM_CFLAGS="$NPM_CFLAGS -Wall" ])

AC_ARG_WITH(boost,
  AS_HELP_STRING([--with-boost=PATH], [specify boost install dir]),
  [ if test "x$withval" != "x" ; then
      NPM_CPPFLAGS="$NPM_CPPFLAGS -I${withval}/include -I${withval}"
      NPM_LDFLAGS="-L${withval}/lib -I${withval} $NPM_LDFLAGS"
    fi ], [])
oldCPPFLAGS="$CPPFLAGS"
if test "$prefix" = "NONE"; then
   CPPFLAGS="$NPM_CPPFLAGS"
else
   CPPFLAGS="$NPM_CPPFLAGS -I$prefix/include"
fi
AC_CHECK_HEADER([boost/shared_ptr.hpp], [], [
  AC_MSG_ERROR([cannot compile boost/shared_ptr.hpp (CPPFLAGS=$CPPFLAGS)])])
CPPFLAGS="$oldCPPFLAGS"

AC_ARG_WITH(sfl,
  AS_HELP_STRING([--with-sfl=PATH], [specify libsunflower install dir]),
  [ if test "x$withval" != "x" ; then
      NPM_CPPFLAGS="$NPM_CPPFLAGS -I${withval}/include -I${withval}"
      NPM_LDFLAGS="-L${withval}/lib -I${withval} $NPM_LDFLAGS"
    fi ], [])
oldCPPFLAGS="$CPPFLAGS"
if test "$prefix" = "NONE"; then
   CPPFLAGS="$NPM_CPPFLAGS"
else
   CPPFLAGS="$NPM_CPPFLAGS -I$prefix/include"
fi
AC_CHECK_HEADER([sfl/util/numeric.hpp], [], [
  AC_MSG_ERROR([cannot compile sfl/util/numeric.hpp (CPPFLAGS=$CPPFLAGS)])])
CPPFLAGS="$oldCPPFLAGS"

AC_ARG_WITH(png,
  AC_HELP_STRING([--with-png=PATH], [specify libpng install dir]),
  [ if test "x$withval" != "x" ; then
      NPM_CPPFLAGS="$NPM_CPPFLAGS -I${withval}/include -I${withval}"
      NPM_LDFLAGS="-L${withval}/lib -L${withval} $NPM_LDFLAGS"
    fi ], [])
oldCPPFLAGS="$CPPFLAGS"
if test "$prefix" = "NONE"; then
  CPPFLAGS="$NPM_CPPFLAGS"
else
  CPPFLAGS="$NPM_CPPFLAGS -I$prefix/include"
fi
AC_CHECK_HEADERS([png.h],
  [ NPM_LDFLAGS="-lpng -lz $NPM_LDFLAGS" ],
  [ AC_MSG_NOTICE([PNG support disabled (CPPFLAGS=$CPPFLAGS)]) ])
CPPFLAGS="$oldCPPFLAGS"

NPM_CXXFLAGS="$NPM_CFLAGS"

AC_SUBST(PACKAGE_VERSION)
AC_SUBST(NPM_CPPFLAGS)
AC_SUBST(NPM_CFLAGS)
AC_SUBST(NPM_CXXFLAGS)
AC_SUBST(NPM_LDFLAGS)
AC_SUBST(NPM_GLUTLIBS)
AC_CONFIG_HEADERS(config.h)
AC_CONFIG_FILES([Makefile
                 npm/Makefile
                 npm/gfx/Makefile
                 npm/ext/Makefile
                 apps/Makefile])
AC_OUTPUT
