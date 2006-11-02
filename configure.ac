AC_REVISION($Id$)
AC_INIT(nepumuk, 0.0pre, libsunflower-devel AT lists DOT sourceforge DOT net)
AC_PREREQ(2.57)
AM_INIT_AUTOMAKE([-Wall -Werror foreign])

AC_LANG(C++)
AC_PROG_CXX
test "x${GXX}" = "xyes" && CXXFLAGS="$CXXFLAGS -pipe"

AC_ARG_ENABLE(debug,
  AC_HELP_STRING([--enable-debug], [GCC options -g -O0 (else -O2)]),
  [ test "x${GXX}" = "xyes" && CXXFLAGS="$CXXFLAGS -DDEBUG -g -O0" ],
  [ test "x${GXX}" = "xyes" && CXXFLAGS="$CXXFLAGS -O2" ])

AC_ARG_ENABLE(pedantic,
  AC_HELP_STRING([--enable-pedantic], [GCC option -pedantic (else -Wall)]),
  [ test "x${GXX}" = "xyes" && CXXFLAGS="$CXXFLAGS -pedantic" ],
  [ test "x${GXX}" = "xyes" && CXXFLAGS="$CXXFLAGS -Wall" ])

LDFLAGS="$LDFLAGS -lsunflower"

AC_CANONICAL_HOST
case $host_os in
  linux*)  AC_MSG_RESULT([detected Linux])
           CPPFLAGS="$CPPFLAGS -DLINUX"
           LDFLAGS="$LDFLAGS -lpthread"
	   AC_SUBST(GLUTLIBS, "-lglut -lGLU -lGL");;
  darwin*) AC_MSG_RESULT([detected Darwin (OS X)])
           CPPFLAGS="$CPPFLAGS -DOSX"
           LDFLAGS="$LDFLAGS -lobjc"
           CXXFLAGS="$CXXFLAGS -Wno-long-double"
	   AC_SUBST(GLUTLIBS, "-framework OpenGL -framework GLUT -lobjc");;
  *)       AC_MSG_ERROR([unknown host_os $host_os, update configure.ac]);;
esac

AC_ARG_WITH(boost,
  AC_HELP_STRING([--with-boost=PATH], [specify boost install dir]),
  [ if test "x$withval" != "x" ; then
      CPPFLAGS="$CPPFLAGS -I${withval}/include"
      LDFLAGS="$LDFLAGS -L${withval}/lib"
    fi ], [])
oldCPPFLAGS="$CPPFLAGS"
test "$prefix" != "NONE" && CPPFLAGS="$CPPFLAGS -I$prefix/include"
AC_CHECK_HEADER([boost/shared_ptr.hpp], [], [
  AC_MSG_ERROR([cannot compile boost/shared_ptr.hpp (CPPFLAGS=$CPPFLAGS)])], [])
CPPFLAGS="$oldCPPFLAGS"

AC_ARG_WITH(sfl,
  AC_HELP_STRING([--with-sfl=PATH], [specify libsunflower install dir]),
  [ if test "x$withval" != "x" ; then
      CPPFLAGS="$CPPFLAGS -I${withval}/include"
      LDFLAGS="$LDFLAGS -L${withval}/lib"
    fi ], [])
oldCPPFLAGS="$CPPFLAGS"
test "$prefix" != "NONE" && CPPFLAGS="$CPPFLAGS -I$prefix/include"
AC_CHECK_HEADER([sfl/util/numeric.hpp], [], [
  AC_MSG_ERROR([cannot compile sfl/util/numeric.hpp (CPPFLAGS=$CPPFLAGS)])], [])
CPPFLAGS="$oldCPPFLAGS"

AC_ARG_WITH(xcf,
  AC_HELP_STRING([--with-xcf=PATH], [enable XCF (installed in PATH)]),
  [ if test "x$withval" = "x" ; then
      if test "$prefix" != "NONE"; then
        AC_SUBST(XCF_CPPFLAGS, "-I$prefix/include/sigc++-1.2 -I$prefix/lib/sigc++-1.2/include")
      else
        AC_SUBST(XCF_CPPFLAGS, "")
      fi
      AC_SUBST(XCF_LDFLAGS, "-lsigc-1.2 -lxcf -lxmltio")
    else
      AC_SUBST(XCF_CPPFLAGS, "-I${withval}/include/sigc++-1.2 -I${withval}/lib/sigc++-1.2/include -I${withval}/include")
      AC_SUBST(XCF_LDFLAGS, "-lsigc-1.2 -lxcf -lxmltio -L${withval}/lib")
    fi
    CPPFLAGS="$CPPFLAGS -DBUILD_BIRON"
    oldCPPFLAGS="$CPPFLAGS"
    if test "$prefix" != "NONE"; then
      CPPFLAGS="$CPPFLAGS $XCF_CPPFLAGS -I$prefix/include"
    else
      CPPFLAGS="$CPPFLAGS $XCF_CPPFLAGS"
    fi
    AC_CHECK_HEADER([xcf/Subscriber.hpp], [], [
      AC_MSG_ERROR([cannot compile xcf/Subscriber.hpp (CPPFLAGS=$CPPFLAGS)])], [])
    CPPFLAGS="$oldCPPFLAGS"
    AM_CONDITIONAL([BUILD_BIRON], [true]) ],
  [ AC_SUBST(XCF_CPPFLAGS, "")
    AC_SUBST(XCF_LDFLAGS, "")
    AM_CONDITIONAL([BUILD_BIRON], [false]) ])

AC_ARG_WITH(xcfglue,
  AC_HELP_STRING([--with-xcfglue=PATH], [special XCFGLUE install PATH]),
  [ if test ! -z "$BUILD_BIRON_TRUE"; then
      AC_MSG_ERROR([--with-xcfglue requires --with-xcf])
    fi
    if test "x$withval" != "x" ; then
      XCF_CPPFLAGS="$XCF_CPPFLAGS -I${withval}/include"
      XCF_LDFLAGS="-lxcfglue $XCF_LDFLAGS -L${withval}/lib"
    fi ], [])

if test -z "$BUILD_BIRON_TRUE"; then
  oldCPPFLAGS="$CPPFLAGS"
  if test "$prefix" != "NONE"; then
    CPPFLAGS="$CPPFLAGS $XCF_CPPFLAGS -I$prefix/include"
  else
    CPPFLAGS="$CPPFLAGS $XCF_CPPFLAGS"
  fi
  AC_CHECK_HEADER([xcfwrap/xcfglue.h], [], [
  AC_MSG_ERROR([cannot compile xcfwrap/xcfglue.h (CPPFLAGS=$CPPFLAGS)])], [])
  CPPFLAGS="$oldCPPFLAGS"
fi

case $host_os in
  darwin*) AC_DISABLE_SHARED;;
  *)       AC_DISABLE_STATIC;;
esac
AC_PROG_LIBTOOL
AC_PROG_INSTALL
AC_CHECK_HEADERS([png.h], [LDFLAGS="$LDFLAGS -lpng -lz"])

test "$prefix" != "NONE" && CPPFLAGS="$CPPFLAGS -I$prefix/include"
test "$prefix" != "NONE" && LDFLAGS="$LDFLAGS -L$prefix/lib"

AC_SUBST(PACKAGE_VERSION)
AC_CONFIG_HEADERS(config.h)
AC_CONFIG_FILES([Makefile common/Makefile robox/Makefile visitor/Makefile
                laas/Makefile biron/Makefile simul/Makefile])
AC_OUTPUT
