AC_REVISION($Id$)
AC_INIT(nepumuk, 0.0pre, libsunflower-devel AT lists DOT sourceforge DOT net)
AC_PREREQ(2.57)
AM_INIT_AUTOMAKE([-Wall -Werror foreign])

AC_LANG(C++)
AC_PROG_CXX
test "x${GXX}" = "xyes" && CXXFLAGS="$CXXFLAGS -pipe"

AC_ARG_ENABLE(debug,
  AC_HELP_STRING([--enable-debug], [GCC options -g -O0 (else -O2)]),
  [ test "x${GXX}" = "xyes" && CXXFLAGS="$CXXFLAGS -DNPM_DEBUG -g -O0" ],
  [ test "x${GXX}" = "xyes" && CXXFLAGS="$CXXFLAGS -O2" ])

AC_ARG_ENABLE(pedantic,
  AC_HELP_STRING([--enable-pedantic], [GCC option -pedantic (else -Wall)]),
  [ test "x${GXX}" = "xyes" && CXXFLAGS="$CXXFLAGS -pedantic" ],
  [ test "x${GXX}" = "xyes" && CXXFLAGS="$CXXFLAGS -Wall" ])

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
      CPPFLAGS="$CPPFLAGS -I${withval}/include -I${withval}"
      LDFLAGS="$LDFLAGS -L${withval}/lib -I${withval}"
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
    CPPFLAGS="$CPPFLAGS -DHAVE_XCF"
    oldCPPFLAGS="$CPPFLAGS"
    if test "$prefix" != "NONE"; then
      CPPFLAGS="$CPPFLAGS $XCF_CPPFLAGS -I$prefix/include"
    else
      CPPFLAGS="$CPPFLAGS $XCF_CPPFLAGS"
    fi
    AC_CHECK_HEADER([xcf/Subscriber.hpp], [], [
      AC_MSG_ERROR([cannot compile xcf/Subscriber.hpp (CPPFLAGS=$CPPFLAGS)])], [])
    CPPFLAGS="$oldCPPFLAGS"
    AM_CONDITIONAL([HAVE_XCF], [true]) ],
  [ AC_SUBST(XCF_CPPFLAGS, "")
    AC_SUBST(XCF_LDFLAGS, "")
    AM_CONDITIONAL([HAVE_XCF], [false]) ])

AC_ARG_WITH(xcfglue,
  AC_HELP_STRING([--with-xcfglue=PATH], [special XCFGLUE install PATH]),
  [ if test ! -z "$HAVE_XCF_TRUE"; then
      AC_MSG_ERROR([--with-xcfglue requires --with-xcf])
    fi
    if test "x$withval" != "x" ; then
      XCF_CPPFLAGS="$XCF_CPPFLAGS -I${withval}/include"
      XCF_LDFLAGS="-lxcfglue $XCF_LDFLAGS -L${withval}/lib"
    fi ], [])

if test -z "$HAVE_XCF_TRUE"; then
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

AC_ARG_WITH(estar,
  AC_HELP_STRING([--with-estar=PATH], [enable E-Star (installed in PATH)]),
  [ if test "x$withval" = "x" ; then
      AC_SUBST(ESTAR_CPPFLAGS, "-DHAVE_ESTAR")
      AC_SUBST(ESTAR_LDFLAGS, "-lestar")
    else
      AC_SUBST(ESTAR_CPPFLAGS, "-DHAVE_ESTAR -I${withval}/include")
      AC_SUBST(ESTAR_LDFLAGS, "-lasl-mcontrol -lestar -L${withval}/lib")
    fi
    AM_CONDITIONAL([HAVE_ESTAR], [true]) ],
  [ AC_SUBST(ESTAR_CPPFLAGS, "")
    AC_SUBST(ESTAR_LDFLAGS, "")
    AM_CONDITIONAL([HAVE_ESTAR], [false]) ])

AC_ARG_WITH(asl,
  AC_HELP_STRING([--with-asl=PATH], [ASL 'standard software' install PATH]),
  [ if test ! -z "$HAVE_ESTAR_TRUE"; then
      AC_MSG_ERROR([--with-asl makes no sense without --with-estar])
    fi
    if test "x$withval" != "x" ; then
      ESTAR_CPPFLAGS="$ESTAR_CPPFLAGS -I${withval}/include"
      ESTAR_LDFLAGS="-L${withval}/lib $ESTAR_LDFLAGS"
    fi ], [])

if test -z "$HAVE_ESTAR_TRUE"; then
  oldCPPFLAGS="$CPPFLAGS"
  if test "$prefix" != "NONE"; then
    CPPFLAGS="$CPPFLAGS $ESTAR_CPPFLAGS -I$prefix/include"
  else
    CPPFLAGS="$CPPFLAGS $ESTAR_CPPFLAGS"
  fi
  AC_CHECK_HEADER([estar/Facade.hpp], [], [
    AC_MSG_ERROR([cannot compile estar/Facade.hpp (CPPFLAGS=$CPPFLAGS)])], [])
  AC_CHECK_HEADER([asl/path_tracking.hpp], [], [
    AC_MSG_ERROR([cannot compile asl/path_tracking.hpp (CPPFLAGS=$CPPFLAGS)])], [])
  CPPFLAGS="$oldCPPFLAGS"
fi

AC_ARG_WITH(genom,
  AC_HELP_STRING([--with-genom=PATH], [specify openrobots install dir]),
  [ if test "x$withval" != "x" ; then
      CPPFLAGS="$CPPFLAGS -DHAVE_GENOM -I${withval}/include"
      LDFLAGS="-L${withval}/lib -lposterLib $LDFLAGS"
    else
      LDFLAGS="-lposterLib $LDFLAGS"
    fi
    AM_CONDITIONAL([HAVE_GENOM], [true])
    oldCPPFLAGS="$CPPFLAGS"
    test "$prefix" != "NONE" && CPPFLAGS="$CPPFLAGS -I$prefix/include"
    AC_CHECK_HEADER([posterLib.h], [], [
      AC_MSG_ERROR([cannot compile posterLib.h (CPPFLAGS=$CPPFLAGS)])], [])
    CPPFLAGS="$oldCPPFLAGS"
  ],[
    AM_CONDITIONAL([HAVE_GENOM], [false])])

case $host_os in
  darwin*) AC_DISABLE_SHARED;;
  *)       AC_DISABLE_STATIC;;
esac
AC_PROG_LIBTOOL
AC_PROG_INSTALL
AC_CHECK_HEADERS([png.h], [LDFLAGS="$LDFLAGS -lpng -lz"])

ABSTOPBUILDDIR=`pwd`
if test -z "$ABSTOPBUILDDIR" ; then
  AC_MSG_ERROR([could not determine current working directory])
fi
if test "$prefix" = "NONE" ; then
  CPPFLAGS="-I${ABSTOPBUILDDIR}/include $CPPFLAGS"
else
  CPPFLAGS="-I${ABSTOPBUILDDIR}/include $CPPFLAGS -I${prefix}/include"
  LDFLAGS="$LDFLAGS -L$prefix/lib"
fi
CXXFLAGS="$CFLAGS"

dnl *** bit of a hack for smartflower 'meta' build
AM_CONDITIONAL([SMARTFLOWER], [false])

AC_SUBST(PACKAGE_VERSION)
AC_CONFIG_HEADERS(config.h)
AC_CONFIG_FILES([Makefile common/Makefile robox/Makefile visitor/Makefile
                laas/Makefile biron/Makefile estar/Makefile smart/Makefile
                simul/Makefile])
AC_OUTPUT
