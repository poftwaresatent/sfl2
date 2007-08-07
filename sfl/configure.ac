AC_REVISION($Id$)
AC_INIT([libsunflower],
        [1.0pre],
        [libsunflower-devel AT lists DOT sourceforge DOT net])
AC_PREREQ(2.57)
AM_INIT_AUTOMAKE([-Wall -Werror foreign])

AC_LANG(C++)

AC_PROG_CC
if test "x${GCC}" != "xyes"; then
   AC_MSG_WARN([Looks like a non-GNU C compiler, build might break.])
   AC_MSG_WARN([Update configure.ac with compiler-vendor detection.])
fi

AC_PROG_CXX
if test "x${GXX}" != "xyes"; then
   AC_MSG_WARN([Looks like a non-GNU C++ compiler, build might break.])
   AC_MSG_WARN([Update configure.ac with compiler-vendor detection.])
fi

AC_CANONICAL_HOST
case $host_os in
  linux*)  AC_MSG_RESULT([detected Linux])
	   SFL_LDFLAGS="-lpthread $SFL_LDFLAGS"
           AC_DISABLE_SHARED;;
  darwin*) AC_MSG_RESULT([detected Darwin (OS X)])
           SFL_CPPFLAGS="$SFL_CPPFLAGS -Wno-long-double"
           AC_DISABLE_STATIC;;
  *)       AC_MSG_WARN([Unknown host_os $host_os, build will probably break.])
           AC_MSG_WARN([Please update configure.ac.]);;
esac
AC_PROG_LIBTOOL
AC_PROG_INSTALL

SFL_CFLAGS="-pipe"

AC_ARG_ENABLE(debug,
  AS_HELP_STRING([--enable-debug], [GCC options -g -O0 (else -O3)]),
  [ SFL_CPPFLAGS="-DSFL_DEBUG"
    SFL_CFLAGS="$SFL_CFLAGS -g -O0" ],
  [ SFL_CPPFLAGS=""
    SFL_CFLAGS="$SFL_CFLAGS -O3" ])

AC_ARG_ENABLE(pedantic,
  AS_HELP_STRING([--enable-pedantic], [GCC options -pedantic (else -Wall)]),
  [ SFL_CFLAGS="$SFL_CFLAGS -pedantic" ],
  [ SFL_CFLAGS="$SFL_CFLAGS -Wall" ])

AC_ARG_WITH(boost,
  AS_HELP_STRING([--with-boost=PATH], [specify boost install dir]),
  [ if test "x$withval" != "x" ; then
      SFL_CPPFLAGS="$SFL_CPPFLAGS -I${withval}/include -I${withval}"
      SFL_LDFLAGS="-L${withval}/lib -I${withval} $SFL_LDFLAGS"
    fi ], [])
oldCPPFLAGS="$CPPFLAGS"
test "$prefix" != "NONE" && CPPFLAGS="$SFL_CPPFLAGS -I$prefix/include"
AC_CHECK_HEADER([boost/shared_ptr.hpp], [], [
  AC_MSG_ERROR([cannot compile boost/shared_ptr.hpp (CPPFLAGS=$CPPFLAGS)])])
CPPFLAGS="$oldCPPFLAGS"

AC_ARG_WITH(robox,
  AC_HELP_STRING([--with-robox=PATH], [support Robox (installed in PATH)]),
  [ AM_CONDITIONAL([SFL_HAVE_ROBOX], [true])
    if test "x$withval" != "x" ; then
      SFL_CPPFLAGS="$SFL_CPPFLAGS -I${withval}/include -I${withval}"
    fi
    oldCPPFLAGS="$CPPFLAGS"
    test "$prefix" != "NONE" && CPPFLAGS="$SFL_CPPFLAGS -I$prefix/include"
    AC_CHECK_HEADER([hal.h], [], [
      AC_MSG_ERROR([cannot find/compile hal.h (adjust --with-robox)])])
    CPPFLAGS="$oldCPPFLAGS" ],
  [ AM_CONDITIONAL([SFL_HAVE_ROBOX], [false]) ])

dnl IDEA: use two separate options for libtool/non-LT install...
AC_ARG_WITH(estar,
  AC_HELP_STRING([--with-estar=PATH], [enable E-Star (installed in PATH)]),
  [ if test "x$withval" = "x" ; then
      SFL_CPPFLAGS="$SFL_CPPFLAGS -DHAVE_ESTAR"
      SFL_LDFLAGS="-lestar $SFL_LDFLAGS"
    else
      SFL_CPPFLAGS="$SFL_CPPFLAGS -DHAVE_ESTAR -I${withval}/include -I${withval}"
      SFL_LDFLAGS="-lestar -L${withval}/lib -L${withval} $SFL_LDFLAGS"
    fi
    AM_CONDITIONAL([SFL_HAVE_ESTAR], [true]) ],
  [ AM_CONDITIONAL([SFL_HAVE_ESTAR], [false]) ])

ABSTOPBUILDDIR=`pwd`
if test -z "$ABSTOPBUILDDIR" ; then
  AC_MSG_ERROR([could not determine current working directory])
fi
if test "$prefix" = "NONE" ; then
  SFL_CPPFLAGS="-I${ABSTOPBUILDDIR}/include $SFL_CPPFLAGS"
else
  SFL_CPPFLAGS="-I${ABSTOPBUILDDIR}/include $SFL_CPPFLAGS -I${prefix}/include"
  SFL_LDFLAGS="$SFL_LDFLAGS -L$prefix/lib"
fi
SFL_CXXFLAGS="$SFL_CFLAGS"

AC_SUBST(PACKAGE_VERSION)
AC_SUBST(SFL_CPPFLAGS)
AC_SUBST(SFL_CFLAGS)
AC_SUBST(SFL_CXXFLAGS)
AC_SUBST(SFL_LDFLAGS)
AC_CONFIG_HEADERS(config.h)
AC_CONFIG_FILES(Makefile sunflower.pc Doxyfile
                contrib/Makefile gplan/Makefile expo/Makefile
                cwrap/Makefile util/Makefile bband/Makefile api/Makefile
                dwa/Makefile)
AC_OUTPUT
