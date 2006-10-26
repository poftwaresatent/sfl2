AC_REVISION($Id$)
AC_INIT(libsunflower,1.0pre)
AC_PREREQ(2.57)
AM_INIT_AUTOMAKE([-Wall -Werror foreign])

AC_CANONICAL_HOST
case $host_os in
  linux*)  AC_MSG_RESULT([detected Linux])
	   LDFLAGS="$LDFLAGS -lpthread";;
  darwin*) AC_MSG_RESULT([detected Darwin (OS X)])
           CPPFLAGS="$CPPFLAGS -Wno-long-double";;
  *)       AC_MSG_WARN([unknown host_os $host_os, please update configure.ac]);;
esac

AC_PROG_CC
test "x${GCC}" = "xyes" && CFLAGS="-pipe"
AC_PROG_CXX
test "x${GXX}" = "xyes" && CXXFLAGS="-pipe"
AC_LANG(C++)

AC_ARG_ENABLE(debug,
  AC_HELP_STRING([--enable-debug], [GCC options -g -O0 (else -O2)]),
  [ test "x${GCC}" = "xyes" && CFLAGS="$CFLAGS -DDEBUG -g -O0"
    test "x${GXX}" = "xyes" && CXXFLAGS="$CXXFLAGS -DDEBUG -g -O0" ],
  [ test "x${GCC}" = "xyes" && CFLAGS="$CFLAGS -O2"
    test "x${GXX}" = "xyes" && CXXFLAGS="$CXXFLAGS -O2" ])

AC_ARG_ENABLE(pedantic,
  AC_HELP_STRING([--enable-pedantic], [GCC option -pedantic (else -Wall)]),
  [ test "x${GCC}" = "xyes" && CFLAGS="$CFLAGS -pedantic"
    test "x${GXX}" = "xyes" && CXXFLAGS="$CXXFLAGS -pedantic" ],
  [ test "x${GCC}" = "xyes" && CFLAGS="$CFLAGS -Wall"
    test "x${GXX}" = "xyes" && CXXFLAGS="$CXXFLAGS -Wall" ])

AC_ARG_WITH(boost,
  AC_HELP_STRING([--with-boost=PATH], [specify boost install dir]),
  [ if test "x$withval" != "x" ; then
      CPPFLAGS="$CPPFLAGS -I${withval}/include"
      LDFLAGS="$LDFLAGS -L${withval}/lib"
    fi ], [])
oldCPPFLAGS="$CPPFLAGS"
CPPFLAGS="$CPPFLAGS -I${prefix}/include"
AC_CHECK_HEADER([boost/shared_ptr.hpp], [], [ AC_MSG_ERROR(
  [cannot find or compile boost/shared_ptr.hpp (use/adjust --with-boost)])] )
CPPFLAGS="$oldCPPFLAGS"

AC_ARG_WITH(robox,
  AC_HELP_STRING([--with-robox=PATH], [support Robox (installed in PATH)]),
  [ AM_CONDITIONAL([ROBOX], [true])
    if test "x$withval" != "x" ; then
      CPPFLAGS="$CPPFLAGS -I${withval}/include"
dnl any libs??? LDFLAGS="$LDFLAGS -L${withval}/lib -lWHATEVER"
    fi
    oldCPPFLAGS="$CPPFLAGS"
    CPPFLAGS="$CPPFLAGS -I${prefix}/include"
    AC_CHECK_HEADER([hal.h], [], [
      AC_MSG_ERROR([cannot find/compile hal.h (adjust --with-robox)])])
    CPPFLAGS="$oldCPPFLAGS" ],
  [ AM_CONDITIONAL([ROBOX], [false]) ])

AC_DISABLE_STATIC
AC_PROG_LIBTOOL
AC_PROG_INSTALL

CPPFLAGS="$CPPFLAGS -I${prefix}/include"
includedir="${includedir}/sfl"

AC_SUBST(PACKAGE_VERSION)
AC_CONFIG_HEADERS(config.h)
AC_CONFIG_FILES(Makefile sunflower.pc Doxyfile
                contrib/Makefile gplan/Makefile expo/Makefile
                cwrap/Makefile util/Makefile bband/Makefile api/Makefile
                dwa/Makefile)
AC_OUTPUT
