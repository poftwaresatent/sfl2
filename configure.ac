AC_REVISION($Id$)
AC_INIT(nepumuk, 0.0pre, libsunflower-devel AT lists DOT sourceforge DOT net)
AC_PREREQ(2.57)
AM_INIT_AUTOMAKE([-Wall -Werror foreign])
AC_CANONICAL_HOST

AC_LANG(C++)
AC_PROG_CXX
test "x${GXX}" = "xyes" && CXXFLAGS="-pipe"

AC_ARG_ENABLE(debug,
  AC_HELP_STRING([--enable-debug], [GCC options -g -O0 (else -O2)]),
  [ test "x${GXX}" = "xyes" && CXXFLAGS="$CXXFLAGS -DDEBUG -g -O0" ],
  [ test "x${GXX}" = "xyes" && CXXFLAGS="$CXXFLAGS -O2" ])

AC_ARG_ENABLE(pedantic,
  AC_HELP_STRING([--enable-pedantic], [GCC option -pedantic (else -Wall)]),
  [ test "x${GXX}" = "xyes" && CXXFLAGS="$CXXFLAGS -pedantic" ],
  [ test "x${GXX}" = "xyes" && CXXFLAGS="$CXXFLAGS -Wall" ])

AC_ARG_WITH(boost,
  AC_HELP_STRING([--with-boost=PATH], [specify boost install dir]),
  [ if test "x$withval" != "x" ; then
      CPPFLAGS="$CPPFLAGS -I${withval}/include"
      LDFLAGS="$LDFLAGS -L${withval}/lib"
    fi ], [])
oldCXXFLAGS="$CXXFLAGS"
CXXFLAGS="$CXXFLAGS $CPPFLAGS"
AC_CHECK_HEADER([boost/shared_ptr.hpp], [], [
  AC_MSG_ERROR([can't compile boost/shared_ptr.hpp (CXXFLAGS=$CXXFLAGS)])], [])
CXXFLAGS="$oldCXXFLAGS"

AC_ARG_WITH(sfl,
  AC_HELP_STRING([--with-sfl=PATH], [specify libsunflower install dir]),
  [ if test "x$withval" != "x" ; then
      CPPFLAGS="$CPPFLAGS -I${withval}/include"
      LDFLAGS="$LDFLAGS -L${withval}/lib"
    fi ], [])
oldCXXFLAGS="$CXXFLAGS"
CXXFLAGS="$CXXFLAGS $CPPFLAGS"
AC_CHECK_HEADER([sfl/util/numeric.hpp], [], [
  AC_MSG_ERROR([can't compile sfl/util/numeric.hpp (CXXFLAGS=$CXXFLAGS)])], [])
CXXFLAGS="$oldCXXFLAGS"

AC_ARG_WITH(xcf,
  AC_HELP_STRING([--with-xcf=PATH], [enable XCF (installed in PATH)]),
  [ AC_SUBST(WITH_XCF, "XCF_")
    if test "x$withval" = "x" ; then
      AC_SUBST(XCF_CPPFLAGS, "")
      AC_SUBST(XCF_LDFLAGS, "")
    else
      AC_SUBST(XCF_CPPFLAGS, "-I${withval}/include/sigc++-1.2 -I${withval}/lib/sigc++-1.2/include -I${withval}/include")
      AC_SUBST(XCF_LDFLAGS, "-L${withval}/lib")
    fi ],
  [ AC_SUBST(WITH_XCF, "NOXCF_")
    AC_SUBST(XCF_CPPFLAGS, "")
    AC_SUBST(XCF_LDFLAGS, "")])

dnl TO DO: test for an XCF header (at least)

dnl AC_PROG_LIBTOOL
dnl AC_PROG_INSTALL
dnl AC_PROG_AWK
AC_SUBST(PACKAGE_VERSION)

dnl AC_OUTPUT(Makefile nepumuk.pc Doxyfile)
dnl AC_CONFIG_FILES([Makefile simul/Makefile])
AC_CONFIG_HEADERS([config.h])
AC_OUTPUT([Makefile simul/Makefile])
