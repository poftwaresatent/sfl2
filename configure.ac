AC_REVISION($Id$)
AC_INIT(nepumuk, 0.0pre, libsunflower-devel AT lists DOT sourceforge DOT net)
AC_PREREQ(2.57)
AM_INIT_AUTOMAKE([-Wall -Werror foreign])

AC_CANONICAL_HOST
case $host_os in
  linux*)  OSCPPFLAGS="-DLINUX"
dnl	   AC_SUBST(GLUTLIBS, "-lglut -lGLU -lGL -lXi -lXmu");;
	   AC_SUBST(GLUTLIBS, "-lglut -lGLU -lGL");;
  darwin*) OSCPPFFLAGS="-DDARWIN"
           OSCXXFLAGS="-Wno-long-double"
dnl        OSLDFLAGS="-dynamiclib -undefined suppress -Wl,-flat_namespace"
	   AC_SUBST(GLUTLIBS, "-framework OpenGL -framework GLUT");;
  *)       echo "WARNING unknown host_os $host_os, update configure.ac";;
esac
CPPFLAGS="$CPPFLAGS $OSCPPFLAGS"
CXXFLAGS="$CXXFLAGS $OSCXXFLAGS"
dnl TO DO: only for dynamic libraries...
dnl LDFLAGS="$LDFLAGS $OSLDFLAGS"

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

dnl TO-DO   AC_ARG_WITH(boost,
dnl TO-DO     AC_HELP_STRING([--with-boost=PATH], [specify boost install dir]),
dnl TO-DO     [ if test "x$withval" != "x" ; then
dnl TO-DO         CPPFLAGS="$CPPFLAGS -I${withval}/include"
dnl TO-DO         LDFLAGS="$LDFLAGS -L${withval}/lib"
dnl TO-DO       fi ], [])
dnl TO-DO   oldCXXFLAGS="$CXXFLAGS"
dnl TO-DO   CXXFLAGS="$CXXFLAGS $CPPFLAGS -I$prefix/include"
dnl TO-DO   AC_CHECK_HEADER([boost/shared_ptr.hpp], [], [
dnl TO-DO     AC_MSG_ERROR([can't compile boost/shared_ptr.hpp (CXXFLAGS=$CXXFLAGS)])], [])
dnl TO-DO   CXXFLAGS="$oldCXXFLAGS"

AC_ARG_WITH(sfl,
  AC_HELP_STRING([--with-sfl=PATH], [specify libsunflower install dir]),
  [ if test "x$withval" != "x" ; then
      CPPFLAGS="$CPPFLAGS -I${withval}/include"
      LDFLAGS="$LDFLAGS -L${withval}/lib"
    fi ], [])
oldCPPFLAGS="$CPPFLAGS"
CPPFLAGS="$CPPFLAGS -I$prefix/include"
AC_CHECK_HEADER([sfl/util/numeric.hpp], [], [
  AC_MSG_ERROR([can't compile sfl/util/numeric.hpp (CPPFLAGS=$CPPFLAGS)])], [])
CPPFLAGS="$oldCPPFLAGS"

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

AC_PROG_RANLIB
dnl AC_PROG_LIBTOOL
dnl AC_PROG_INSTALL
dnl AC_PROG_AWK

AC_SUBST(PACKAGE_VERSION)

AC_CONFIG_HEADERS(config.h)
AC_OUTPUT([Makefile common/Makefile robox/Makefile visitor/Makefile
           laas/Makefile simul/Makefile])
