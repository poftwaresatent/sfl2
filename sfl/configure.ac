AC_REVISION($Id$)
AC_INIT(libsunflower,0.9pre)
AC_PREREQ(2.57)

AC_CANONICAL_HOST


AC_PROG_CC
test "x${GCC}" = "xyes" && CFLAGS="-pipe"

AC_PROG_CXX
test "x${GXX}" = "xyes" && CXXFLAGS="-pipe"

AC_ARG_ENABLE(debug,
  AC_HELP_STRING([--enable-debug], [GCC options -g -O0 (else -O2)]),
  [
    test "x${GCC}" = "xyes" && CFLAGS="$CFLAGS -DDEBUG -g -O0"
    test "x${GXX}" = "xyes" && CXXFLAGS="$CXXFLAGS -DDEBUG -g -O0"
  ],[
    test "x${GCC}" = "xyes" && CFLAGS="$CFLAGS -O2"
    test "x${GXX}" = "xyes" && CXXFLAGS="$CXXFLAGS -O2"
  ])

AC_ARG_ENABLE(pedantic,
  AC_HELP_STRING([--enable-pedantic], [GCC option -pedantic (else -Wall)]),
  [
    test "x${GCC}" = "xyes" && CFLAGS="$CFLAGS -pedantic"
    test "x${GXX}" = "xyes" && CXXFLAGS="$CXXFLAGS -pedantic"
  ],[
    test "x${GCC}" = "xyes" && CFLAGS="$CFLAGS -Wall"
    test "x${GXX}" = "xyes" && CXXFLAGS="$CXXFLAGS -Wall"
  ])

AC_ARG_WITH(boost,
  AC_HELP_STRING([--with-boost=PATH], [specify boost install dir]),
  [
    if test "x$withval" = "x" ; then
      AC_SUBST(BOOST_CPPFLAGS, "")
      AC_SUBST(BOOST_LDFLAGS, "")
    else
      AC_SUBST(BOOST_CPPFLAGS, "-I${withval}/include")
      AC_SUBST(BOOST_LDFLAGS, "-L${withval}/lib")
    fi
  ],[
    AC_SUBST(BOOST_CPPFLAGS, "")
    AC_SUBST(BOOST_LDFLAGS, "")])

AC_LANG_PUSH([C++])
oldCPPFLAGS="$CPPFLAGS"
CPPFLAGS="$CPPFLAGS $BOOST_CPPFLAGS"
$AC_CHECK_HEADER([boost/shared_ptr.hpp], [],
  [AC_MSG_ERROR([can't compile boost/shared_ptr.hpp (CPPFLAGS=$CPPFLAGS)])], [])
CPPFLAGS="$oldCPPFLAGS"
AC_LANG_POP

AC_ARG_WITH(xcf,
  AC_HELP_STRING([--with-xcf=PATH], [enable XCF (installed in PATH)]),
  [
    AC_SUBST(WITH_XCF, "XCF_")
    if test "x$withval" = "x" ; then
      AC_SUBST(XCF_CPPFLAGS, "")
      AC_SUBST(XCF_LDFLAGS, "")
    else
      AC_SUBST(XCF_CPPFLAGS,
               "-I${withval}/include/sigc++-1.2 -I${withval}/lib/sigc++-1.2/include -I${withval}/include")
      AC_SUBST(XCF_LDFLAGS, "-L${withval}/lib")
    fi
  ],[
    AC_SUBST(WITH_XCF, "NOXCF_")
    AC_SUBST(XCF_CPPFLAGS, "")
    AC_SUBST(XCF_LDFLAGS, "")])

AC_ARG_WITH(robox,
  AC_HELP_STRING([--with-robox=PATH], [Robox (installed in PATH)]),
  [
    AC_SUBST(WITH_ROBOX, "ROBOX_")
    if test "x$withval" = "x" ; then
      AC_SUBST(ROBOX_CPPFLAGS, "")
      AC_SUBST(ROBOX_LDFLAGS, "")
    else
      AC_SUBST(ROBOX_CPPFLAGS, "-I${withval}/include")
      AC_SUBST(ROBOX_LDFLAGS, "-L${withval}/lib")
    fi
  ],[
    AC_SUBST(WITH_ROBOX, "NOROBOX_")
    AC_SUBST(ROBOX_CPPFLAGS, "")
    AC_SUBST(ROBOX_LDFLAGS, "")])

AC_PROG_LIBTOOL
AC_PROG_INSTALL
AC_PROG_AWK

AC_SUBST(PACKAGE_VERSION)

AC_OUTPUT(Makefile sunflower.pc Doxyfile)
