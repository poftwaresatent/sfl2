AC_REVISION($Id$)
AC_INIT(libsunflower,0.8pre)
AC_PREREQ(2.57)

AC_CANONICAL_HOST

AC_PROG_CC
if test "x${GCC}" = "xyes"; then
   if [ -z "$CFLAGS" ]; then
     CFLAGS="-Wall -pipe"
   else
     CFLAGS=`echo $CFLAGS | sed 's:-O.::'`
     CFLAGS="$CFLAGS -Wall -pipe"
   fi
fi

AC_PROG_CXX
if test "x${GXX}" = "xyes"; then
   if [ -z "$CXXFLAGS" ]; then
     CXXFLAGS="-Wall -pipe"
   else
     CXXFLAGS=`echo $CXXFLAGS | sed 's:-O.::'`
     CXXFLAGS="$CXXFLAGS -Wall -pipe"
   fi
fi

AC_ARG_WITH(boost,
  AC_HELP_STRING([--with-boost=PATH], [specify boost install dir)]),
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

AC_ARG_WITH(sfl,
  AC_HELP_STRING([--with-sfl=PATH], [specify sunflower install dir)]),
  [
    if test "x$withval" = "x" ; then
      AC_SUBST(SFL_CPPFLAGS, "")
      AC_SUBST(SFL_LDFLAGS, "")
    else
      AC_SUBST(SFL_CPPFLAGS, "-I${withval}/include")
      AC_SUBST(SFL_LDFLAGS, "-L${withval}/lib")
    fi
  ],[
    AC_SUBST(SFL_CPPFLAGS, "")
    AC_SUBST(SFL_LDFLAGS, "")])

AC_PROG_LIBTOOL
AC_PROG_INSTALL
AC_PROG_AWK

AC_SUBST(PACKAGE_VERSION)

AC_OUTPUT(Makefile sunflower.pc Doxyfile)
