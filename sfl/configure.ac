AC_REVISION($Id$)
AC_INIT(libsunflower,0.3)
AC_PREREQ(2.57)

AC_CANONICAL_HOST

AC_PROG_CC
if test "x${GCC}" = "xyes"; then
  CFLAGS="${CFLAGS} -Wall"
fi

AC_PROG_CXX
if test "x${GXX}" = "xyes"; then
  CXXFLAGS="${CXXFLAGS} -Wall"
fi

AC_PROG_MKDEP

AC_PROG_LIBTOOL

AC_PROG_INSTALL

ROBOT_PROG_MKDEP

AC_SUBST(PACKAGE_VERSION)

AC_OUTPUT(Makefile sunflower.pc)

