AC_REVISION($Id$)
AC_INIT(libsunflower,0.6)
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

AC_PATH_PROG(MKDEP, mkdep, no, $prefix/bin:$exec_prefix/bin:$PATH)

AC_LANG_PUSH(C++)
AC_CHECK_HEADERS(boost/shared_ptr.hpp, [], AC_MSG_ERROR(BOOST not found))
AC_LANG_POP

AC_ARG_WITH([xcf],
            AC_HELP_STRING([--with-xcf],
                           [enable XCF support]),
	    AC_SUBST(WITH_XCF, "XCF_"),
            AC_SUBST(WITH_XCF, "NOXCF_"))

AC_PROG_LIBTOOL

AC_PROG_INSTALL

AC_SUBST(PACKAGE_VERSION)

AC_OUTPUT(Makefile sunflower.pc)

