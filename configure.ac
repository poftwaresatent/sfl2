AC_REVISION($Id$)
AC_INIT([nepumuk],
        [0.0pre],
        [libsunflower-devel AT lists DOT sourceforge DOT net])
AC_CONFIG_AUX_DIR([.])
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

NPM_CPPFLAGS=""
NPM_CFLAGS="-pipe"
NPM_LDFLAGS=""

AC_CANONICAL_HOST
case $host_os in
  linux*)  AC_MSG_NOTICE([detected Linux])
           NPM_CPPFLAGS="$NPM_CPPFLAGS -DLINUX"
	   NPM_LDFLAGS="-lpthread $NPM_LDFLAGS"
           NPM_GLUTLIBS="-lglut -lGLU -lGL"
	   oldCPPFLAGS="$CPPFLAGS"
	   if test "$prefix" = "NONE"; then
	     CPPFLAGS="$NPM_CPPFLAGS"
	   else
	     CPPFLAGS="$NPM_CPPFLAGS -I$prefix/include"
	   fi
	   AC_CHECK_HEADER([GL/glut.h], [],
	     [ AC_MSG_ERROR([cannot find GL/glut.h]) ])
           CPPFLAGS="$oldCPPFLAGS"
           AC_DISABLE_STATIC;;
  *openbsd*) AC_MSG_NOTICE([detected OpenBSD])
             NPM_CPPFLAGS="$NPM_CPPFLAGS -DOPENBSD -I/usr/local/include -I/usr/X11R6/include"
	     NPM_LDFLAGS="-lpthread $NPM_LDFLAGS -L/usr/local/lib -L/usr/X11R6/lib"

             NPM_GLUTLIBS="-lX11 -lXi -lXmu -lglut -lGLU -lGL"
             oldCPPFLAGS="$CPPFLAGS"
             if test "$prefix" = "NONE"; then
               CPPFLAGS="$NPM_CPPFLAGS"
             else
               CPPFLAGS="$NPM_CPPFLAGS -I$prefix/include"
             fi
             AC_CHECK_HEADER([GL/glut.h], [],
               [ AC_MSG_ERROR([Nepumuk requires GLUT development packages (which in turn requires GL and GLU)])])
             CPPFLAGS="$oldCPPFLAGS"
             AC_DISABLE_STATIC;;
  darwin*) AC_MSG_NOTICE([detected Darwin (OS X)])
           NPM_CPPFLAGS="$NPM_CPPFLAGS -DOSX -Wno-long-double"
           NPM_GLUTLIBS="-framework OpenGL -framework GLUT -lobjc"
	   oldCPPFLAGS="$CPPFLAGS"
	   if test "$prefix" = "NONE"; then
	     CPPFLAGS="$NPM_CPPFLAGS"
	   else
	     CPPFLAGS="$NPM_CPPFLAGS -I$prefix/include"
	   fi
	   AC_CHECK_HEADER([GLUT/glut.h], [],
	     [ AC_MSG_ERROR([cannot find GLUT/glut.h]) ])
           CPPFLAGS="$oldCPPFLAGS"
           AC_DISABLE_SHARED;;
  *)       AC_MSG_WARN([Unknown host_os $host_os, build will probably break.])
           AC_MSG_WARN([Please update configure.ac.]);;
esac
AC_PROG_LIBTOOL
AC_PROG_INSTALL
AC_PROG_EGREP

AC_ARG_ENABLE(debug,
  AS_HELP_STRING([--enable-debug], [GCC options -g -O0 (else -O3)]),
  [ NPM_CPPFLAGS="$NPM_CPPFLAGS -DNPM_DEBUG"
    NPM_CFLAGS="$NPM_CFLAGS -g -O0" ],
  [ NPM_CFLAGS="$NPM_CFLAGS -O3" ])

AC_ARG_ENABLE(pedantic,
  AS_HELP_STRING([--enable-pedantic], [GCC options -pedantic (else -Wall)]),
  [ NPM_CFLAGS="$NPM_CFLAGS -pedantic" ],
  [ NPM_CFLAGS="$NPM_CFLAGS -Wall" ])

AC_ARG_ENABLE(qt,
  AC_HELP_STRING([--enable-qt], [enable QtNepumuk (experimental)]),
  [ AM_CONDITIONAL([NPM_ENABLE_QT], [true]) ],
  [ AM_CONDITIONAL([NPM_ENABLE_QT], [false]) ])

AC_ARG_WITH(boost,
  AS_HELP_STRING([--with-boost=PATH], [specify boost install dir]),
  [ if test "x$withval" != "x" ; then
      NPM_CPPFLAGS="$NPM_CPPFLAGS -I${withval}/include -I${withval}"
      NPM_LDFLAGS="-L${withval}/lib -I${withval} $NPM_LDFLAGS"
    fi ], [])
oldCPPFLAGS="$CPPFLAGS"
if test "$prefix" = "NONE"; then
   CPPFLAGS="$NPM_CPPFLAGS"
else
   CPPFLAGS="$NPM_CPPFLAGS -I$prefix/include"
fi
AC_CHECK_HEADER([boost/shared_ptr.hpp], [], [
  AC_MSG_ERROR([cannot compile boost/shared_ptr.hpp (CPPFLAGS=$CPPFLAGS)])])
CPPFLAGS="$oldCPPFLAGS"

AC_ARG_WITH(sfl,
  AS_HELP_STRING([--with-sfl=PATH], [specify libsunflower install dir]),
  [ if test "x$withval" != "x" ; then
      NPM_CPPFLAGS="$NPM_CPPFLAGS -I${withval}/include -I${withval}"
      NPM_LDFLAGS="-L${withval}/lib -I${withval} $NPM_LDFLAGS"
    fi ], [])
oldCPPFLAGS="$CPPFLAGS"
if test "$prefix" = "NONE"; then
   CPPFLAGS="$NPM_CPPFLAGS"
else
   CPPFLAGS="$NPM_CPPFLAGS -I$prefix/include"
fi
AC_CHECK_HEADER([sfl/util/numeric.hpp], [], [
  AC_MSG_ERROR([cannot compile sfl/util/numeric.hpp (CPPFLAGS=$CPPFLAGS)])])
CPPFLAGS="$oldCPPFLAGS"

NPM_XCF_CPPFLAGS=""
NPM_XCF_LDFLAGS=""
AC_ARG_WITH(xcf,
  AC_HELP_STRING([--with-xcf=PATH], [enable XCF (installed in PATH)]),
  [ if test "x$withval" = "x" ; then
      if test "$prefix" != "NONE"; then
        NPM_XCF_CPPFLAGS="-I$prefix/include/sigc++-1.2 -I$prefix/lib/sigc++-1.2/include"
      fi
      NPM_XCF_LDFLAGS="-lsigc-1.2 -lxcf -lxmltio"
    else
      NPM_XCF_CPPFLAGS="-I${withval}/include/sigc++-1.2 -I${withval}/lib/sigc++-1.2/include -I${withval}/include -I${withval}"
      NPM_XCF_LDFLAGS="-lsigc-1.2 -lxcf -lxmltio -L${withval}/lib -L${withval}"
    fi
    NPM_CPPFLAGS="$NPM_CPPFLAGS -DNPM_HAVE_XCF"
    oldCPPFLAGS="$CPPFLAGS"
    if test "$prefix" = "NONE"; then
      CPPFLAGS="$NPM_CPPFLAGS $XCF_CPPFLAGS"
    else
      CPPFLAGS="$NPM_CPPFLAGS $XCF_CPPFLAGS -I$prefix/include"
    fi
    AC_CHECK_HEADER([xcf/Subscriber.hpp], [], [
      AC_MSG_ERROR([cannot compile xcf/Subscriber.hpp (CPPFLAGS=$CPPFLAGS)])], [])
    CPPFLAGS="$oldCPPFLAGS"
    AM_CONDITIONAL([NPM_HAVE_XCF], [true]) ],
  [ AM_CONDITIONAL([NPM_HAVE_XCF], [false]) ])

AC_ARG_WITH(xcfglue,
  AC_HELP_STRING([--with-xcfglue=PATH], [special XCFGLUE install PATH]),
  [ if test ! -z "$NPM_HAVE_XCF_TRUE"; then
      AC_MSG_ERROR([--with-xcfglue requires --with-xcf])
    fi
    if test "x$withval" != "x" ; then
      NPM_XCF_CPPFLAGS="$NPM_XCF_CPPFLAGS -I${withval}/include -I${withval}"
      NPM_XCF_LDFLAGS="-lxcfglue -L${withval}/lib -L${withval} $NPM_XCF_LDFLAGS"
    fi ], [])

if test -z "$NPM_HAVE_XCF_TRUE"; then
  oldCPPFLAGS="$CPPFLAGS"
  if test "$prefix" = "NONE"; then
    CPPFLAGS="$NPM_CPPFLAGS $NPM_XCF_CPPFLAGS"
  else
    CPPFLAGS="$NPM_CPPFLAGS $NPM_XCF_CPPFLAGS -I$prefix/include"
  fi
  AC_CHECK_HEADER([xcfwrap/xcfglue.h], [], [
    AC_MSG_ERROR([cannot compile xcfwrap/xcfglue.h (CPPFLAGS=$CPPFLAGS)])], [])
  CPPFLAGS="$oldCPPFLAGS"
fi

NPM_ESTAR_CPPFLAGS=""
NPM_ESTAR_LDFLAGS=""
AC_ARG_WITH(estar,
  AC_HELP_STRING([--with-estar=PATH], [enable E-Star (installed in PATH)]),
  [ if test "x$withval" = "x" ; then
      NPM_ESTAR_CPPFLAGS="-DNPM_HAVE_ESTAR"
      NPM_ESTAR_LDFLAGS="-lestar"
    else
      NPM_ESTAR_CPPFLAGS="-DNPM_HAVE_ESTAR -I${withval}/include -I${withval}"
      NPM_ESTAR_LDFLAGS="-lestar -L${withval}/lib -L${withval}"
    fi
    AM_CONDITIONAL([NPM_HAVE_ESTAR], [true]) ],
  [ AM_CONDITIONAL([NPM_HAVE_ESTAR], [false]) ])

AC_ARG_WITH(asl,
  AC_HELP_STRING([--with-asl=PATH], [ASL 'standard software' install PATH]),
  [ if test ! -z "$NPM_HAVE_ESTAR_TRUE"; then
      AC_MSG_ERROR([--with-asl makes no sense without --with-estar])
    fi
    if test "x$withval" != "x" ; then
      NPM_ESTAR_CPPFLAGS="$NPM_ESTAR_CPPFLAGS -DNPM_HAVE_ASL -I${withval}/include -I${withval}"
      NPM_ESTAR_LDFLAGS="-lasl-mplan -lasl-mcontrol -L${withval}/lib -L${withval} $NPM_ESTAR_LDFLAGS"
    else
      NPM_ESTAR_CPPFLAGS="$NPM_ESTAR_CPPFLAGS -DNPM_HAVE_ASL"
      NPM_ESTAR_LDFLAGS="-lasl-mplan -lasl-mcontrol $NPM_ESTAR_LDFLAGS"
    fi
    oldCPPFLAGS="$CPPFLAGS"
    if test "$prefix" = "NONE"; then
      CPPFLAGS="$NPM_CPPFLAGS $NPM_ESTAR_CPPFLAGS"
    else
      CPPFLAGS="$NPM_CPPFLAGS $NPM_ESTAR_CPPFLAGS -I$prefix/include"
    fi
    AC_CHECK_HEADER([asl/path_tracking.hpp], [], [
      AC_MSG_ERROR([cannot compile asl/path_tracking.hpp (CPPFLAGS=$CPPFLAGS)])], [])
    CPPFLAGS="$oldCPPFLAGS"
    AM_CONDITIONAL([NPM_HAVE_ASL], [true]) ],
  [ AM_CONDITIONAL([NPM_HAVE_ASL], [false]) ])

if test -z "$NPM_HAVE_ESTAR_TRUE"; then
  oldCPPFLAGS="$CPPFLAGS"
  if test "$prefix" = "NONE"; then
    CPPFLAGS="$NPM_CPPFLAGS $NPM_ESTAR_CPPFLAGS"
  else
    CPPFLAGS="$NPM_CPPFLAGS $NPM_ESTAR_CPPFLAGS -I$prefix/include"
  fi
  AC_CHECK_HEADER([estar/Facade.hpp], [], [
    AC_MSG_ERROR([cannot compile estar/Facade.hpp (CPPFLAGS=$CPPFLAGS)])], [])
  CPPFLAGS="$oldCPPFLAGS"
fi

AC_ARG_WITH(genom,
  AC_HELP_STRING([--with-genom=PATH], [specify openrobots install dir]),
  [ if test "x$withval" != "x" ; then
      NPM_CPPFLAGS="$NPM_CPPFLAGS -DNPM_HAVE_GENOM -I${withval}/include -I${withval}"
      NPM_LDFLAGS="-L${withval}/lib -L${withval} -lposterLib $NPM_LDFLAGS"
    else
      NPM_LDFLAGS="-lposterLib $NPM_LDFLAGS"
    fi
    AM_CONDITIONAL([NPM_HAVE_GENOM], [true])
    oldCPPFLAGS="$CPPFLAGS"
    if test "$prefix" = "NONE"; then
      CPPFLAGS="$NPM_CPPFLAGS"
    else
      CPPFLAGS="$NPM_CPPFLAGS -I$prefix/include"
    fi
    AC_CHECK_HEADER([posterLib.h], [], [
      AC_MSG_ERROR([cannot compile posterLib.h (CPPFLAGS=$CPPFLAGS)])], [])
    CPPFLAGS="$oldCPPFLAGS"
  ],[
    AM_CONDITIONAL([NPM_HAVE_GENOM], [false])])

NPM_ROS_CPPFLAGS=""
NPM_ROS_LDFLAGS=""
AC_ARG_ENABLE(ros,
  AC_HELP_STRING([--enable-ros], [enable ROS (rospack must be in PATH)]),
  [ NPM_ROS_CPPFLAGS="-DNPM_HAVE_ROS"
    for component in roscpp std_msgs; do
      cflags=`rospack export/cpp/cflags $component`
      if test "x$cflags" = "x"; then
        AC_MSG_ERROR([cannot rospack export/cpp/cflags $component])
      fi
      NPM_ROS_CPPFLAGS="$NPM_ROS_CPPFLAGS $cflags"
      AC_MSG_NOTICE([$component: cflags are $cflags])
      lflags=`rospack export/cpp/lflags $component`
      if test "x$lflags" = "x"; then
        AC_MSG_ERROR([cannot rospack export/cpp/lflags $component])
      fi
      NPM_ROS_LDFLAGS="$lflags $NPM_ROS_LDFLAGS"
      AC_MSG_NOTICE([$component: lflags are $lflags])
    done
    AM_CONDITIONAL([NPM_HAVE_ROS], [true])
    oldCPPFLAGS="$CPPFLAGS"
    if test "$prefix" = "NONE"; then
      CPPFLAGS="$NPM_CPPFLAGS $NPM_ROS_CPPFLAGS"
    else
      CPPFLAGS="$NPM_CPPFLAGS $NPM_ROS_CPPFLAGS -I$prefix/include"
    fi
    AC_CHECK_HEADER([rosthread/mutex.h], [], [
      AC_MSG_ERROR([cannot compile rosthread/mutex.h (CPPFLAGS=$CPPFLAGS)])], [])
    AC_CHECK_HEADER([XmlRpc.h], [], [
      AC_MSG_ERROR([cannot compile XmlRpc.h (CPPFLAGS=$CPPFLAGS)])], [])
    AC_CHECK_HEADER([ros/node.h], [], [
      AC_MSG_ERROR([cannot compile ros/node.h (CPPFLAGS=$CPPFLAGS)])], [])
    AC_CHECK_HEADER([rostools/Header.h], [], [
      AC_MSG_ERROR([cannot compile rostools/Header.h (CPPFLAGS=$CPPFLAGS)])], [])
    AC_CHECK_HEADER([std_msgs/BaseVel.h], [], [
      AC_MSG_ERROR([cannot compile std_msgs/BaseVel.h (CPPFLAGS=$CPPFLAGS)])], [])
    AC_CHECK_HEADER([std_msgs/LaserScan.h], [], [
      AC_MSG_ERROR([cannot compile std_msgs/LaserScan.h (CPPFLAGS=$CPPFLAGS)])], [])
    AC_CHECK_HEADER([std_msgs/RobotBase2DOdom.h], [], [
      AC_MSG_ERROR([cannot compile std_msgs/RobotBase2DOdom.h (CPPFLAGS=$CPPFLAGS)])], [])
    CPPFLAGS="$oldCPPFLAGS"
  ],[
    AM_CONDITIONAL([NPM_HAVE_ROS], [false])])

AC_ARG_WITH(png,
  AC_HELP_STRING([--with-png=PATH], [specify libpng install dir]),
  [ if test "x$withval" != "x" ; then
      NPM_CPPFLAGS="$NPM_CPPFLAGS -I${withval}/include -I${withval}"
      NPM_LDFLAGS="-L${withval}/lib -L${withval} $NPM_LDFLAGS"
    fi ], [])
oldCPPFLAGS="$CPPFLAGS"
if test "$prefix" = "NONE"; then
  CPPFLAGS="$NPM_CPPFLAGS"
else
  CPPFLAGS="$NPM_CPPFLAGS -I$prefix/include"
fi
AC_CHECK_HEADERS([png.h],
  [ NPM_LDFLAGS="-lpng -lz $NPM_LDFLAGS" ],
  [ AC_MSG_NOTICE([PNG support disabled (CPPFLAGS=$CPPFLAGS)]) ])
CPPFLAGS="$oldCPPFLAGS"

ABSTOPBUILDDIR=`pwd`
if test -z "$ABSTOPBUILDDIR" ; then
  AC_MSG_ERROR([could not determine current working directory])
fi
if test "$prefix" = "NONE" ; then
  NPM_CPPFLAGS="-I${ABSTOPBUILDDIR}/include $NPM_CPPFLAGS"
else
  NPM_CPPFLAGS="-I${ABSTOPBUILDDIR}/include $NPM_CPPFLAGS -I${prefix}/include"
  NPM_LDFLAGS="$NPM_LDFLAGS -L$prefix/lib"
fi
NPM_CXXFLAGS="$NPM_CFLAGS"

AM_CONDITIONAL([NPM_SMARTFLOWER], [false])

AC_SUBST(PACKAGE_VERSION)
AC_SUBST(NPM_CPPFLAGS)
AC_SUBST(NPM_CFLAGS)
AC_SUBST(NPM_CXXFLAGS)
AC_SUBST(NPM_LDFLAGS)
AC_SUBST(NPM_GLUTLIBS)
AC_SUBST(NPM_XCF_CPPFLAGS)
AC_SUBST(NPM_XCF_LDFLAGS)
AC_SUBST(NPM_ROS_CPPFLAGS)
AC_SUBST(NPM_ROS_LDFLAGS)
AC_SUBST(NPM_ESTAR_CPPFLAGS)
AC_SUBST(NPM_ESTAR_LDFLAGS)
AC_CONFIG_HEADERS(config.h)
AC_CONFIG_FILES([Makefile common/Makefile robox/Makefile visitor/Makefile
                ros/Makefile laas/Makefile biron/Makefile estar/Makefile asl/Makefile
                simul/Makefile qt/Makefile qt/QtNepumuk.pro])
AC_OUTPUT
