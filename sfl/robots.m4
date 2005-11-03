dnl  Copyright (c) 2002-2005 LAAS/CNRS                   --  Fri Mar 15 2002
dnl  All rights reserved.
dnl 
dnl  Redistribution  and  use in source   and binary forms,  with or without
dnl  modification, are permitted provided that  the following conditions are
dnl  met:
dnl 
dnl    1. Redistributions  of  source code must  retain  the above copyright
dnl       notice, this list of conditions and the following disclaimer.
dnl    2. Redistributions in binary form must  reproduce the above copyright
dnl       notice,  this list of  conditions and  the following disclaimer in
dnl       the  documentation   and/or  other  materials   provided with  the
dnl       distribution.
dnl 
dnl  THIS SOFTWARE IS PROVIDED BY THE  AUTHOR AND CONTRIBUTORS ``AS IS'' AND
dnl  ANY  EXPRESS OR IMPLIED WARRANTIES, INCLUDING,  BUT NOT LIMITED TO, THE
dnl  IMPLIED WARRANTIES   OF MERCHANTABILITY AND  FITNESS  FOR  A PARTICULAR
dnl  PURPOSE ARE DISCLAIMED.  IN NO  EVENT SHALL THE AUTHOR OR  CONTRIBUTORS
dnl  BE LIABLE FOR ANY DIRECT, INDIRECT,  INCIDENTAL, SPECIAL, EXEMPLARY, OR
dnl  CONSEQUENTIAL DAMAGES (INCLUDING,  BUT  NOT LIMITED TO, PROCUREMENT  OF
dnl  SUBSTITUTE  GOODS OR SERVICES;  LOSS   OF  USE,  DATA, OR PROFITS;   OR
dnl  BUSINESS  INTERRUPTION) HOWEVER CAUSED AND  ON ANY THEORY OF LIABILITY,
dnl  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
dnl  OTHERWISE) ARISING IN ANY WAY OUT OF THE  USE OF THIS SOFTWARE, EVEN IF
dnl  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

dnl --- Look for the LAAS mkdep executable ------------------------------

AC_DEFUN([ROBOT_PROG_MKDEP],
[
   AC_PATH_PROG(MKDEP, mkdep, no, $exec_prefix/bin:$prefix/bin:$PATH)
   if test "$MKDEP" = "no"; then
      AC_MSG_ERROR([You need the mkdep package])
   fi

   AC_CACHE_CHECK(
      [whether mkdep accepts -I/-D options], ac_cv_robot_mkdep,
      [
         if $MKDEP -I foo -D bar 1>/dev/null 2>&1; then
            ac_cv_robot_mkdep=yes;
         else
            ac_cv_robot_mkdep=no;
         fi
      ])
   if test x"${ac_cv_robot_mkdep}" = xno; then
      AC_MSG_ERROR([You need the mkdep package])
   fi 
])
