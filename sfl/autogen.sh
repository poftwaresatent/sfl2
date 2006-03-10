#! /bin/sh

srcdir=`dirname $0`
test -z "$srcdir" && srcdir=.

origdir=`pwd`
cd $srcdir

echo -n "Running aclocal..."
aclocal -I .
if [ $? -eq 0 ]; then
    echo "OK"
else
    echo "failed"
    cd $origdir
    exit 1
fi

echo -n "Running autoconf..."
autoconf
if [ $? -eq 0 ]; then
    echo "OK"
else
    echo "FAILED"
    echo "==> try removing $srcdir/autom4te.cache"
    cd $origdir
    exit 1
fi

cd $origdir
