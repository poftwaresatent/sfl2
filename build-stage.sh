#!/bin/bash

PREFIX="$PWD/stage"
EXTRA_CFGOPTS=""
RUN_BOOTSTRAP="yes"
WORKDIR="build"

while [ ! -z "$1" ]; do
    case $1 in
	-h|--help)
echo 'build-stage.sh command line options:'
echo '  [-p|--prefix]  <PREFIX>     install prefix ($PREFIX)'
echo '  [-w|--work]    <DIR>        build work directory ($WORKDIR)'
echo '  [-b|--boost]   <DIR>        BOOST library install directory'
echo '  [-n|--png]     <DIR>        PNG install library directory'
echo '  [-f|--sfl]     <DIR>        libsunflower install directory'
echo '  [-e|--estar]   <DIR>        E* install directory'
echo '  [-a|--asl]     <DIR>        ASL install directory'
echo '  [-d|--debug]                enable debug messages and symbols'
echo '  [-s|--skipbs]               do not bootstrap build system'
echo '  [-q|--qt]                   enable Qt support (experimental)'
        exit 0;;
	-p|--prefix)
	    PREFIX=$2;
	    shift; shift; continue;;
	-w|--work)
	    WORKDIR=$2;
	    shift; shift; continue;;
	-b|--boost)
	    EXTRA_CFGOPTS="$EXTRA_CFGOPTS --with-boost=$2"
	    shift; shift; continue;;
	-n|--png)
	    EXTRA_CFGOPTS="$EXTRA_CFGOPTS --with-png=$2"
	    shift; shift; continue;;
	-f|--sfl)
	    EXTRA_CFGOPTS="$EXTRA_CFGOPTS --with-sfl=$2"
	    shift; shift; continue;;
	-e|--estar)
	    EXTRA_CFGOPTS="$EXTRA_CFGOPTS --with-estar=$2"
	    shift; shift; continue;;
	-a|--asl)
	    EXTRA_CFGOPTS="$EXTRA_CFGOPTS --with-asl=$2"
	    shift; shift; continue;;
	-d|--debug)
	    EXTRA_CFGOPTS="$EXTRA_CFGOPTS --enable-debug"
	    shift; continue;;
	-s|--skipbs)
	    RUN_BOOTSTRAP="no"
	    shift; continue;;
	-q|--qt)
	    EXTRA_CFGOPTS="$EXTRA_CFGOPTS --enable-qt"
	    shift; continue;;
	*)
	    echo "ERROR unhandled option(s) $*" 1>&2
	    exit 1;;
    esac
done

CFGOPTS="--prefix=$PREFIX $EXTRA_CFGOPTS"

echo "configure options: $CFGOPTS"

if [ $RUN_BOOTSTRAP = "yes" ]; then
    echo "running bootstrap-buildsystem.sh (takes a while)"
    ./bootstrap-buildsystem.sh
    if [ $? -ne 0 ]; then
	echo "ERROR bootstrap-buildsystem.sh"
	exit 1
    fi
fi

rm -rf $WORKDIR
if [ ! -d $WORKDIR ]; then
    mkdir $WORKDIR
    if [ $? -ne 0 ]; then
	echo "ERROR mkdir $WORKDIR"
	exit 1
    fi
fi

cd $WORKDIR
../configure $CFGOPTS
if [ $? -ne 0 ]; then
    echo "ERROR ../configure $CFGOPTS"
    exit 1
fi

make
if [ $? -ne 0 ]; then
    echo "ERROR make"
    exit 1
fi

make install
if [ $? -ne 0 ]; then
    echo "ERROR make install"
    exit 1
fi
