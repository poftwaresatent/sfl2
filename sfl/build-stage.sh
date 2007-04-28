#!/bin/bash
PREFIX=$PWD/stage
./bootstrap-buildsystem.sh
if [ $? -ne 0 ]; then
    echo "ERROR bootstrap-buildsystem.sh"
    exit 1
fi

rm -rf build
mkdir build
if [ $? -ne 0 ]; then
    echo "ERROR mkdir build"
    exit 1
fi

cd build
../configure --prefix=$PREFIX
if [ $? -ne 0 ]; then
    echo "ERROR ../configure --prefix=$PREFIX"
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
