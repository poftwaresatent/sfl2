#!/bin/bash

echo "redo.sh: removing generated and temporary stuff"
rm -rf build configure autom4te.cache aclocal.m4 

echo "redo.sh: running autogen.sh"
./autogen.sh

echo "redo.sh: setting up build dir"
mkdir build
cd build
../configure --prefix=/home/rolo/local

echo "redo.sh: compiling"
make
