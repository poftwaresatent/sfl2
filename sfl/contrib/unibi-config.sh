#!/bin/bash

CPPFLAGS=-I/vol/mobirob/include CFLAGS=-I/vol/mobirob/include CXXFLAGS=-I/vol/mobirob/include CC=/vol/gcc/bin/gcc CXX=/vol/gcc/bin/g++ PKG_CONFIG_PATH=/vol/mobirob/lib/pkgconfig ../configure --prefix=/vol/mobirob/
