#!/bin/bash

INCLUDES="-I/vol/mobirob/include -I$XCF/include -I$BIRON/include/HardwareControl -I$BIRON/include/RobotDataServer -I/vol/xcf/0.7.1/include/sigc++-1.2 -I/vol/xcf/0.7.1/lib/sigc++-1.2/include -I$BIRON/include/ExecSV"

LINKS="-L$XCF/lib -L$BIRON/lib"

CPPFLAGS="$INCLUDES" CFLAGS="$INCLUDES -g" CXXFLAGS="$INCLUDES -DXCF_07 -g" LDFLAGS="$LINKS" CC=/vol/gcc/bin/gcc CXX=/vol/gcc/bin/g++ PKG_CONFIG_PATH=/vol/mobirob/lib/pkgconfig ../configure --prefix=/vol/mobirob/
