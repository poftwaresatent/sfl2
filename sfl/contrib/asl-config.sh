#!/bin/bash

ENVROOT="/users/fred/robox-linux/build/ppc_buildroot_v012"
SRCROOT="/users/fred/robox-linux/build/ppc_buildroot_v012/src/rolo/sunflower"

$ENVROOT/env-ppc-linux \
  $SRCROOT/configure \
  --prefix=/robox-linux/ \
  --with-tcl=/robox-linux/lib/ \
  --without-rtai \
  --build=i386-linux \
  --host=powerpc-linux \
  --with-genom=/robox-linux/ \
  --with-pocolibs=/robox-linux/ \
  --with-boost=/robox-linux
