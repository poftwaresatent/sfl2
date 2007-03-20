#!/bin/bash

ARGS="-f -m travmap.smarttest -l layout.smarttest -r robots.smarttest -W world.smarttest"

echo "ARGS: $ARGS"
./nepumuk $ARGS
