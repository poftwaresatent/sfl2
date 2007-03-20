#!/bin/bash

ARGS="-f -m travmap.smart2 -l layout.smart2 -r robots.smart2 -W world.smart2"

echo "ARGS: $ARGS"
./nepumuk $ARGS
