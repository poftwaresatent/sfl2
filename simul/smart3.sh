#!/bin/bash

ARGS="-f -m travmap.smart3 -l layout.smart3 -r robots.smart3 -W world.smart3"

echo "ARGS: $ARGS"
./nepumuk $ARGS
