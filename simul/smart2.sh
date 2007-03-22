#!/bin/bash

ARGS="-f -M travmap.smart2 -l layout.smart2 -r robots.smart2"

echo "ARGS: $ARGS"
./nepumuk $ARGS
