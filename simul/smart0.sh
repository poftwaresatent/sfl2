#!/bin/bash

ARGS="-f -M travmap.smart0 -l layout.smart0 -r robots.smart0"

echo "ARGS: $ARGS"
./nepumuk $ARGS
