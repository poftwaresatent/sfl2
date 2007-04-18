#!/bin/bash

ARGS="-f -l layout.icra07-logreplay -r robots.icra07-logreplay -W world.icra07-logreplay"

echo "ARGS: $ARGS"
./nepumuk $ARGS
