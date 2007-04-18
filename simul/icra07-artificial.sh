#!/bin/bash

ARGS="-f -l layout.icra07-artificial -r robots.icra07-artificial -W world.icra07-artificial"

echo "ARGS: $ARGS"
./nepumuk $ARGS
