#!/bin/bash
if [ "$1" = "-b" ]; then
    cmake -S $UHA -B $UHA/build
fi

cd $UHA/build
make flash || exit $?
cd $UHA

if [ "$1" = "-u" ]; then
    screen /dev/ttyUSB0 115200
fi

