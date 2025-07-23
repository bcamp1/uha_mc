#!/bin/bash
./d && sleep 0.3

if [ "$1" = "-b" ]; then
    cmake -S $UHA -B $UHA/build
    cd $UHA/build
    make flash || exit $?
    cd $UHA
fi

cd $UHA/build
make flash || exit $?
cd $UHA

if [ "$1" = "-u" ]; then
    screen /dev/ttyUSB0 115200
fi

if [ "$1" = "-e" ]; then
    ./e
fi

