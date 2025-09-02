#!/bin/bash
./d && sleep 0.3

if [ "$1" = "-b" ]; then
    cmake -S $UHA -B $UHA/build
fi

if [ "$1" = "-c" ]; then
    cd build
    make clean
    cd ..
fi

cd build
make flash || exit $?
cd ..

if [ "$1" = "-u" ]; then
    screen /dev/ttyUSB0 115200
fi

if [ "$1" = "-e" ]; then
    ./e
fi

