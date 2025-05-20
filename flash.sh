#!/bin/bash
cd build
make flash
cd ..

if [ "$1" = "-u" ]; then
    screen /dev/ttyUSB0 115200
fi

