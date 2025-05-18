#!/bin/bash
echo "Toggling Motors..."
echo -n "e" | socat -u - /dev/ttyUSB0,raw,echo=0,b115200
