#!/bin/bash
echo "Emergency Shutdown..."
trap "echo 'Caught Ctrl+C, exiting.'; exit 1" SIGINT
while true; do
    echo -n "d" | socat -u - /dev/ttyUSB0,raw,echo=0,b115200
done

