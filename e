#!/bin/bash
echo "Enabling Motors..."
trap "echo 'Caught Ctrl+C, exiting.'; exit 1" SIGINT
for i in {1..20}; do
    echo -n "e" | socat -u - /dev/ttyUSB0,raw,echo=0,b115200
done
