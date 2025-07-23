#!/bin/bash
echo "Disabling Motors..."
trap "echo 'Caught Ctrl+C, exiting.'; exit 1" SIGINT
for i in {1..1}; do
    echo -n "p" | socat -u - /dev/ttyUSB0,raw,echo=0,b115200
done

