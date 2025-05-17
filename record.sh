#!/bin/bash
echo "Recording... (Ctrl-C to stop)"
cd ~/dev/embed/uha_mc/logs
socat -u FILE:/dev/ttyUSB0,b115200,raw - > log.bin
python log_decoder.py -i log.bin -o out.csv -n 9 -f myplot.png -l theta1,theta2,theta1d,theta2d,tens1,tens2,tens1d,tens2d,tape -p 4,5
if [ "$1" = "-f" ]; then
    pinta myplot.png &
fi
cd ~/dev/embed/uha_mc/

