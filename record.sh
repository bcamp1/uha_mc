#!/bin/bash
echo "Recording... (Ctrl-C to stop)"
cd ~/dev/embed/uha_mc/logs
socat -u FILE:/dev/ttyUSB0,b115200,raw - > log.bin
# python log_decoder.py -i log.bin -o out.csv -n 9 -f myplot.png -l theta1,theta2,theta1d,theta2d,tens1,tens2,tens1d,tens2d,tape -p 2,3,4,5
python log_decoder.py -i log.bin -o out.csv -n 11 -f myplot.png -l time,theta1,theta2,theta1_dot,theta2_dot,tension1,tension2,tension1_dot,tension2_dot,tape_position,tape_speed -p 1,2,3,4,5,6,7,8,9,10
if [ "$1" = "-f" ]; then
    pinta myplot.png &
fi
cd ~/dev/embed/uha_mc/

