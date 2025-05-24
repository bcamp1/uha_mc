#!/bin/bash
date_str=$(date +"%-m-%-d-%y_h%Hm%Ms%S")
bin_file="$UHA/logs/bin/$date_str.bin"
plot_file="$UHA/logs/plots/$date_str.png"
python_file="$UHA/logs/log_decoder.py"
echo "Recording to $bin_file... (Ctrl-C to stop)"
socat -u FILE:/dev/ttyUSB0,b115200,raw - > "$bin_file"
echo "Parsing binary to CSV..."
python "$python_file" -i "$bin_file"
if [ "$1" = "-f" ]; then
    echo "Showing plot..."
    pinta "$plot_file"
fi

