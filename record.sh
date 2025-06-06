#!/bin/bash
date_str=$(date +"%-m-%-d-%y_h%Hm%Ms%S")

# Data files
bin_file="$UHA/logs/bin/$date_str.bin"
csv_file="$UHA/logs/csv/$date_str.csv"
plot_file="$UHA/logs/plots/$date_str.png"

# Python scripts
bin2csv_file="$UHA/logs/bin2csv.py"
csv2plot_file="$UHA/logs/csv2plot.py"

# Record UART data
echo "Recording to $bin_file... (Ctrl-C to stop)"
stty -F /dev/ttyUSB0 -echo
socat -u FILE:/dev/ttyUSB0,b115200,raw STDOUT > "$bin_file"

# Create CSV
echo "Parsing binary to CSV..."
python "$bin2csv_file" -i "$bin_file"

# Create plot
echo "Generating plot from CSV..."
python "$csv2plot_file" -i "$csv_file"

# Optionally show plot
if [ "$1" = "-f" ]; then
    echo "Showing plot..."
    pinta "$plot_file"
fi

