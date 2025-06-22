import matplotlib.pyplot as plt
import struct
import numpy as np
import argparse
import yaml
import os

def convert_filename_to_datetime(s):
    # Split into date and time parts
    date_part, time_part = s.split('_')
    
    # Parse date
    month, day, year = date_part.split('-')
    
    # Extract hour and minute (ignore seconds)
    hour_str = time_part[1:3]  # e.g., '23' from 'h23'
    minute_str = time_part[4:6]  # e.g., '00' from 'm00'
    
    hour = int(hour_str)
    minute = int(minute_str)
    
    # Convert to 12-hour time with am/pm
    ampm = 'am' if hour < 12 else 'pm'
    hour12 = hour % 12
    if hour12 == 0:
        hour12 = 12
    
    # Return formatted string
    return f"{int(month)}/{int(day)}/{int(year)} {hour12}:{minute:02d}{ampm}"

# Process arguments
# Set up the argument parser
parser = argparse.ArgumentParser(description="Process serial binary data and save it as a CSV file.")
parser.add_argument('-c', type=str, help='Path to config YAML file', default='$UHA/logs/decoder_config.yaml')
parser.add_argument('-i', type=str, help='Path to binary input file')
args = parser.parse_args()
config_file = os.path.expandvars(args.c)
config_stream = open(config_file, 'r');
config = yaml.safe_load(config_stream)

input_file = args.i
file_prefix = str.split(str.split(input_file, '/')[-1], '.')[0]
plot_file = os.path.expandvars(config['plots_dir'] + file_prefix + '.png')
data_info = config['data_info']
plot_settings = config['plot_settings']

show_xlims = plot_settings.get('show_xlims', False)
x_min = plot_settings.get('x_min', 0)
x_max = plot_settings.get('x_max', 0)

show_ylims = plot_settings.get('show_ylims', False)
y_min = plot_settings.get('y_min', 0)
y_max = plot_settings.get('y_max', 0)

# Load data
data = np.loadtxt(input_file, delimiter=',')
time = data[:, 0]
time_start = time[0]
time = time - time_start

# Gather plot numbers
plot_numbers = []
plot_ylabels = []
plot_names = []
for index, info in enumerate(data_info):
    if info['plot']:
        plot_numbers.append(index)
        plot_ylabels.append(info['ylabel'])
        plot_names.append(info['name'])

plot_ylabel = plot_ylabels[0]
plot_title = "SuperDeck Log " + convert_filename_to_datetime(file_prefix)

print('Saving figure to ' + plot_file + '...')
# Plot each column with a different color
for i in plot_numbers:
    info = data_info[i]
    # plt.plot(time, data[:, i], label=info['name'], linestyle='-', marker='o', linewidth=1, markersize=1.5)
    plt.plot(time, data[:, i], label=info['name'], linestyle='-', linewidth=1)

if 'Tape Speed' in plot_names:
    plt.axhline(y=15, color='black', linewidth=0.5)

if show_xlims:
    plt.xlim(x_min, x_max)

if show_ylims:
    plt.ylim(y_min, y_max)

plt.xlabel('Time (s)')
plt.ylabel(plot_ylabel)
plt.title(plot_title)
plt.legend()
plt.grid(True)

# Show the plot
plt.savefig(plot_file, dpi=300)

