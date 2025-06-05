import struct
import numpy as np
import argparse
import yaml
import os

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
output_file = config['output_dir'] + file_prefix + '.csv'
plot_file = config['plots_dir'] + file_prefix + '.png'
data_info = config['data_info']

labels = []
for info in data_info:
    labels.append(info['name'])

# Expand bash variables
input_file = os.path.expandvars(input_file)
output_file = os.path.expandvars(output_file)
plot_file = os.path.expandvars(plot_file)

n = len(data_info)

print('Reading ' + input_file + '...')
with open(input_file, 'rb') as f:
    binary_data = list(f.read())

def split_by_sublist(data, delimiter):
    result = []
    i = 0
    start = 0
    while i <= len(data) - len(delimiter):
        if data[i:i+len(delimiter)] == delimiter:
            result.append(data[start:i])
            i += len(delimiter)
            start = i
        else:
            i += 1
    result.append(data[start:])  # append remaining part
    return result

split_list = split_by_sublist(binary_data, [127, 128, 0, 0])

data = []
for sublist in split_list:
    if len(sublist) != 4*n:
        continue
    num_list = []
    for i in range(0, 4*n, 4):
        float_list = sublist[i:i+4]
        b = bytes(float_list)
        num = struct.unpack('>f', b)[0]
        num_list.append(num)
    data.append(num_list)

data = np.array(data)
time = data[:, 0]

# Save to csv
print('Writing to ' + output_file + '...')
np.savetxt(output_file, data, fmt='%.10e', delimiter=',', header=','.join(labels))


