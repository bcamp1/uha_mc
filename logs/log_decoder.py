import struct
import numpy as np
import matplotlib.pyplot as plt
import argparse

# Process arguments
# Set up the argument parser
parser = argparse.ArgumentParser(description="Process serial binary data and save it as a CSV file/figure")
parser.add_argument('-i', type=str, default='log.bin', help='Input file name (binary)')
parser.add_argument('-o', type=str, default='out.csv', help='Output file name (CSV)')
parser.add_argument('-f', type=str, help='Optional figure file to save the plot (e.g., figure.png)', default=None)
parser.add_argument('-n', type=int, default=5)
parser.add_argument('-l', type=str)
args = parser.parse_args()

input_file = args.i
output_file = args.o
output_image = args.f
n = args.n

if args.l is not None:
    labels = str.split(args.l, ',')
else:
    labels = ['Column ' + str(i+1) for i in range(n)]

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

# Save to csv
print('Writing to ' + output_file + '...')
np.savetxt(output_file, data, fmt='%.10e', delimiter=',')

if output_image is not None:
    print('Saving figure to ' + output_image + '...')
    # Plot each column with a different color
    for i in range(data.shape[1]):
        plt.plot(data[:, i], label=labels[i])

    # Adding labels and title
    plt.xlabel('Index')
    plt.ylabel('Value')
    plt.title('UHA Log Data')
    plt.legend()

    # Show the plot
    plt.savefig(output_image)

