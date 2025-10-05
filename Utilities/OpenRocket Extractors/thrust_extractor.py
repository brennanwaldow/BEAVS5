import csv
from matplotlib import pyplot as plt
import numpy as np

dataset = 'subscale-L1000W'

# Time [s], thrust [N]
thrust_table = [[], []]

## Reader designed for OpenRocket simulation CSV export, NOT recorded flight telemetry
with open('Utilities/Data/' + dataset + '_DataSet.csv', newline='') as csvfile:
    reader = csv.reader(csvfile)
    for row in reader:
        time = row[0]
        # Break first row
        if (time == '# Time (s)'): continue

        time = float(time) * 1000   # Convert [s] to [ms]
        altitude = float(row[1])
        thrust = float(row[29])

        thrust_table[0].append(time)
        thrust_table[1].append(thrust)

        # Stop at motor cutout after acquiring last datapoint
        if (thrust == 0 and altitude > 0): break


# Output for paste into Arduino

print('\n')
print(f'double timeValues[{len(thrust_table[0])}] = ', end='')
print('{ ', end='')

for time in thrust_table[0]:
    print(f'{time}, ', end='')
print('};')

print('\n\n')

print(f'double thrustValues[{len(thrust_table[1])}] = ', end='')
print('{ ', end='')

for thrust in thrust_table[1]:
    print(f'{thrust}, ', end='')
print('};')

print(f'\n\n\nLower bound: {thrust_table[0][0]} ms')
print(f'Upper bound: {thrust_table[0][-1]} ms')
print('\n')


# Graph thrust table
fig, axes = plt.subplots(1, 1)

axes.plot(thrust_table[0], thrust_table[1], label='OpenRocket simulation')

axes.set_xlabel('Time (ms)')
axes.set_ylabel('Thrust (N)')
axes.grid(True)
axes.legend()

plt.show()