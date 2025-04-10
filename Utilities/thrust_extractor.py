import csv
from matplotlib import pyplot as plt
import numpy as np

dataset = '5.3.2'

# Time [s], thrust [N]
table = [[], []]

## Reader designed for OpenRocket simulation CSV export, NOT recorded flight telemetry
with open('Utilities/Data/' + dataset + '_DataSet.csv', newline='') as csvfile:
    reader = csv.reader(csvfile)
    for row in reader:
        time = row[0]
        # Break first row
        if (time == '# Time (s)'): continue

        time = float(time)
        thrust = float(row[28])

        table[0].append(time * 1000) # Convert [s] to [ms]
        table[1].append(thrust)
        
        # Stop at motor cutout
        if (thrust == 0): break

# Polyfit
constants = np.polyfit(table[0], table[1], 20)
p = np.poly1d(constants)

# Output
print('\n--- Polynomial Constants ---\n')

i = 1
for const in constants:
    print(f'P{i}: {const}')
    i += 1

# Print for easy copy-paste into Arduino IDE
print('\nArray output:\n')

i = 1
for const in constants:
    print(f'    {const},')
    i += 1

print(f'\nPolynomial lower bound: {table[0][0]} s')
print(f'Polynomial upper bound: {table[0][-1]} s')
                      
# Graph velocity table
fig, axes = plt.subplots(1, 1)

axes.plot(table[0], table[1], label='OpenRocket simulation')
axes.plot(table[0], p(table[0]), label='Polynomial fit')

axes.set_xlabel('Time (ms)')
axes.set_ylabel('Thrust (N)')
axes.grid(True)
axes.legend()

plt.show()