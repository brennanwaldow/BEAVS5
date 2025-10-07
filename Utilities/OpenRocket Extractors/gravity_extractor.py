import csv
from matplotlib import pyplot as plt
import numpy as np

dataset = 'subscale-L1000W'
launch_altitude = 1380
# Openrocket sims are in AGL - set this to launch altitude used (Brothers = 1,380m)

# Altitude [m], gravitational acceleration [m/s^2]
table = [[], []]

## Reader designed for OpenRocket simulation CSV export, NOT recorded flight telemetry
# Use highspeed dataset to get a higher range of polynomial validity than expected in case of overshoot
with open('Utilities/Data/' + dataset + '_HighSpeed_DataSet.csv', newline='') as csvfile:
    reader = csv.reader(csvfile)
    for row in reader:
        time = row[0]
        # Break first row
        if (time == '# Time (s)'): continue

        time = float(time)
        altitude = float(row[2])
        velocity = float(row[3])

        gravity = float(row[25])

        # Stop at apogee
        if (velocity < 0 and time > 0.5): break

        table[0].append(altitude)
        table[1].append(gravity)


# Polyfit
constants = np.polyfit(table[0], table[1], 5)
p = np.poly1d(constants)

# Output
print('\n--- Polynomial Constants ---\n')

i = 1
for const in constants:
    print(f'    {const},   // P{i}')
    i += 1

print(f'\nPolynomial lower bound: {table[0][0]} m, at {p(table[0][0])} m/s^2')
print(f'Polynomial upper bound: {table[0][-1]} m, at {p(table[0][-1])} m/s^2')

print('\n')

# Graph gravity table
fig, axes = plt.subplots(1, 1)

axes.plot(table[0], table[1], label='OpenRocket simulation')
axes.plot(table[0], p(table[0]), label='Polynomial fit')

axes.set_xlabel('Altitude (m)')
axes.set_ylabel('g (m/s^2)')
axes.grid(True)
axes.legend()

plt.show()