import csv
from matplotlib import pyplot as plt
import numpy as np

dataset = '5.3.2_Modified'
target_apogee = 3048 # [m]

# Altitude [m], velocity [m/s]
table = [[], []]

## Reader designed for OpenRocket simulation CSV export, NOT recorded flight telemetry
with open('Utilities/Data/' + dataset + '_DataSet.csv', newline='') as csvfile:
    reader = csv.reader(csvfile)
    for row in reader:
        time = row[0]
        # Break first row
        if (time == '# Time (s)'): continue

        time = float(time)
        altitude = float(row[1])
        velocity = float(row[2])
        acceleration = float(row[3])
        thrust = float(row[28])

        # Stop at apogee
        if (velocity < 0 and time > 0.5): break

        # Wait until motor cutout to build table
        if (thrust == 0):
            table[0].append(altitude)
            table[1].append(velocity)

# Readjust altitude to finely intercept target
apogee = table[0][-1]
print(f'\nRaw apogee: {round(apogee, 4)} m')
difference = target_apogee - apogee
for i in range(len(table[0])):
    table[0][i] = table[0][i] + difference
apogee = table[0][-1]
print(f'Adjusted apogee: {round(apogee, 2)} m')

# Polyfit
constants = np.polyfit(table[0], table[1], 15)
p = np.poly1d(constants)

# Output
print('\n--- Polynomial Constants ---\n')

i = 1
for const in constants:
    print(f'P{i}: {const}')
    i += 1

print('')
print(f'Polynomial deviation at apogee: {round(p(table[0][-1]), 3)} m/s\n')


# Graph velocity table
fig, axes = plt.subplots(1, 1)

axes.plot(table[0], table[1], label='OpenRocket simulation')
axes.plot(table[0], p(table[0]), label='Polynomial fit')

axes.set_xlabel('Altitude (m)')
axes.set_ylabel('Velocity (m/s)')
axes.grid(True)
axes.legend()

plt.show()