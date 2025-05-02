import csv
from matplotlib import pyplot as plt
import numpy as np

dataset = '5.3.10_Brothers'
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
        altitude = float(row[1]) + launch_altitude
        velocity = float(row[2])

        temperature = float(row[49]) + 273.15     # Convert to [K]
        pressure = float(row[50]) * 100     # Convert to [Pa]

        density = (pressure / (287.058 * temperature))

        # Stop at apogee
        if (velocity < 0 and time > 0.5): break

        table[0].append(altitude)
        table[1].append(density)


# Polyfit
constants = np.polyfit(table[0], table[1], 3)
p = np.poly1d(constants)

# Output
print('\n--- Polynomial Constants ---\n')

i = 1
for const in constants:
    print(f'    {const},   // P{i}')
    i += 1

# print(f'\nPolynomial lower bound: {table[0][0]} m, at {p(table[0][0])} m/s^2')
# print(f'Polynomial upper bound: {table[0][-1]} m, at {p(table[0][-1])} m/s^2')

                      
# Graph velocity table
fig, axes = plt.subplots(1, 1)

axes.plot(table[0], table[1], label='OpenRocket simulation')
axes.plot(table[0], p(table[0]), label='Polynomial fit')

def old_curve(h):
    return (-6.85185e-14 * (h**3)) + (4.30675e-09 * (h**2)) + (-0.0001176 * h) + 1.22499

print(old_curve(3048))

x = np.linspace(0, table[0][-1], 1000)
axes.plot(x, old_curve(x), 'b', label='Old density curve')

axes.set_xlabel('Altitude (m)')
axes.set_ylabel('Air density (kg/m^3)')
axes.grid(True)
axes.legend()

plt.show()