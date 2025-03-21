import csv
from matplotlib import pyplot as plt
import numpy as np

dataset = '5.3.2'

# Mach [M], drag coefficient [Cd]
drag_table = [[], []]
base_table = [[], []]

## Reader designed for OpenRocket simulation CSV export, NOT recorded flight telemetry
with open('Utilities/Data/' + dataset + '_HighSpeed_DataSet.csv', newline='') as csvfile:
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

        mach = float(row[26])
        drag_coeff = float(row[30])

        # Stop at apogee
        if (velocity < 0 and time > 0.5): break

        # Wait until motor cutout to build table
        # Drag unreliable below M0.1: Rocket no longer vertical, drag increases
        if (thrust == 0 and mach > 0.1 and mach < 1.2):
            drag_table[0].append(mach)
            drag_table[1].append(drag_coeff)

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

        mach = float(row[26])
        drag_coeff = float(row[30])

        # Stop at apogee
        if (velocity < 0 and time > 0.5): break

        # Wait until motor cutout to build table
        # Drag unreliable below M0.1: Rocket no longer vertical, drag increases
        if (thrust == 0 and mach > 0.1 and mach < 1.1):
            base_table[0].append(mach)
            base_table[1].append(drag_coeff)


# Polyfit
constants = np.polyfit(drag_table[0], drag_table[1], 20)
p = np.poly1d(constants)

print(drag_table[0])

# Output
print('\n--- Polynomial Constants ---\n')

i = 1
for const in constants:
    print(f'P{i}: {const}')
    i += 1

def old_drag_curve(m):
    return (0.0936073 * (m**3)) + (-0.0399526 * (m**2)) + (0.0455436 * m) + 0.582895;

# Graph velocity table
fig, axes = plt.subplots(1, 1)

axes.plot(drag_table[0], drag_table[1], label='OpenRocket highspeed simulation')
axes.plot(base_table[0], base_table[1], label='OpenRocket simulation')
x = np.linspace(0.1, drag_table[0][0] + 0, 1000)
axes.plot(x, p(x), 'r', label='Polynomial fit')
axes.plot(x, old_drag_curve(x), 'g', label='Old drag curve')

axes.set_xlabel('Mach (M)')
axes.set_ylabel('Rocket drag coefficient, Cd')
axes.grid(True)
axes.legend()

plt.show()