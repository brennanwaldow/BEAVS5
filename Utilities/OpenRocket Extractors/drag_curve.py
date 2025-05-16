import csv
from matplotlib import pyplot as plt
import numpy as np

dataset = '5.3.10_Brothers'

# Mach [M], drag coefficient [Cd]
drag_table = [[], []]
drag_table_apogee = [[], []]
base_table = [[], []]

apogee_transition = 0.1 # [Mach]
max_mach = 1.2 # [Mach]

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
        if (thrust == 0 and mach < apogee_transition):
            drag_table_apogee[0].append(mach)
            drag_table_apogee[1].append(drag_coeff)
        elif (thrust == 0 and mach < max_mach):
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

# Reverse tables (coast phase is in descending mach)
drag_table[0].reverse()
drag_table[1].reverse()
drag_table_apogee[0].reverse()
drag_table_apogee[1].reverse()

# Print for easy copy-paste into Arduino IDE

print(f'double machValues[{len(drag_table[0])}] = ', end='')
print('{ ', end='')

for mach in drag_table[0]:
    print(f'{mach}, ', end='')
print('};')

print('\n\n')

print(f'double cdValues[{len(drag_table[1])}] = ', end='')
print('{ ', end='')

for cd in drag_table[1]:
    print(f'{cd}, ', end='')
print('};')

print(f'\n\n\nLower bound (apogee): Cd = {drag_table_apogee[1][0]} at M{drag_table_apogee[0][0]}')
print(f'Lower bound (coast): Cd = {drag_table[1][0]} at M{drag_table[0][0]}')
print(f'Upper bound: Cd = {drag_table[1][-1]} at M{drag_table[0][-1]}')

# print(f'\nPolynomial upper bound: M {drag_table_transonic[0][0]}, at Cd = {p_transonic(drag_table_transonic[0][0])}')
# print(f'Polynomial lower bound (subsonic): M {drag_table_subsonic[0][-1]}, at Cd = {p_subsonic(drag_table_subsonic[0][-1])}')
# print(f'Polynomial lower bound (apogee): M {drag_table_apogee[0][-1]}, at Cd = {p_apogee(drag_table_apogee[0][-1])}')


# Graph drag table
fig, axes = plt.subplots(1, 1)

axes.plot(drag_table[0], drag_table[1], label='OpenRocket highspeed simulation')
axes.plot(drag_table_apogee[0], drag_table_apogee[1], label='OpenRocket highspeed simulation')
axes.plot(base_table[0], base_table[1], label='OpenRocket simulation')

plt.axvline(x=apogee_transition, color='black', ls='--')
plt.axvline(x=max_mach, color='black', ls='--')

def old_drag_curve(m):
    return (0.0936073 * (m**3)) + (-0.0399526 * (m**2)) + (0.0455436 * m) + 0.582895;
x = np.linspace(0, drag_table[0][0], 1000)
axes.plot(x, old_drag_curve(x), 'b', label='Old drag curve')

axes.set_xlabel('Mach (M)')
axes.set_ylabel('Rocket drag coefficient, Cd')
axes.grid(True)
axes.legend()

plt.show()