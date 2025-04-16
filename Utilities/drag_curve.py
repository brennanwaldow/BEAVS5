import csv
from matplotlib import pyplot as plt
import numpy as np

dataset = '5.3.2'

# Mach [M], drag coefficient [Cd]
drag_table_apogee = [[], []]
drag_table_subsonic = [[], []]
drag_table_transonic = [[], []]
base_table = [[], []]

apogee_transition = 0.1 # [Mach]
transonic_transition = 1 # [Mach]

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
        if (thrust == 0 and mach > 0 and mach < apogee_transition):
            drag_table_apogee[0].append(mach)
            drag_table_apogee[1].append(drag_coeff)
        if (thrust == 0 and mach > apogee_transition and mach < transonic_transition):
            drag_table_subsonic[0].append(mach)
            drag_table_subsonic[1].append(drag_coeff)
        if (thrust == 0 and mach >= transonic_transition and mach < 1.2):
            drag_table_transonic[0].append(mach)
            drag_table_transonic[1].append(drag_coeff)

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
constants_apogee = np.polyfit(drag_table_apogee[0], drag_table_apogee[1], 15)
p_apogee = np.poly1d(constants_apogee)

constants_subsonic = np.polyfit(drag_table_subsonic[0], drag_table_subsonic[1], 15)
p_subsonic = np.poly1d(constants_subsonic)

constants_transonic = np.polyfit(drag_table_transonic[0], drag_table_transonic[1], 10)
p_transonic = np.poly1d(constants_transonic)

# Print for easy copy-paste into Arduino IDE

print('\n--- Polynomial Constants ---\n')


print(drag_table_apogee)

print('\nApogee: \n')

i = 1
for const in constants_apogee:
    print(f'      {const},    // P{i}')
    i += 1

print('\nSubsonic: \n')

i = 1
for const in constants_subsonic:
    print(f'      {const},    // P{i}')
    i += 1

print('\nTransonic: \n')

i = 1
for const in constants_transonic:
    print(f'      {const},    // P{i}')
    i += 1

print(f'\nPolynomial upper bound: M {drag_table_transonic[0][0]}, at Cd = {p_transonic(drag_table_transonic[0][0])}')
print(f'Polynomial lower bound (subsonic): M {drag_table_subsonic[0][-1]}, at Cd = {p_subsonic(drag_table_subsonic[0][-1])}')
print(f'Polynomial lower bound (apogee): M {drag_table_apogee[0][-1]}, at Cd = {p_apogee(drag_table_apogee[0][-1])}')


# Graph velocity table
fig, axes = plt.subplots(1, 1)

axes.plot(drag_table_apogee[0], drag_table_apogee[1], label='OpenRocket highspeed simulation')
axes.plot(drag_table_subsonic[0], drag_table_subsonic[1], label='OpenRocket highspeed simulation')
axes.plot(drag_table_transonic[0], drag_table_transonic[1], label='OpenRocket highspeed simulation')
axes.plot(base_table[0], base_table[1], label='OpenRocket simulation')
x = np.linspace(drag_table_apogee[0][-1], drag_table_apogee[0][0] + 0, 1000)
axes.plot(x, p_apogee(x), 'r', label='Polynomial fit')
x = np.linspace(0.1, drag_table_subsonic[0][0] + 0, 1000)
axes.plot(x, p_subsonic(x), 'r', label='Polynomial fit')
x = np.linspace(drag_table_subsonic[0][0], drag_table_transonic[0][0] + 0, 1000)
axes.plot(x, p_transonic(x), 'r', label='Polynomial fit')

plt.axvline(x=apogee_transition, color='black', ls='--')
plt.axvline(x=transonic_transition, color='black', ls='--')

def old_drag_curve(m):
    return (0.0936073 * (m**3)) + (-0.0399526 * (m**2)) + (0.0455436 * m) + 0.582895;
x = np.linspace(0, drag_table_transonic[0][0] + 0, 1000)
axes.plot(x, old_drag_curve(x), 'b', label='Old drag curve')

axes.set_xlabel('Mach (M)')
axes.set_ylabel('Rocket drag coefficient, Cd')
axes.grid(True)
axes.legend()

plt.show()