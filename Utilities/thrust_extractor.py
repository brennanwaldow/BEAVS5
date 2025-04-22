import csv
from matplotlib import pyplot as plt
import numpy as np

dataset = '5.3.10'

# Time [s], thrust [N]
thrust_table = [[[], []], [[], []], [[], []], [[], []]]

breakpoints = [37, 3247, 3845]

## Reader designed for OpenRocket simulation CSV export, NOT recorded flight telemetry
with open('Utilities/Data/' + dataset + '_DataSet.csv', newline='') as csvfile:
    reader = csv.reader(csvfile)
    for row in reader:
        time = row[0]
        # Break first row
        if (time == '# Time (s)'): continue

        time = float(time) * 1000   # Convert [s] to [ms]
        thrust = float(row[28])

        if (time < breakpoints[0]):
            thrust_table[0][0].append(time)
            thrust_table[0][1].append(thrust)
        elif (time < breakpoints[1]):
            thrust_table[1][0].append(time)
            thrust_table[1][1].append(thrust)
        elif (time < breakpoints[2]):
            thrust_table[2][0].append(time)
            thrust_table[2][1].append(thrust)
        else:
            thrust_table[3][0].append(time)
            thrust_table[3][1].append(thrust)
        
        # Stop at motor cutout
        if (thrust == 0): break

# Polyfit
constants1 = np.polyfit(thrust_table[0][0], thrust_table[0][1], 10)
constants2 = np.polyfit(thrust_table[1][0], thrust_table[1][1], 20)
constants3 = np.polyfit(thrust_table[2][0], thrust_table[2][1], 10)
constants4 = np.polyfit(thrust_table[3][0], thrust_table[3][1], 20)
p1 = np.poly1d(constants1)
p2 = np.poly1d(constants2)
p3 = np.poly1d(constants3)
p4 = np.poly1d(constants4)

# Output
print('\n--- Polynomial Constants ---\n')


print('\nPhase 1: \n')

i = 1
for const in constants1:
    print(f'      {const},   // P{i}')
    i += 1


print('\nPhase 2: \n')

i = 1
for const in constants2:
    print(f'      {const},   // P{i}')
    i += 1

print('\nPhase 3: \n')

i = 1
for const in constants3:
    print(f'      {const},   // P{i}')
    i += 1

    

print('\nPhase 4: \n')

i = 1
for const in constants4:
    print(f'      {const},   // P{i}')
    i += 1

print(f'\nPolynomial lower bound: {thrust_table[0][0][0]} ms')
print(f'Polynomial upper bound: {thrust_table[3][0][-1]} ms')
                      
# Graph velocity table
fig, axes = plt.subplots(1, 1)

axes.plot(thrust_table[0][0], thrust_table[0][1], label='OpenRocket simulation')
axes.plot(thrust_table[1][0], thrust_table[1][1], label='OpenRocket simulation')
axes.plot(thrust_table[2][0], thrust_table[2][1], label='OpenRocket simulation')
axes.plot(thrust_table[3][0], thrust_table[3][1], label='OpenRocket simulation')

axes.plot(thrust_table[0][0], p1(thrust_table[0][0]), label='Polynomial fit')
axes.plot(thrust_table[1][0], p2(thrust_table[1][0]), label='Polynomial fit')
axes.plot(thrust_table[2][0], p3(thrust_table[2][0]), label='Polynomial fit')
axes.plot(thrust_table[3][0], p4(thrust_table[3][0]), label='Polynomial fit')

plt.axvline(x=breakpoints[0], color='black', ls='--')
plt.axvline(x=breakpoints[1], color='black', ls='--')
plt.axvline(x=breakpoints[2], color='black', ls='--')

axes.set_xlabel('Time (ms)')
axes.set_ylabel('Thrust (N)')
axes.grid(True)
axes.legend()

plt.show()