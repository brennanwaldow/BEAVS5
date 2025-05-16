index = input("Enter log index number: ")

import csv
from matplotlib import pyplot as plt
import numpy as np

graph = [[], [], [], [], [], []]
current_phase = 0
phase_changes = []

target_agl = 3048
launch_elevation = 1380

with open('Utilities/Data/Flight Data/data_' + index + '.csv', newline='') as csvfile:
    reader = csv.reader(csvfile)
    for row in reader:
        time = row[0]
        # Break first row
        if (time == '# Time (s)'): continue

        time = float(time)
        altitude = float(row[1])
        velocity = float(row[3])
        acceleration = float(row[4])

        commanded_angle = float(row[5])
        flight_phase = float(row[6])

        graph[0].append(time)
        graph[1].append(altitude)
        graph[2].append(velocity)
        graph[3].append(acceleration)

        graph[4].append(commanded_angle)
        graph[5].append(flight_phase)
        
        if (flight_phase != current_phase):
            phase_changes.append(time)
            current_phase = flight_phase


# Graph table
fig, axes = plt.subplots(1, 1)

ax2 = axes.twinx()

axes.plot(graph[0], graph[1], label='Altitude', color='r')
axes.plot(graph[0], graph[2], label='Velocity', color='coral')
axes.plot(graph[0], graph[3], label='Acceleration', color='goldenrod')

ax2.plot(graph[0], graph[4], label='Commanded Angle', color='darkmagenta')
# axes.plot(graph[0], graph[5], label='Flight Phase', color='lightsalmon')

axes.axhline(y=target_agl + launch_elevation, color='black', ls='--')

for phase_change in phase_changes:
    plt.axvline(x=phase_change, color='black', ls='--')

axes.set_xlabel('Time (s)')
axes.set_ylabel(' ')
axes.grid(True)
axes.legend()

ax2.set_ylabel("Commanded Angle (Degrees)", color='darkmagenta')
ax2.tick_params(labelcolor='darkmagenta')
ax2.set_ylim(0, 180)

plt.show()