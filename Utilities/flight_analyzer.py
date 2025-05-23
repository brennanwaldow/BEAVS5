index = input("\nEnter log index number: ")

import csv
from matplotlib import pyplot as plt
import numpy as np

graph = [[], [], [], [], [], [], [], [], [], [], []]
current_phase = 0
phase_changes = []

target_agl = 3048
launch_elevation = 1380

skip = 0
apogee = 0
apogee_agl = 0

# Reader designed for BEAVS5 SD card logging
with open('Utilities/Data/Flight Data/data_' + index + '.csv', newline='') as csvfile:
    reader = csv.reader(csvfile)
    for row in reader:
        # Skip first two data rows to avoid dodgy data as it calibrates
        skip += 1
        if (skip < 4): continue

        # idiot's way of making old logs backwards compatible because i don't want to rerun everything
        row.extend([0, -1, 0, -1])

        time = float(row[0])
        altitude = float(row[1])
        altitude_AGL = float(row[2])
        velocity = float(row[3])
        acceleration = float(row[4])

        commanded_angle = float(row[5])
        flight_phase = float(row[6])

        experienced_drag = float(row[7])
        expected_drag = float(row[8])

        target_velocity = float(row[9])
        velocity_error = float(row[10])

        graph[0].append(time)
        graph[1].append(altitude)
        graph[8].append(altitude_AGL)
        graph[2].append(velocity)
        graph[3].append(acceleration)

        graph[4].append(commanded_angle)
        graph[5].append(flight_phase)

        graph[6].append(experienced_drag)
        graph[7].append(expected_drag)

        graph[9].append(target_velocity)
        graph[10].append(velocity_error)

        if apogee < altitude:
            apogee = altitude
            apogee_agl = altitude_AGL
        
        if (flight_phase != current_phase):
            phase_changes.append(time)
            current_phase = flight_phase

# Basic text outputs

print(f'\n\nApogee: {apogee}m ({apogee_agl}m AGL)')
print(f'Deviation from {target_agl}m target: {round(target_agl - apogee_agl, 3)}m\n\n')


# Graph table
fig, axes = plt.subplots(5, 1)

# ax2 = axes.twinx()

axes[0].plot(graph[0], graph[1], label='Altitude', color='r')
axes[0].plot(graph[0], graph[8], label='Altitude AGL', color='lightseagreen')
axes[1].plot(graph[0], graph[2], label='Velocity (m/s)', color='coral')
axes[1].plot(graph[0], graph[3], label='Acceleration (m/s^2)', color='goldenrod')

axes[2].plot(graph[0], graph[4], label='Commanded Angle', color='darkmagenta')
axes[2].plot(graph[0], graph[5], label='Flight Phase', color='lightsalmon')

axes[3].plot(graph[0], graph[6], label='Experienced Drag', color='firebrick')
axes[3].plot(graph[0], graph[7], label='Expected Drag', color='tomato')

axes[4].plot(graph[0], graph[2], label='Velocity (m/s)', color='coral')
axes[4].plot(graph[0], graph[9], label='Target Velocity', color='dodgerblue')
axes[4].plot(graph[0], graph[10], label='Velocity Error', color='slategray')

if apogee > 500:
    axes[0].axhline(y=target_agl + launch_elevation, color='black', ls='--')
    axes[0].axhline(y=target_agl, color='palegreen', ls='--')

for phase_change in phase_changes:
    axes[0].axvline(x=phase_change, color='black', ls='--')
    axes[1].axvline(x=phase_change, color='black', ls='--')
    axes[2].axvline(x=phase_change, color='black', ls='--')
    axes[3].axvline(x=phase_change, color='black', ls='--')
    axes[4].axvline(x=phase_change, color='black', ls='--')

axes[0].set_xlabel('Time (s)')
axes[0].set_ylabel('Altitude (m)')
axes[0].grid(True)
axes[0].legend()

axes[1].set_xlabel('Time (s)')
axes[1].set_ylabel(' ')
axes[1].grid(True)
axes[1].legend()

axes[2].set_xlabel('Time (s)')
axes[2].set_ylabel('Degrees')
axes[2].grid(True)
axes[2].legend()

axes[3].set_xlabel('Time (s)')
axes[3].set_ylabel('Drag [N]')
axes[3].grid(True)
axes[3].legend()
axes[3].set_ylim(bottom=-100)

axes[4].set_xlabel('Time (s)')
axes[4].set_ylabel('Velocity [m/s]')
axes[4].grid(True)
axes[4].legend()

# ax2.set_ylabel("Commanded Angle (Degrees)", color='darkmagenta')
# ax2.tick_params(labelcolor='darkmagenta')
# ax2.set_ylim(0, 180)

plt.show()