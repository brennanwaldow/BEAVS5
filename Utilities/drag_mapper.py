import csv
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import axes3d, art3d
from matplotlib.patches import Circle, Ellipse
import numpy as np

dataset = '5.3.10_Brothers'

# Mach [M], drag coefficient [Cd]
drag_table = [[], []]

apogee_transition = 0.1 # [Mach]

# Output from Ansys Fluent CFD
# Deflection [N], (Velocity [m/s], Total Drag [N])
cfd_values = [
    [0, [
        [100, 1000],
        [250, 3000],
    ]],
    [0.5, [
        [100, 1000],
    ]],
    [1, [
        [100, 1000],
    ]]
]

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
        if (thrust == 0 and mach > apogee_transition and mach < 1.2):
            drag_table[0].append(mach)
            drag_table[1].append(drag_coeff)


drag_table[0].reverse()
drag_table[1].reverse()


x = np.linspace(drag_table[0][-1], drag_table[0][0], 100)
y = np.linspace(0, 1, 100)

cd_rocket = []
for mach in x:
    cd_rocket.append(np.interp(mach, drag_table[0], drag_table[1]))

# Mach to m/s
def mach_to_ms(mach):
    return mach * 343.2   # Sea level

# m/s to Mach
def ms_to_mach(velocity):
    return velocity / 343.2   # Sea level

# Drag function
def get_BEAVS_drag(mach, deflection):
    A_ref = 0.017424685
    A_beavs = 0.0052434475776 * deflection
    cd_beavs = 4.8 * np.sqrt(A_beavs / A_ref)

    cd = cd_rocket + (cd_beavs * (A_beavs / A_ref))
    velocity = mach_to_ms(mach)
    Fd = (0.5 * 1.225 * (velocity ** 2) * cd * A_ref)

    return Fd

# i lifted this from stackexchange yippee
def plot_point(ax, x, y, z, fc = None, ec = None, radius = 0.025):
    xy_len, z_len = ax.get_figure().get_size_inches()
    axis_length = [x[1] - x[0] for x in [ax.get_xbound(), ax.get_ybound(), ax.get_zbound()]]
    axis_rotation =  {'z': ((x, y, z), axis_length[1]/axis_length[0]),
                        'y': ((x, z, y), axis_length[2]/axis_length[0]*xy_len/z_len),
                        'x': ((y, z, x), axis_length[2]/axis_length[1]*xy_len/z_len)}
    for a, ((x0, y0, z0), ratio) in axis_rotation.items():
        p = Ellipse((x0, y0), width = radius, height = radius*ratio, fc=fc, ec=ec)
        p.set_color('crimson')
        ax.add_patch(p)
        art3d.pathpatch_2d_to_3d(p, z=z0, zdir=a)


fig = plt.figure()
ax = fig.add_subplot(projection='3d')

X, Y = np.meshgrid(x, y)
ax.plot_surface(X, Y, get_BEAVS_drag(X, Y), cmap = 'plasma')

for deflection_set in cfd_values:
    deflection = deflection_set[0]
    drag_pairs = deflection_set[1]

    for drag_pair in drag_pairs:
        velocity = drag_pair[0]
        drag = drag_pair[1]

        mach = ms_to_mach(velocity)

        plot_point(ax, mach, deflection, drag)

ax.set_xlabel('Mach')
ax.set_ylabel('Deflection')
ax.set_zlabel('Drag Force (N)')

plt.show()
