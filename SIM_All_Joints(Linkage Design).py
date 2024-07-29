import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider

#Dimensional specifications
L1 = 3  # Length of link 1
L2 = 3  # Length of link 2
B = 3  # Base height
t = 1  # tool length
c = 1  # length of cranks

def forward_kinematics(roll_1, pitch_1, pitch_2_link, pitch_3):
    L1 = 3  # Length of link 1
    L2 = 3  # Length of link 2
    B = 3  # Base height
    t = 1  # tool length
    c = 1  # length of cranks

    pitch_2= pitch_2_link-pitch_1

    z1 = L1 * np.sin(pitch_1) + B
    pL1 = L1 * np.cos(pitch_1)
    x1 = pL1 * np.cos(roll_1)
    y1 = pL1 * np.sin(roll_1)

    z2 = z1 - L2 * np.sin(pitch_1 + pitch_2)
    pL2 = -L2 * np.cos(pitch_1 + pitch_2)
    x2 = x1 + pL2 * np.cos(roll_1)
    y2 = y1 + pL2 * np.sin(roll_1)

    z3 = z2 + t * np.sin(pitch_1 + pitch_2 + pitch_3)
    pL3 = t * np.cos(pitch_1 + pitch_2 + pitch_3)
    x3 = x2 + pL3 * np.cos(roll_1)
    y3 = y2 + pL3 * np.sin(roll_1)

    z_1= B + c*np.sin(pitch_1+pitch_2)
    plc=c*np.cos(pitch_1+pitch_2)
    x_1=plc*np.cos(roll_1)
    y_1=plc*np.sin(pitch_1+pitch_2)

    z_2 = z1 + c * np.sin(pitch_1 + pitch_2)
    plc = c * np.cos(pitch_1 + pitch_2)
    x_2 = x1 + plc * np.cos(roll_1)
    y_2 = y1 + plc * np.sin(pitch_1 + pitch_2)

    return x1, y1, z1, x2, y2, z2, x3, y3, z3,x_1,y_1,z_1,x_2,y_2,z_2


path = []

def update(val):
    roll_1 = np.radians(sli_roll.val)
    pitch_1 = np.radians(sli_pitch1.val)
    pitch_2_link = np.radians(sli_pitch2_link.val)
    pitch_3 = np.radians(sli_pitch3.val)

    x1, y1, z1, x2, y2, z2, x3, y3, z3,x_1,y_1,z_1,x_2,y_2,z_2 = forward_kinematics(roll_1, pitch_1, pitch_2_link, pitch_3)
    update_plot(roll_1, pitch_1, pitch_2_link, pitch_3, x1, y1, z1, x2, y2, z2, x3, y3, z3,x_1,y_1,z_1,x_2,y_2,z_2)

def update_plot(roll_1, pitch_1,pitch_2_link, pitch_3, x1, y1, z1, x2, y2, z2, x3, y3, z3,x_1,y_1,z_1,x_2,y_2,z_2):
    pitch_2=pitch_2_link-pitch_1

    ax.clear()

    # Plot Base
    ax.plot([0, 0], [0, 0], [0, B], color='g', linewidth=3)

    # Plot link 1
    ax.plot([0, x1], [0, y1], [B, z1], color='b', linewidth=3)
    ax.text(x1, y1, z1, f'Roll: {np.degrees(roll_1):.2f}°\nPitch1: {np.degrees(pitch_1):.2f}°', color='b')

    # plot cranks and linkage
    ax.plot([0, x_1], [0, y_1], [B, z_1], color='b', linewidth=3)
    ax.text(x1, y1, z1, f'Roll: {np.degrees(roll_1):.2f}°\nPitch1: {np.degrees(pitch_1):.2f}°', color='b')

    ax.plot([x1, x_2], [y1, y_2], [z1, z_2], color='b', linewidth=3)

    ax.plot([x_1, x_2], [y_1, y_2], [z_1, z_2], color='b', linewidth=3)

    # Plot link 2
    ax.plot([x1, x2], [y1, y2], [z1, z2], color='r', linewidth=3)
    ax.text(x2, y2, z2, f'Pitch2: {np.degrees(pitch_2):.2f}°', color='r')

    # Plot tool
    ax.plot([x2, x3], [y2, y3], [z2, z3], color='y', linewidth=3)
    ax.text(x3, y3, z3, f'Pitch3: {np.degrees(pitch_3):.2f}°', color='y')

    # Plot the path of the end effector
    if len(path) > 1:
        path_array = np.array(path)
        ax.plot(path_array[:, 0], path_array[:, 1], path_array[:, 2], color='m', linewidth=2, alpha=0.5)

    # Set plot limits and labels
    ax.set_xlim([-8, 8])
    ax.set_ylim([-8, 8])
    ax.set_zlim([0, 8])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    fig.canvas.draw_idle()

# Create the main figure and 3D axes
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Create sliders for joint angles
ax_roll = plt.axes([0.25, 0.01, 0.65, 0.03])
ax_pitch1 = plt.axes([0.25, 0.05, 0.65, 0.03])
ax_pitch2_link = plt.axes([0.25, 0.09, 0.65, 0.03])
ax_pitch3 = plt.axes([0.25, 0.13, 0.65, 0.03])

sli_roll = Slider(ax_roll, 'Roll', 0, 180, valinit=0)
sli_pitch1 = Slider(ax_pitch1, 'Pitch1', 0, 180, valinit=45)
sli_pitch2_link = Slider(ax_pitch2_link, 'Pitch2_link', 0, 180, valinit=0)
sli_pitch3 = Slider(ax_pitch3, 'Pitch3', 0, 180, valinit=0)

sli_roll.on_changed(update)
sli_pitch1.on_changed(update)
sli_pitch2_link.on_changed(update)
sli_pitch3.on_changed(update)

# Initial plot
initial_roll = 0
initial_pitch1 = np.radians(45)
initial_pitch2_link = 0
initial_pitch3 = 0
x1, y1, z1,x2, y2, z2, x3, y3, z3,x_1,y_1,z_1,x_2,y_2,z_2 = forward_kinematics(initial_roll, initial_pitch1, initial_pitch2_link, initial_pitch3)
path.append([x3, y3, z3])
update_plot(initial_roll, initial_pitch1, initial_pitch2_link, initial_pitch3,x1, y1, z1,x2, y2, z2, x3, y3, z3,x_1,y_1,z_1,x_2,y_2,z_2)

plt.show()