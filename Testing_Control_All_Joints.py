import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider
import serial
import time


def initialize_serial(port, baudrate=9600, timeout=1):
    try:
        ser = serial.Serial(port, baudrate, timeout=timeout)
        return ser
    except serial.SerialException as e:
        print(f"Error: Could not open port {port}. {e}")
        return None


# Initialize serial communication with Arduino
ser = initialize_serial('COM6')

if ser is None:
    print("Exiting program.")
    exit(1)


def forward_kinematics(roll_1, pitch_1, pitch_2, pitch_3):
    L1 = 3  # Length of link 1
    L2 = 3  # Length of link 2
    B = 4.5  # Base height
    t = 1  # tool length

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

    return x1, y1, z1, x2, y2, z2, x3, y3, z3


path = []
last_sent_time = 0
last_sent_angles = (0, 0)


def send_angles_to_arduino(roll, pitch1, pitch2, pitch3):
    global last_sent_time, last_sent_angles
    current_time = time.time()

    # Send angles if 50ms have passed since the last transmission or if angles have changed significantly
    if (current_time - last_sent_time > 0.05) or (
            abs(roll - last_sent_angles[0]) > 1 or
            abs(pitch1 - last_sent_angles[1]) > 1 or
            abs(pitch2 - last_sent_angles[2]) > 1 or
            abs(pitch3 - last_sent_angles[3]) > 1):
        if ser:
            angles_str = f"{roll:.2f},{180 - pitch1:.2f},{pitch2:.2f},{180-pitch3:.2f}\n"
            ser.write(angles_str.encode('utf-8'))
            last_sent_time = current_time
            last_sent_angles = (roll, pitch1, pitch2, pitch3)


def update(val):
    roll_1 = np.radians(sli_roll.val)
    pitch_1 = np.radians(sli_pitch1.val)
    pitch_2 = np.radians(sli_pitch2.val)
    pitch_3 = np.radians(sli_pitch3.val)

    # Send all angle values to Arduino
    send_angles_to_arduino(sli_roll.val, sli_pitch1.val, sli_pitch2.val, sli_pitch3.val)

    x1, y1, z1, x2, y2, z2, x3, y3, z3 = forward_kinematics(roll_1, pitch_1, pitch_2, pitch_3)
    update_plot(roll_1, pitch_1, pitch_2, pitch_3, x1, y1, z1, x2, y2, z2, x3, y3, z3)


def update_plot(roll_1, pitch_1, pitch_2, pitch_3, x1, y1, z1, x2, y2, z2, x3, y3, z3):
    B = 4.5  # Base height

    ax.clear()

    # Plot Base
    ax.plot([0, 0], [0, 0], [0, B], color='g', linewidth=3)

    # Plot link 1
    ax.plot([0, x1], [0, y1], [B, z1], color='b', linewidth=3)
    ax.text(x1, y1, z1, f'Roll: {np.degrees(roll_1):.2f}°\nPitch1: {np.degrees(pitch_1):.2f}°', color='b')

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
ax_pitch2 = plt.axes([0.25, 0.09, 0.65, 0.03])
ax_pitch3 = plt.axes([0.25, 0.13, 0.65, 0.03])

sli_roll = Slider(ax_roll, 'Roll', 0, 180, valinit=0)
sli_pitch1 = Slider(ax_pitch1, 'Pitch1', 0, 180, valinit=0)
sli_pitch2 = Slider(ax_pitch2, 'Pitch2', 0, 180, valinit=90)
sli_pitch3 = Slider(ax_pitch3, 'Pitch3', 0, 180, valinit=0)

sli_roll.on_changed(update)
sli_pitch1.on_changed(update)
sli_pitch2.on_changed(update)
sli_pitch3.on_changed(update)

# Initial plot
initial_roll = 0
initial_pitch1 = 0
initial_pitch2 = np.radians(90)
initial_pitch3 = 0
x1, y1, z1, x2, y2, z2, x, y, z = forward_kinematics(initial_roll, initial_pitch1, initial_pitch2, initial_pitch3)
path.append([x, y, z])
update_plot(initial_roll, initial_pitch1, initial_pitch2, initial_pitch3, x1, y1, z1, x2, y2, z2, x, y, z)

plt.show()

# Close the serial connection when the program ends
if ser:
    ser.close()