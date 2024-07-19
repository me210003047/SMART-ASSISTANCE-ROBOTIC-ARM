import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
from scipy.interpolate import CubicSpline


def inverse_kinematics(end_effector):
    L1 = 3  # Length of link 1
    L2 = 3  # Length of link 2
    B = 4.5  # Base height
    t = 1  # tool length

    x, y, z = end_effector
    z_off = z - B
    R = np.sqrt(x ** 2 + y ** 2 + z_off ** 2)
    r = np.sqrt(x ** 2 + y ** 2)

    roll_1 = np.arctan2(y, x)
    cos_1 = (L1 ** 2 + x ** 2 + y ** 2 + (z_off + t) ** 2 - L2 ** 2) / (
            2 * L1 * np.sqrt(x ** 2 + y ** 2 + (z_off + t) ** 2))
    pitch_1 = np.arccos(cos_1) + np.arctan2((z_off + t), r)
    cos_2 = (L1 ** 2 + L2 ** 2 - (x ** 2 + y ** 2 + (z_off + t) ** 2)) / (2 * L1 * L2)
    pitch_2 = np.arccos(cos_2)
    cos_3_1 = (L2 ** 2 + x ** 2 + y ** 2 + (z_off + t) ** 2 - L1 ** 2) / (
            2 * L2 * np.sqrt(x ** 2 + y ** 2 + (z_off + t) ** 2))
    cos_3_2 = (-(x ** 2 + y ** 2 + z_off ** 2) + x ** 2 + y ** 2 + (z_off + t) ** 2 + t ** 2) / (
            2 * t * np.sqrt(x ** 2 + y ** 2 + (z_off + t) ** 2))
    pitch_3 = np.arccos(cos_3_1) + np.arccos(cos_3_2)
    return roll_1, pitch_1, pitch_2, pitch_3


def calculate_joint_positions(roll_1, pitch_1, pitch_2, pitch_3):
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

    return (x1, y1, z1), (x2, y2, z2), (x3, y3, z3)


def draw_path():
    fig, ax = plt.subplots()
    ax.set_xlim(-8, 8)
    ax.set_ylim(-8, 8)
    ax.set_aspect('equal')
    ax.grid(True)

    path = []
    line, = ax.plot([], [], 'r-')
    points, = ax.plot([], [], 'bo')  # To show clicked points

    def onclick(event):
        if event.inaxes != ax: return
        path.append((event.xdata, event.ydata))
        x_points = [p[0] for p in path]
        y_points = [p[1] for p in path]
        points.set_data(x_points, y_points)

        if len(path) > 2:
            t = np.arange(len(path))
            cs_x = CubicSpline(t, x_points)
            cs_y = CubicSpline(t, y_points)
            t_smooth = np.linspace(0, len(path) - 1, 100)
            x_smooth = cs_x(t_smooth)
            y_smooth = cs_y(t_smooth)
            line.set_data(x_smooth, y_smooth)
        elif len(path) == 2:
            line.set_data(x_points, y_points)

        fig.canvas.draw()

    def on_finish(event):
        plt.close(fig)

    fig.canvas.mpl_connect('button_press_event', onclick)
    finish_button = Button(plt.axes([0.81, 0.05, 0.1, 0.075]), 'Finish')
    finish_button.on_clicked(on_finish)

    plt.show()

    if len(path) > 2:
        t = np.arange(len(path))
        cs_x = CubicSpline(t, [p[0] for p in path])
        cs_y = CubicSpline(t, [p[1] for p in path])
        t_smooth = np.linspace(0, len(path) - 1, 200)
        x_smooth = cs_x(t_smooth)
        y_smooth = cs_y(t_smooth)
        return list(zip(x_smooth, y_smooth))
    else:
        return path


def animate(frame):
    ax.clear()

    progress = frame / (frames - 1)
    path_index = int(progress * (len(path) - 1))
    current_x, current_y = path[path_index]
    current_z = 3  # Keep z constant at height

    roll, pitch1, pitch2, pitch3 = inverse_kinematics([current_x, current_y, current_z])

    (x1, y1, z1), (x2, y2, z2), (x3, y3, z3) = calculate_joint_positions(
        roll, pitch1, pitch2, pitch3)
    dis1 = (x1 - 0) ** 2 + (y1 - 0) ** 2 + (z1 - B) ** 2
    dis2 = (x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2
    print(dis1)
    print(dis2)

    # Plot Base
    ax.plot([0, 0], [0, 0], [0, B], color='g', linewidth=3)

    # Plot link 1
    ax.plot([0, x1], [0, y1], [B, z1], color='b', linewidth=3)
    ax.text(x1, y1, z1, f'Roll: {np.degrees(roll):.2f}°\nPitch: {np.degrees(pitch1):.2f}°', color='b')

    # Plot link 2
    ax.plot([x1, x2], [y1, y2], [z1, z2], color='r', linewidth=3)
    ax.text(x2, y2, z2, f'Pitch: {np.degrees(pitch2):.2f}°', color='r')

    # Plot tool
    ax.plot([x2, x3], [y2, y3], [z2, z3], color='y', linewidth=3)
    ax.text(x3, y3, z3, f'Pitch: {np.degrees(pitch3):.2f}°', color='y')

    # Plot the entire path
    path_array = np.array(path)
    ax.plot(path_array[:, 0], path_array[:, 1], [current_z] * len(path), color='m', linewidth=2, alpha=0.5)

    # Set plot limits and labels
    ax.set_xlim([-8, 8])
    ax.set_ylim([-8, 8])
    ax.set_zlim([0, 8])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')


# Main script
print("Click on the plot to draw the path. Click 'Finish' when done.")
path = draw_path()

# Set up the figure and 3D axis
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Animation settings
frames = 200  # Increase the number of frames for smoother animation
B = 4.5  # Base height

# Create the animation
anim = FuncAnimation(fig, animate, frames=frames, interval=50, repeat=False)

plt.show()