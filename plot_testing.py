import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R


def plot_setup(axis, origin, frame, frame_label, attitude=None):
    o = np.array(origin)
    x_color = 'b'
    y_color = 'b'
    z_color = 'r'

    # Plot each axis as a separate quiver
    axis.quiver(o[0], o[1], o[2], frame[0, 0], frame[1, 0], frame[2, 0], color=x_color, length=0.5)
    axis.text(o[0] + frame[0, 0], o[1] + frame[1, 0], o[2] + frame[2, 0], ' X', color=x_color)

    axis.quiver(o[0], o[1], o[2], frame[0, 1], frame[1, 1], frame[2, 1], color=y_color, length=0.5)
    axis.text(o[0] + frame[0, 1], o[1] + frame[1, 1], o[2] + frame[2, 1], ' Y', color=y_color)

    axis.quiver(o[0], o[1], o[2], frame[0, 2], frame[1, 2], frame[2, 2], color=z_color, length=0.5)
    axis.text(o[0] + frame[0, 2], o[1] + frame[1, 2], o[2] + frame[2, 2], ' Z', color=z_color)

    axis.set_xlabel('x')
    axis.set_ylabel('y')
    axis.set_zlabel('z')
    axis.set_title(frame_label)
    axis.set_xlim(-1, 1)
    axis.set_ylim(-1, 1)
    axis.set_zlim(-1, 1)
    axis.xaxis.set_ticks(np.arange(-1, 1.5, 1))
    axis.yaxis.set_ticks(np.arange(-1, 1.5, 1))
    axis.zaxis.set_ticks(np.arange(-1, 1.5, 1))
    axis.view_init(azim=110, elev=200)

    textstr = f'Y = {attitude[0]}\nP = {attitude[1]}\nZ = {attitude[2]}'
    plt.text(2000, 2000, s=textstr, fontsize=14)
    plt.subplots_adjust(left=0.25)


def maneuver(euler_angle_list, euler_seq, degrees=True, origin=np.array([0, 0, 0])):
    fig = plt.figure(figsize=(9, 16))
    maneuver_count = 1
    rotation_list = []

    for maneuver_angle_set in euler_angle_list:
        rotation = R.from_euler(seq=euler_seq, angles=maneuver_angle_set, degrees=degrees)
        if maneuver_count == 1:
            axis_label = 'Initial Attitude'
            apply_rotation = rotation.as_matrix()
        elif maneuver_count == len(euler_angle_list):
            axis_label = 'Final Attitude'
            apply_rotation = (rotation * rotation_list[-1]).as_matrix()
        else:
            axis_label = 'Maneuver ' + str(maneuver_count)
            apply_rotation = (rotation * rotation_list[-1]).as_matrix()
        axis = fig.add_subplot(len(euler_angle_list), 1, maneuver_count, projection='3d')
        plot_setup(axis, origin, apply_rotation, axis_label, attitude=euler_angle_list[maneuver_count])
        rotation_list.append(rotation)
        maneuver_count += 1

    plt.show()

    for rotation in rotation_list:
        print(rotation.as_euler('zyx'))


angles = [[0, 0, 0], [30, 0, 0], [-30, 45, 0], [45, 180, 180]]
maneuver(angles, 'ZYX', True)
