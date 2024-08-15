import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

def plot_setup(axis, origin, reference_frame, reference_frame_label, maneuver_angles=None, sequence='ZYX'):
    o = np.array(origin)
    x_color = 'r'
    y_color = 'g'
    z_color = 'b'

    # Plot each axis as a separate quiver
    axis.quiver(o[0], o[1], o[2], reference_frame[0, 0], reference_frame[1, 0], reference_frame[2, 0], color=x_color, length=0.5)
    axis.text(o[0] + reference_frame[0, 0], o[1] + reference_frame[1, 0], o[2] + reference_frame[2, 0], ' X', color=x_color)

    axis.quiver(o[0], o[1], o[2], reference_frame[0, 1], reference_frame[1, 1], reference_frame[2, 1], color=y_color, length=0.5)
    axis.text(o[0] + reference_frame[0, 1], o[1] + reference_frame[1, 1], o[2] + reference_frame[2, 1], ' Y', color=y_color)

    axis.quiver(o[0], o[1], o[2], reference_frame[0, 2], reference_frame[1, 2], reference_frame[2, 2], color=z_color, length=0.5)
    axis.text(o[0] + reference_frame[0, 2], o[1] + reference_frame[1, 2], o[2] + reference_frame[2, 2], ' Z', color=z_color)

    axis.set_xlabel('x')
    axis.set_ylabel('y')
    axis.set_zlabel('z')
    axis.set_title(reference_frame_label)
    axis.set_xlim(-1, 1)
    axis.set_ylim(-1, 1)
    axis.set_zlim(-1, 1)
    axis.xaxis.set_ticks(np.arange(-1, 1.5, 1))
    axis.yaxis.set_ticks(np.arange(-1, 1.5, 1))
    axis.zaxis.set_ticks(np.arange(-1, 1.5, 1))
    axis.view_init(azim=110, elev=200)

    if maneuver_angles is not None:
        maneuver_angle_string = f'Maneuver: \nY = {maneuver_angles[0]}\nP = {maneuver_angles[1]}\nR = {maneuver_angles[2]}'
        axis.text2D(x=1, y=0.25, s=maneuver_angle_string, transform=axis.transAxes, fontsize=14)

    attitude = R.from_matrix(reference_frame).as_euler(seq=sequence, degrees=True)
    # print('output', attitude)
    attitude_angle_string = f'Attitude: \nY = {np.round(attitude[0], 2)}\nP = {np.round(attitude[1], 2)}\nR = {np.round(attitude[2], 2)}'
    axis.text2D(x=1.5, y=0.25, s=attitude_angle_string, transform=axis.transAxes, fontsize=14)

def maneuver(euler_angle_list, euler_seq, degrees=True, origin=np.array([0, 0, 0])):
    fig = plt.figure(figsize=(9, 16))
    maneuver_count = 1
    rotation_list = []

    for maneuver_angle_set in euler_angle_list:
        rotation = R.from_euler(seq=euler_seq, angles=maneuver_angle_set, degrees=degrees)
        if maneuver_count == 1:
            axis_label = 'Initial Attitude'
            apply_rotation = rotation
        elif maneuver_count == len(euler_angle_list):
            axis_label = 'Final Attitude'
            apply_rotation = (rotation * rotation_list[-1])
        else:
            axis_label = 'Maneuver ' + str(maneuver_count)
            apply_rotation = (rotation * rotation_list[-1])
        axis = fig.add_subplot(len(euler_angle_list), 1, maneuver_count, projection='3d')
        plot_setup(axis, origin, apply_rotation.as_matrix(), axis_label, maneuver_angles=maneuver_angle_set)
        rotation_list.append(apply_rotation)
        maneuver_count += 1

    plt.show()

    #for rotation_i in rotation_list:
        #print(rotation_i.as_euler('zyx', degrees=True))

# Example usage
angles = [[0, 0, 0], [30, 0, 0], [-30, 0, 0], [45, 180, 180]]
maneuver(angles, 'ZYX', True)

#todo adjust titles closer to plot
#todo move plots to the left