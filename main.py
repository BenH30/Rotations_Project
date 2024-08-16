import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R


def plot_setup(axis, origin, reference_frame, reference_frame_label, maneuver_angles=None, sequence='ZYX'):
    o = np.array(origin)
    x_color = 'r'
    y_color = 'g'
    z_color = 'b'

    # Plot each axis as a separate quiver
    axis.quiver(o[0], o[1], o[2], reference_frame[0, 0], reference_frame[1, 0], reference_frame[2, 0], color=x_color,
                length=0.5)
    axis.text(o[0] + reference_frame[0, 0], o[1] + reference_frame[1, 0], o[2] + reference_frame[2, 0], ' X',
              color=x_color)

    axis.quiver(o[0], o[1], o[2], reference_frame[0, 1], reference_frame[1, 1], reference_frame[2, 1], color=y_color,
                length=0.5)
    axis.text(o[0] + reference_frame[0, 1], o[1] + reference_frame[1, 1], o[2] + reference_frame[2, 1], ' Y',
              color=y_color)

    axis.quiver(o[0], o[1], o[2], reference_frame[0, 2], reference_frame[1, 2], reference_frame[2, 2], color=z_color,
                length=0.5)
    axis.text(o[0] + reference_frame[0, 2], o[1] + reference_frame[1, 2], o[2] + reference_frame[2, 2], ' Z',
              color=z_color)

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
        maneuver_angles = np.round(maneuver_angles, decimals=2)
        maneuver_angle_string = f'Maneuver: \nY = {maneuver_angles[0]}\nP = {maneuver_angles[1]}\nR = {maneuver_angles[2]}'
        axis.text2D(x=-0.4, y=0.25, s=maneuver_angle_string, transform=axis.transAxes, fontsize=12)

    attitude = R.from_matrix(reference_frame).as_euler(seq=sequence, degrees=True)
    # print('output', attitude)
    attitude_angle_string = f'Attitude: \nY = {np.round(attitude[0], 2)}\nP = {np.round(attitude[1], 2)}\nR = {np.round(attitude[2], 2)}'
    axis.text2D(x=1, y=0.25, s=attitude_angle_string, transform=axis.transAxes, fontsize=12)


def compute_attitude(euler_angle_list, euler_seq, degrees=True, origin=np.array([0, 0, 0])):
    fig = plt.figure(figsize=(9, 16))
    maneuver_count = 1
    rotation_list = []
    total_maneuvers = len(euler_angle_list)
    num_rows = int(np.ceil(total_maneuvers ** 0.5))
    num_columns = int(np.ceil(total_maneuvers / num_rows))
    column_index = 1
    row_index = 1

    for maneuver_angle_set in euler_angle_list:
        print(row_index, column_index)
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
        axis = fig.add_subplot(num_rows, num_columns, maneuver_count, projection='3d')
        plot_setup(axis, origin, apply_rotation.as_matrix(), axis_label, maneuver_angles=maneuver_angle_set)
        rotation_list.append(apply_rotation)
        maneuver_count += 1
        row_index += 1
        if row_index % num_rows == 1:
            column_index += 1
            row_index = 1

    plt.tight_layout()
    plt.subplots_adjust(top=0.9, hspace=0.5)

    plt.show()


def compute_maneuver(attitude_list, euler_sequence, degrees=True, origin=np.array([0, 0, 0])):
    fig = plt.figure(figsize=(9, 16))
    rotation_list = []
    total_maneuvers = len(attitude_list) - 1
    num_rows = int(np.ceil((total_maneuvers + 1) ** 0.5))  # +1 for the initial attitude
    num_columns = int(np.ceil((total_maneuvers + 1) / num_rows))

    # Plot the initial attitude
    initial_attitude = attitude_list[0]
    initial_rotation = R.from_euler(seq=euler_sequence, angles=initial_attitude, degrees=degrees)
    axis = fig.add_subplot(num_rows, num_columns, 1, projection='3d')
    plot_setup(axis, origin, initial_rotation.as_matrix(), 'Initial Attitude')

    # Loop through the attitude list to calculate and plot maneuvers
    for index in range(1, len(attitude_list)):
        att1 = R.from_euler(seq=euler_sequence, angles=attitude_list[index - 1], degrees=degrees)
        att2 = R.from_euler(seq=euler_sequence, angles=attitude_list[index], degrees=degrees)
        rotation = att2 * att1.inv()
        mnvr_angles = rotation.as_euler(seq=euler_sequence, degrees=degrees)

        # Create a subplot for each maneuver
        maneuver_count = index
        row_index = (maneuver_count) % num_rows + 1
        column_index = (maneuver_count) // num_rows + 1
        axis = fig.add_subplot(num_rows, num_columns, maneuver_count + 1, projection='3d')

        # Set the title for each plot
        if maneuver_count == 0:
            axis_label = 'Initial Attitude'
        elif maneuver_count == len(attitude_list)-1:
            axis_label = 'Final Attitude'
        else:
            axis_label = 'Maneuver ' + str(maneuver_count)

        # Plot setup with calculated maneuver angles
        plot_setup(axis, origin, rotation.as_matrix(), axis_label, maneuver_angles=mnvr_angles)

    plt.tight_layout()
    plt.subplots_adjust(top=0.9, hspace=0.5)
    plt.show()


# Example usage
angles = [[0, 0, 0], [30, 0, 0], [-30, 0, 0], [45, 180, 180], [-146.31389377, -32.49235207, 29.01456352]]
attitudes = [[0, 0, 0], [30, 0, 0], [0, 0, 0], [45, 180, 180], [-4.7, 5.2, -42.21]]
compute_attitude(angles, 'ZYX', True)
compute_maneuver(angles, 'ZYX', True)

#todo adjust titles closer to plot
#todo move plots to the left


