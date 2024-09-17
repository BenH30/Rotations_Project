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
        maneuver_angle_string = f'Maneuver: \nY = {maneuver_angles[0]:.2f}\nP = {maneuver_angles[1]:.2f}\nR = {maneuver_angles[2]:.2f}'
        axis.text2D(x=-0.3, y=0.25, s=maneuver_angle_string, transform=axis.transAxes, fontsize=12)

    attitude = R.from_matrix(reference_frame).as_euler(seq=sequence, degrees=True)
    attitude_angle_string = f'Attitude: \nY = {attitude[0]:.2f}\nP = {attitude[1]:.2f}\nR = {attitude[2]:.2f}'
    axis.text2D(x=1, y=0.25, s=attitude_angle_string, transform=axis.transAxes, fontsize=12)


def compute_attitude(euler_angle_list, euler_sequence, degrees=True, origin=np.array([0, 0, 0])):
    fig = plt.figure(figsize=(9, 16))
    attitude_list = [R.from_euler(seq=euler_sequence, angles=euler_angle_list[0], degrees=degrees)]
    total_maneuvers = len(euler_angle_list)
    num_rows = int(np.ceil(total_maneuvers ** 0.5))
    num_columns = int(np.ceil(total_maneuvers / num_rows))

    # Plot the initial attitude
    initial_attitude = attitude_list[0].as_euler(seq=euler_sequence, degrees=True)
    initial_rotation = R.from_euler(seq=euler_sequence, angles=initial_attitude, degrees=degrees)
    axis = fig.add_subplot(num_rows, num_columns, 1, projection='3d')
    plot_setup(axis, origin, initial_rotation.as_matrix(), 'Initial Attitude')

    for attitude_index in range(1, total_maneuvers):
        maneuver_euler_angles = euler_angle_list[attitude_index]
        maneuver_rotation = R.from_euler(seq=euler_sequence, angles=maneuver_euler_angles, degrees=degrees)

        # Create a subplot for each maneuver
        maneuver_count = attitude_index
        axis = fig.add_subplot(num_rows, num_columns, maneuver_count + 1, projection='3d')

        # Set the title for each plot
        if maneuver_count == 0:
            axis_label = 'Initial Attitude'
            post_maneuver_attitude = maneuver_rotation
        elif maneuver_count == total_maneuvers - 1:
            axis_label = 'Final Attitude'
            post_maneuver_attitude = (maneuver_rotation * attitude_list[-1])
        else:
            axis_label = 'Maneuver ' + str(maneuver_count)
            post_maneuver_attitude = (maneuver_rotation * attitude_list[-1])

        axis_title = 'Compute Attitude'
        fig.suptitle(axis_title)
        plot_setup(axis, origin, post_maneuver_attitude.as_matrix(), axis_label, maneuver_angles=maneuver_euler_angles)
        attitude_list.append(post_maneuver_attitude)
        maneuver_count += 1

    plt.tight_layout()
    plt.subplots_adjust(top=0.9, hspace=0.5)

    plt.show()


def compute_maneuver(attitude_list, euler_sequence, degrees=True, origin=np.array([0, 0, 0])):
    fig = plt.figure(figsize=(9, 16))
    total_maneuvers = len(attitude_list)
    num_rows = int(np.ceil(total_maneuvers ** 0.5))
    num_columns = int(np.ceil(total_maneuvers / num_rows))

    # Plot the initial attitude
    initial_attitude = attitude_list[0]
    initial_rotation = R.from_euler(seq=euler_sequence, angles=initial_attitude, degrees=degrees)
    axis = fig.add_subplot(num_rows, num_columns, 1, projection='3d')
    plot_setup(axis, origin, initial_rotation.as_matrix(), 'Initial Attitude')

    # Loop through the attitude list to calculate and plot maneuvers
    for attitude_index in range(1, len(attitude_list)):
        pre_maneuver_attitude = R.from_euler(seq=euler_sequence, angles=attitude_list[attitude_index - 1],
                                             degrees=degrees)
        post_maneuver_attitude = R.from_euler(seq=euler_sequence, angles=attitude_list[attitude_index], degrees=degrees)
        maneuver_rotation = post_maneuver_attitude * pre_maneuver_attitude.inv()
        maneuver_angles = maneuver_rotation.as_euler(seq=euler_sequence, degrees=degrees)

        # Create a subplot for each maneuver
        maneuver_count = attitude_index
        axis = fig.add_subplot(num_rows, num_columns, maneuver_count + 1, projection='3d')

        # Set the title for each plot
        if maneuver_count == 0:
            axis_label = 'Initial Attitude'
        elif maneuver_count == len(attitude_list) - 1:
            axis_label = 'Final Attitude'
        else:
            axis_label = 'Maneuver ' + str(maneuver_count)

        # Plot setup with calculated maneuver angles
        axis_title = 'Compute Maneuver'
        fig.suptitle(axis_title)
        plot_setup(axis, origin, post_maneuver_attitude.as_matrix(), axis_label, maneuver_angles=maneuver_angles)

    plt.tight_layout()
    plt.subplots_adjust(top=0.9, hspace=0.5)
    plt.show()


# Example usage
angles = [[0, 0, 0], [30, 0, 0], [-30, 0, 0], [45, 180, 180], [-146.31389377, -32.49235207, 29.01456352]]
attitudes = [[0, 0, 0], [30, 0, 0], [0, 0, 0], [-135, 0, 0], [90, 42, 7]]
compute_attitude(angles, 'ZYX', True)
compute_maneuver(attitudes, 'ZYX', True)

#todo adjust titles closer to plot
#todo move plots to the left
