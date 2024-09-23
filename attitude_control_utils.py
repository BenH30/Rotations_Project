import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R


def compute_single_rotation(initial_attitude, euler_angles, euler_angle_type='commanded_attitude', euler_sequence='ZYX',
                            degrees=True):
    initial_attitude = R.from_euler(seq=euler_sequence, angles=initial_attitude, degrees=degrees)
    if euler_angle_type == 'commanded_attitude':
        final_attitude = R.from_euler(seq=euler_sequence, angles=euler_angles, degrees=degrees)
        rotation = final_attitude * initial_attitude.inv()
        return rotation, final_attitude
    elif euler_angle_type == 'commanded_maneuver':
        rotation = R.from_euler(seq=euler_sequence, angles=euler_angles, degrees=degrees)
        final_attitude = rotation * initial_attitude
        return rotation, final_attitude
    else:
        raise ValueError('angle_type must be commanded_attitude or commanded_maneuver')


def combine_rotations(initial_attitude, angle_dictionary, euler_angle_type='commanded_attitude', euler_sequence='ZYX',
                      degrees=True):
    maneuver_dictionary = dict()
    attitude_dictionary = {0: initial_attitude}
    resulting_attitude = initial_attitude
    for attitude_index, angles in angle_dictionary.items():
        if attitude_index == 1:
            rotation, resulting_attitude = compute_single_rotation(initial_attitude=attitude_dictionary[0],
                                                                   euler_angles=angles,
                                                                   euler_angle_type=euler_angle_type,
                                                                   euler_sequence=euler_sequence, degrees=degrees)
        else:
            rotation, resulting_attitude = compute_single_rotation(
                initial_attitude=resulting_attitude.as_euler(seq=euler_sequence, degrees=degrees),
                euler_angles=angles, euler_angle_type=euler_angle_type,
                euler_sequence=euler_sequence, degrees=degrees)

        maneuver_dictionary[attitude_index] = rotation.as_euler(seq=euler_sequence, degrees=degrees)
        attitude_dictionary[attitude_index] = resulting_attitude.as_euler(seq=euler_sequence, degrees=degrees)

    return maneuver_dictionary, attitude_dictionary


def print_maneuvers(initial_attitude, maneuver_dictionary, attitude_dictionary):
    dictionary_difference = len(attitude_dictionary) - len(maneuver_dictionary)
    if dictionary_difference > 1:
        raise ValueError(f'{dictionary_difference} attitudes without corresponding maneuvers')
    elif dictionary_difference < 1:
        raise ValueError(f'{dictionary_difference} maneuvers without corresponding attitudes')
    num_maneuvers = len(maneuver_dictionary)
    attitude_dictionary[0] = initial_attitude
    for maneuver_index in range(1, num_maneuvers + 1):
        print(f'=====================================')
        print(f'Starting Attitude: {attitude_dictionary[maneuver_index - 1]}')
        print(f'Maneuver Required: {maneuver_dictionary[maneuver_index]}')
        print(f'Ending Attitude:   {attitude_dictionary[maneuver_index]}')


def plot_setup(axis, reference_frame, reference_frame_label, origin=np.array([0, 0, 0]), maneuver_angles=None,
               sequence='ZYX', attitude_in=None, attitude_out=None):
    x_color = 'r'
    y_color = 'g'
    z_color = 'b'

    # Plot each axis as a separate quiver
    axis.quiver(origin[0], origin[1], origin[2],
                reference_frame[0, 0], reference_frame[1, 0], reference_frame[2, 0],
                color=x_color, length=0.5)
    axis.text(origin[0] + reference_frame[0, 0], origin[1] + reference_frame[1, 0], origin[2] + reference_frame[2, 0],
              ' X', color=x_color)

    axis.quiver(origin[0], origin[1], origin[2],
                reference_frame[0, 1], reference_frame[1, 1], reference_frame[2, 1],
                color=y_color, length=0.5)
    axis.text(origin[0] + reference_frame[0, 1], origin[1] + reference_frame[1, 1], origin[2] + reference_frame[2, 1],
              ' Y', color=y_color)

    axis.quiver(origin[0], origin[1], origin[2],
                reference_frame[0, 2], reference_frame[1, 2], reference_frame[2, 2],
                color=z_color, length=0.5)
    axis.text(origin[0] + reference_frame[0, 2], origin[1] + reference_frame[1, 2], origin[2] + reference_frame[2, 2],
              ' Z', color=z_color)

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

    # Standardize text display vertically with bounds
    y_positions = [0.65, 0.4, 0.15]  # Fixed y-positions within the plot bounds
    text_lines = []

    if attitude_in is not None:
        attitude_in = np.round(attitude_in, 2)
        text_lines.append(f'Attitude In:\nY = {attitude_in[0]}\nP = {attitude_in[1]}\nR = {attitude_in[2]}')
    if maneuver_angles is not None:
        maneuver_angles = np.round(maneuver_angles, 2)
        text_lines.append(f'Maneuver:\nY = {maneuver_angles[0]}\nP = {maneuver_angles[1]}\nR = {maneuver_angles[2]}')
    if attitude_out is not None:
        attitude_out = np.round(attitude_out, 2)
        text_lines.append(f'Attitude Out:\nY = {attitude_out[0]}\nP = {attitude_out[1]}\nR = {attitude_out[2]}')

    for i, text in enumerate(text_lines):
        axis.text2D(x=1.05, y=y_positions[i], s=text, transform=axis.transAxes, fontsize=10, ha='left')


def plot_attitudes(attitude_dictionary, maneuver_dictionary, euler_sequence='ZYX', degrees=True):
    total_plots = len(attitude_dictionary)
    num_rows = int(np.ceil(total_plots ** 0.5))
    num_columns = int(np.ceil(total_plots / num_rows))

    fig = plt.figure(figsize=(num_columns * 4, num_rows * 4))
    fig.suptitle('Maneuver Plotter', y=0.95, fontsize=16)

    for index, attitude_angles in attitude_dictionary.items():
        axis = fig.add_subplot(num_rows, num_columns, index + 1, projection='3d')
        if index == 0:
            axis_label = 'Initial Attitude'
            attitude_in = attitude_dictionary[0]
            plot_setup(axis,
                       R.from_euler(angles=attitude_dictionary[0], seq=euler_sequence, degrees=degrees).as_matrix(),
                       axis_label, attitude_in=attitude_in)
        else:
            attitude_in = attitude_dictionary[index - 1]
            attitude_out = attitude_dictionary[index]
            plot_setup(axis, R.from_euler(angles=attitude_out, seq=euler_sequence, degrees=degrees).as_matrix(),
                       f'Maneuver {index}', maneuver_angles=maneuver_dictionary[index],
                       attitude_in=attitude_in, attitude_out=attitude_out)

    plt.tight_layout(pad=3.0)
    plt.subplots_adjust(top=0.92, hspace=0.5, wspace=0.5)
    plt.show()


def plot_single_maneuver(initial_attitude, final_attitude, euler_sequence='ZYX', degrees=True,
                         origin=np.array([0, 0, 0])):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    initial_rotation = R.from_euler(angles=initial_attitude, seq=euler_sequence, degrees=degrees)
    plot_setup(ax, initial_rotation.as_matrix(), "Initial Attitude", origin=origin)

    final_rotation_matrix = R.from_euler(angles=final_attitude, seq=euler_sequence, degrees=degrees).as_matrix()

    length = 0.75
    label_distance = 0.75

    x_color, y_color, z_color = 'r', 'g', 'b'
    ax.quiver(*origin, final_rotation_matrix[0, 0], final_rotation_matrix[1, 0], final_rotation_matrix[2, 0],
              color=x_color, linestyle='dashed', length=length)
    ax.text(origin[0] + final_rotation_matrix[0, 0] * label_distance,
            origin[1] + final_rotation_matrix[1, 0] * label_distance,
            origin[2] + final_rotation_matrix[2, 0] * label_distance,
            "X'", color=x_color)

    ax.quiver(*origin, final_rotation_matrix[0, 1], final_rotation_matrix[1, 1], final_rotation_matrix[2, 1],
              color=y_color, linestyle='dashed', length=length)
    ax.text(origin[0] + final_rotation_matrix[0, 1] * label_distance,
            origin[1] + final_rotation_matrix[1, 1] * label_distance,
            origin[2] + final_rotation_matrix[2, 1] * label_distance,
            "Y'", color=y_color)

    ax.quiver(*origin, final_rotation_matrix[0, 2], final_rotation_matrix[1, 2], final_rotation_matrix[2, 2],
              color=z_color, linestyle='dashed', length=length)
    ax.text(origin[0] + final_rotation_matrix[0, 2] * label_distance,
            origin[1] + final_rotation_matrix[1, 2] * label_distance,
            origin[2] + final_rotation_matrix[2, 2] * label_distance,
            "Z'", color=z_color)

    # Standardize text display vertically within plot bounds for single maneuver
    y_positions = [0.75, 0.5, 0.25]  # Fixed y-positions within the plot bounds
    text_lines = []

    # Attitude In and Attitude Out text display
    attitude_in = np.round(initial_attitude, 2)
    attitude_out = np.round(final_attitude, 2)
    text_lines.append(f'Attitude In:\nY = {attitude_in[0]}\nP = {attitude_in[1]}\nR = {attitude_in[2]}')
    text_lines.append(f'Attitude Out:\nY = {attitude_out[0]}\nP = {attitude_out[1]}\nR = {attitude_out[2]}')

    for i, text in enumerate(text_lines):
        ax.text2D(x=1.05, y=y_positions[i], s=text, transform=ax.transAxes, fontsize=10, ha='left')

    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_zlim(-1, 1)
    ax.xaxis.set_ticks(np.arange(-1, 1.5, 1))
    ax.yaxis.set_ticks(np.arange(-1, 1.5, 1))
    ax.zaxis.set_ticks(np.arange(-1, 1.5, 1))
    ax.view_init(azim=110, elev=200)

    plt.show()


initial_att = [0, 0, 0]
att_commands = {1: [20, 0, 0], 2: [20, 15, 0], 3: [0, 0, 0]}

maneuver_out, att_out = combine_rotations(initial_att, att_commands, euler_angle_type='commanded_attitude')

plot_attitudes(att_out, maneuver_out, euler_sequence='ZYX', degrees=True)
