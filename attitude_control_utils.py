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
        # For the first set of angles, use the initial attitude as the starting point instead of the previous rotation
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

        # If the user is providing the angles as attitudes, set the angles to the attitude dictionary and the rotation to the maneuver dictionary
        # If the user is providing the angles as maneuvers, set the angles to the maneuver dictionary and the resulting attitude to the attitude dictionary
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
               sequence='ZYX'):
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

    # Adjust position of maneuver and attitude text to be right of the plot and aligned
    if maneuver_angles is not None:
        maneuver_angles = np.round(maneuver_angles, decimals=2)
        maneuver_angle_string = f'Maneuver: \nY = {maneuver_angles[0]:.2f}\nP = {maneuver_angles[1]:.2f}\nR = {maneuver_angles[2]:.2f}'
        axis.text2D(x=1.05, y=0.5, s=maneuver_angle_string, transform=axis.transAxes, fontsize=10, ha='left')

    # Display attitude right below maneuver, left-aligned with maneuver
    attitude = R.from_matrix(reference_frame).as_euler(seq=sequence, degrees=True)
    attitude_angle_string = f'Attitude: \nY = {attitude[0]:.2f}\nP = {attitude[1]:.2f}\nR = {attitude[2]:.2f}'
    axis.text2D(x=1.05, y=0.2, s=attitude_angle_string, transform=axis.transAxes, fontsize=10, ha='left')


def plot_attitudes(attitude_dictionary, maneuver_dictionary):
    # Dynamically adjust figure size based on number of subplots
    total_plots = len(attitude_dictionary)
    num_rows = int(np.ceil(total_plots ** 0.5))
    num_columns = int(np.ceil(total_plots / num_rows))

    # Dynamic figure size: width and height scale with the number of rows/columns
    fig = plt.figure(figsize=(num_columns * 4, num_rows * 4))

    # Adjust title position
    axis_title = 'Maneuver Plotter'
    fig.suptitle(axis_title, y=0.95, fontsize=16)

    for index, attitude_angles in attitude_dictionary.items():
        # Create a subplot for each maneuver
        axis = fig.add_subplot(num_rows, num_columns, index + 1, projection='3d')

        # Set the title for each plot
        if total_plots == 1:
            axis_label = 'Input Attitude'
            post_maneuver_attitude = R.from_euler(angles=attitude_angles, seq=euler_sequence, degrees=degrees)
        elif index == 0:
            axis_label = 'Initial Attitude'
            post_maneuver_attitude = R.from_euler(angles=attitude_dictionary[0], seq=euler_sequence, degrees=degrees)
            plot_setup(axis, post_maneuver_attitude.as_matrix(), axis_label)
            continue
        elif index == len(attitude_dictionary) - 1:
            axis_label = 'Final Attitude'
            post_maneuver_attitude = R.from_euler(angles=attitude_dictionary[index], seq=euler_sequence,
                                                  degrees=degrees)
        else:
            axis_label = 'Maneuver ' + str(index)
            post_maneuver_attitude = R.from_euler(angles=attitude_dictionary[index], seq=euler_sequence,
                                                  degrees=degrees)

        plot_setup(axis, post_maneuver_attitude.as_matrix(), axis_label, maneuver_angles=maneuver_dictionary[index])

    # Adjust subplot layout and spacing
    plt.tight_layout(pad=3.0)  # Add padding between subplots
    plt.subplots_adjust(top=0.92, hspace=0.5, wspace=0.5)  # Adjust vertical and horizontal spacing
    plt.show()


def plot_single_maneuver(initial_attitude, final_attitude, euler_sequence='ZYX', degrees=True, origin=np.array([0, 0, 0])):
    """
    Plots the initial and final attitudes for a single maneuver.

    Parameters:
    - initial_attitude (list or np.array): Initial attitude in degrees or radians.
    - final_attitude (list or np.array): Final attitude in degrees or radians.
    - euler_sequence (str): Euler angle sequence. Default is 'ZYX'.
    - degrees (bool): Whether the angles are in degrees. Default is True.
    - origin (np.array): The origin point for the quiver arrows. Default is np.array([0, 0, 0]).
    """
    # Define colors for X, Y, Z axes
    x_color, y_color, z_color = 'r', 'g', 'b'

    # Convert final attitude to a rotation object
    final_angles = R.from_euler(angles=final_attitude, seq=euler_sequence, degrees=degrees)

    # Create the plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot initial attitude
    initial_rotation = R.from_euler(angles=initial_attitude, seq=euler_sequence, degrees=degrees)
    plot_setup(ax, initial_rotation.as_matrix(), "Initial Attitude", origin=origin)

    # Plot final attitude with dashed arrows and modified labels
    final_rotation_matrix = final_angles.as_matrix()

    length = 0.5  # Length of arrows
    label_distance = 0.75  # Factor to bring text 25% closer to the arrows

    # Plot X' axis
    ax.quiver(*origin, final_rotation_matrix[0, 0], final_rotation_matrix[1, 0], final_rotation_matrix[2, 0],
              color=x_color, linestyle='dashed', length=length)
    ax.text(origin[0] + label_distance * length * final_rotation_matrix[0, 0],
            origin[1] + label_distance * length * final_rotation_matrix[1, 0],
            origin[2] + label_distance * length * final_rotation_matrix[2, 0],
            "X'", color=x_color)

    # Plot Y' axis
    ax.quiver(*origin, final_rotation_matrix[0, 1], final_rotation_matrix[1, 1], final_rotation_matrix[2, 1],
              color=y_color, linestyle='dashed', length=length)
    ax.text(origin[0] + label_distance * length * final_rotation_matrix[0, 1],
            origin[1] + label_distance * length * final_rotation_matrix[1, 1],
            origin[2] + label_distance * length * final_rotation_matrix[2, 1],
            "Y'", color=y_color)

    # Plot Z' axis
    ax.quiver(*origin, final_rotation_matrix[0, 2], final_rotation_matrix[1, 2], final_rotation_matrix[2, 2],
              color=z_color, linestyle='dashed', length=length)
    ax.text(origin[0] + label_distance * length * final_rotation_matrix[0, 2],
            origin[1] + label_distance * length * final_rotation_matrix[1, 2],
            origin[2] + label_distance * length * final_rotation_matrix[2, 2],
            "Z'", color=z_color)

    # Set plot limits and labels
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Add title
    ax.set_title('Initial and Final Attitudes')

    plt.show()


# # # # # # # # # # # # #
# #    Format Rules   # #
# # # # # # # # # # # # #

# Always start with an initial attitude
# All dictionaries will start with an index of 1, as the initial attitude is considered 0
# When providing angles, the two options are commanded attitude or commanded maneuver:
#   Commanded attitude means the euler angles represent the attitude the vehicle is being commanded to, so the maneuver required to get there will be calculated
#       Ex: Initial attitude: [15, 0, 0], commanded attitude: [35, 0, 0], calculated maneuver: [20, 0, 0]
#   Commanded maneuver means the euler angles represent the maneuver the vehicle is being commanded to execute, so the resulting attitude will be calculated
#       Ex: Initial attitude: [15, 0, 0], commanded maneuver: [35, 0,0], calculated resulting attitude: [50, 0, 0]


# # # # # # # # # #
# #    Inputs   # #
# # # # # # # # # #

initial_attitude_angles = np.array([10., 0., 0.])
desired_attitudes = {1: [30, 0, 0], 2: [100, 0, 0], 3: [60, 0, 0], 4: [42, 18, 77]}
euler_sequence = 'ZYX'
degrees = True

# # # # # # # # # # # # # #
# #    Function Calls   # #
# # # # # # # # # # # # # #

predicted_maneuvers, commanded_attitudes = combine_rotations(initial_attitude=initial_attitude_angles,
                                                             angle_dictionary=desired_attitudes,
                                                             euler_angle_type='commanded_attitude',
                                                             euler_sequence=euler_sequence, degrees=degrees)

plot_attitudes(attitude_dictionary=commanded_attitudes, maneuver_dictionary=predicted_maneuvers)

plot_single_maneuver(initial_attitude=initial_attitude_angles, final_attitude=desired_attitudes[1], euler_sequence=euler_sequence, degrees=degrees)