import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Define a function to plot reference frames with Z-axis positive downwards and swapped X and Y axes
def plot_frame(ax, origin, R_matrix, frame_label, color):
    axis_length = 1.0
    o = np.array(origin)

    ax.quiver(o[1], o[0], o[2], R_matrix[1, 0], R_matrix[0, 0], R_matrix[2, 0], color=color, length=axis_length)
    ax.text(o[1] + R_matrix[1, 0], o[0] + R_matrix[0, 0], o[2] + R_matrix[2, 0], frame_label + ' X', color=color)
    ax.quiver(o[1], o[0], o[2], R_matrix[1, 1], R_matrix[0, 1], R_matrix[2, 1], color=color, length=axis_length)
    ax.text(o[1] + R_matrix[1, 1], o[0] + R_matrix[0, 1], o[2] + R_matrix[2, 1], frame_label + ' Y', color=color)
    ax.quiver(o[1], o[0], o[2], R_matrix[1, 2], R_matrix[0, 2], R_matrix[2, 2], color=color, length=axis_length)
    ax.text(o[1] + R_matrix[1, 2], o[0] + R_matrix[0, 2], o[2] + R_matrix[2, 2], frame_label + ' Z', color=color)

# Correcting the initial inertial and body frame matrices with swapped X and Y axes
R_inertial = np.array([[1, 0, 0],
                       [0, 1, 0],
                       [0, 0, -1]])

# First Rotation (Yaw, Pitch, Roll)
initial_euler_angles = [30, 20, 10]  # Example initial YPR rotation
initial_euler_angles_rad = np.radians(initial_euler_angles)
rotation_quat_1 = R.from_euler('ZYX', initial_euler_angles_rad)

# Get the rotation matrix after the first rotation
R_body_rotated_1 = rotation_quat_1.as_matrix() @ R_inertial

# Second Rotation (Yaw, Pitch, Roll)
second_euler_angles = [90, 45, 0]  # Second YPR rotation
second_euler_angles_rad = np.radians(second_euler_angles)
rotation_quat_2 = R.from_euler('ZYX', second_euler_angles_rad)

# Combine the two rotations (quaternion multiplication)
final_quat = rotation_quat_2 * rotation_quat_1

# Get the final rotation matrix
R_body_rotated_final = final_quat.as_matrix() @ R_inertial

# Plot the initial, intermediate, and final frames with Z-axis positive downwards and swapped X and Y axes
fig = plt.figure(figsize=(18, 6))

# Plot the initial frames
ax1 = fig.add_subplot(131, projection='3d')
plot_frame(ax1, [0, 0, 0], R_inertial, 'Inertial', 'blue')
plot_frame(ax1, [0, 0, 0], R_inertial, 'Body (initial)', 'red')
ax1.set_title('Initial Frame Alignment')
ax1.set_xlim([1, -1])  # Swap X-axis
ax1.set_ylim([1, -1])
ax1.set_zlim([1, -1])  # Invert Z-axis
ax1.set_xlabel('Y')
ax1.set_ylabel('X')
ax1.set_zlabel('Z (Downwards)')

# Plot the rotated body frame after the first rotation
ax2 = fig.add_subplot(132, projection='3d')
plot_frame(ax2, [0, 0, 0], R_inertial, 'Inertial', 'blue')
plot_frame(ax2, [0, 0, 0], R_body_rotated_1, 'Body (after 1st rotation)', 'red')
ax2.set_title('Body Frame After First YPR Rotation')
ax2.set_xlim([1, -1])  # Swap X-axis
ax1.set_ylim([1, -1])
ax2.set_zlim([1, -1])  # Invert Z-axis
ax2.set_xlabel('Y')
ax2.set_ylabel('X')
ax2.set_zlabel('Z (Downwards)')

# Plot the final rotated body frame after the second rotation
ax3 = fig.add_subplot(133, projection='3d')
plot_frame(ax3, [0, 0, 0], R_inertial, 'Inertial', 'blue')
plot_frame(ax3, [0, 0, 0], R_body_rotated_final, 'Body (final)', 'red')
ax3.set_title('Body Frame After Second YPR Rotation')
ax3.set_xlim([1, -1])  # Swap X-axis
ax1.set_ylim([1, -1])
ax3.set_zlim([1, -1])  # Invert Z-axis
ax3.set_xlabel('Y')
ax3.set_ylabel('X')
ax3.set_zlabel('Z (Downwards)')

plt.show()

# Convert the final quaternion back to Euler angles to display the final attitude
final_euler_angles = final_quat.as_euler('ZYX', degrees=True)
print("Final body frame attitude in Euler angles (Yaw, Pitch, Roll):")
print(final_euler_angles)
