import numpy as np

roll_angle = 0
pitch_angle = 0
yaw_angle = np.deg2rad(45)

Rx = np.array([[1, 0, 0], [0, np.cos(roll_angle), -np.sin(roll_angle)], [0, np.sin(roll_angle), np.cos(roll_angle)]])
Ry = np.array(
    [[np.cos(pitch_angle), 0, np.sin(pitch_angle)], [0, 1, 0], [-np.sin(pitch_angle), 0, np.cos(pitch_angle)]])
Rz = np.array([[np.cos(yaw_angle), -np.sin(yaw_angle), 0], [np.sin(yaw_angle), np.cos(yaw_angle), 0], [0, 0, 1]])

init = np.array([np.pi, 0, 0])
out = np.matmul(Rz, init)
print(np.rad2deg(out))
