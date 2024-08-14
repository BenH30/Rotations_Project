import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d


def plot_setup(axis, origin, frame, frame_label):
    o = np.array(origin)
    color = 'b'

    plt.quiver(o[0], o[1], o[2], frame[0], frame[1], frame[2], color=color, length=0.5)
    # print(frame[:,0], frame[0,:])
    axis.text(o[0] + frame[0, 0] + 0.25, o[1] + frame[1, 0]-.25, o[2] + frame[2, 0], frame_label + ' X', color=color)
    axis.text(o[0] + frame[0, 1], o[1] + frame[1, 1], o[2] + frame[2, 1], frame_label + ' Y', color=color)
    axis.text(o[0] + frame[0, 2], o[1] + frame[1, 2], o[2] + frame[2, 2], frame_label + ' Z', color=color)

    axis.set_xlabel('x')
    axis.set_ylabel('y')
    axis.set_zlabel('z')
    axis.set_title(frame_label)
    axis.set_xlim(-1, 1)
    axis.set_ylim(-1, 1)
    axis.set_zlim(-1, 1)
    axis.view_init(azim=110, elev=200)


# fig = plt.figure(figsize=(18, 6))
# ax1 = fig.add_subplot(131, projection='3d')

ax = plt.axes(projection='3d')
R_inertial = np.array([[1, 0, 0],
                       [0, 1, 0],
                       [0, 0, 1]])
inertial_origin = np.array([0, 0, 0])
label = 'Inertial Frame'
plot_setup(ax, inertial_origin, R_inertial, label)
plt.show()
