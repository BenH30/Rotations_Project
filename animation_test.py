import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Animation code from ImportanceOfBeingErnest on stackoverflow
# https://stackoverflow.com/questions/55169099/animating-a-3d-vector#:~:text=I%20need%20to%20animate%20both%20the

# Initialize figure and 3D axis
fig, ax = plt.subplots(subplot_kw=dict(projection="3d"))


# Define the initial vector (along the x-axis)
def get_arrow(theta):
    x = 0  # Starting point x
    y = 0  # Starting point y
    z = 0  # Starting point z

    # The vector components will rotate around the z-axis
    u = np.cos(theta)  # X component of the vector
    v = np.sin(theta)  # Y component of the vector
    w = 0  # Z component remains 0 for rotation around z-axis

    return x, y, z, u, v, w


# Initial quiver plot with the vector
quiver = ax.quiver(*get_arrow(0))

# Set up the 3D plot limits and labels
ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
ax.set_zlim(-1, 1)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')


# Function to update the arrow as it rotates
def update(theta):
    global quiver
    quiver.remove()  # Remove the old quiver
    quiver = ax.quiver(*get_arrow(theta))  # Create a new quiver for the updated vector


# Create the animation with 90 degrees (pi/2 radians) rotation
ani = FuncAnimation(fig, update, frames=np.linspace(0, np.pi / 2, 100), interval=50)

# Save the animation as a GIF
ani.save("rotation_animation.gif", writer='imagemagick')

# Show the plot (optional, as the animation is saved as a file)
plt.show()
