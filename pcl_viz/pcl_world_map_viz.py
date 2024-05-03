from pypcd4 import PointCloud
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

map_source = PointCloud.from_path('world_map.pcd')

map_source = map_source.numpy(("x","y","z"))

x_source, y_source, z_source = map_source[:, 0], map_source[:, 1], map_source[:, 2]

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the point clouds with different colors and labels
ax.scatter(x_source, y_source, z_source, c='r', label='Source', s=2)


# Set the labels for the axes
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Add a legend
ax.legend()

# Show the plot
plt.show()