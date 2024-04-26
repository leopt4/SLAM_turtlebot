from pypcd4 import PointCloud
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Load the PCD files using pypcd4
pc_source = PointCloud.from_path('source.pcd')
pc_aligned = PointCloud.from_path('aligned.pcd')
pc_target = PointCloud.from_path('target.pcd')

# Convert the point cloud data to a NumPy array
points_source = pc_source.numpy(("x", "y", "z"))
points_aligned = pc_aligned.numpy(("x", "y", "z"))
points_target = pc_target.numpy(("x", "y", "z"))

# Separate the x, y, and z coordinates for each point cloud
x_source, y_source, z_source = points_source[:, 0], points_source[:, 1], points_source[:, 2]
x_aligned, y_aligned, z_aligned = points_aligned[:, 0], points_aligned[:, 1], points_aligned[:, 2]
x_target, y_target, z_target = points_target[:, 0], points_target[:, 1], points_target[:, 2]

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the point clouds with different colors and labels
ax.scatter(x_source, y_source, z_source, c='r', label='Source')
ax.scatter(x_aligned, y_aligned, z_aligned, c='g', label='Aligned')
ax.scatter(x_target, y_target, z_target, c='b', label='Target')

# Set the labels for the axes
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Add a legend
ax.legend()

# Show the plot
plt.show()
