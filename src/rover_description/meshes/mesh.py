import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from stl import mesh
import matplotlib.pyplot as plt

# Load STL file
your_mesh = mesh.Mesh.from_file('pioneer_3dx.stl')

# Create figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Get mesh coordinates
ax.add_collection3d(Poly3DCollection(your_mesh.vectors, alpha=0.5, edgecolor='k'))

# Adjust axis scale
scale = your_mesh.points.flatten()
ax.auto_scale_xyz(scale, scale, scale)

# Display
plt.show()
