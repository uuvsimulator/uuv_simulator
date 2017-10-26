import numpy as np
from scipy.interpolate import griddata
import scipy.ndimage as ndimage
from scipy.ndimage import gaussian_filter
from scipy.misc import imsave
from matplotlib import cm
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from stl import mesh, Mode
import matplotlib.tri as mtri
from mpl_toolkits.mplot3d.axes3d import get_test_data

# Generating the surface
x, y, z = get_test_data(delta=0.1)
# Scale the surface for this example
z *= 0.05
# Remember that Gazebo uses ENU (east-north-up) convention, so underwater
# the Z coordinate will be negative
z -= 3
# Note: Gazebo will import your mesh in meters. 

# Point clouds usually don't come in nice grids, so let's make it a (N, 3)
# matrix just to show how it can be done. If you have outliers or noise, you should
# treat those values now.
xyz = np.zeros(shape=(x.size, 3))
xyz[:, 0] = x.flatten()
xyz[:, 1] = y.flatten()
xyz[:, 2] = z.flatten()

# Generate a grid for the X and Y coordinates, change the number of points
# to your needs. Large grids can generate files that are too big for Gazebo, so
# be careful when choosing the resolution of your grid.
x_grid, y_grid = np.meshgrid(np.linspace(xyz[:, 0].min(), xyz[:, 0].max(), 300),
                             np.linspace(xyz[:, 1].min(), xyz[:, 1].max(), 200))

# Interpolate over the point cloud for our grid
z_grid = griddata(xyz[:, 0:2], xyz[:, 2], (x_grid, y_grid),
                  method='linear')

# Option to treat noise
#z_grid = gaussian_filter(z_grid, sigma=1)

# Show the resulting heightmap as an image
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111)
plt.imshow(z_grid)

# Flatten our interpolated data for triangulation
output = np.zeros(shape=(x_grid.size, 3))
output[:, 0] = x_grid.flatten()
output[:, 1] = y_grid.flatten()
output[:, 2] = z_grid.flatten()

# Triangulation of the interpolated data
tri = mtri.Triangulation(output[:, 0], output[:, 1])

# Show the resulting surface
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')
ax.plot_trisurf(tri, output[:, 2], cmap=plt.cm.CMRmap, shade=True, linewidth=0.1)
ax.axis('equal')

# Create the mesh object
seabed_mesh = mesh.Mesh(np.zeros(tri.triangles.shape[0], dtype=mesh.Mesh.dtype))

# Set the vectors
for i, f in enumerate(tri.triangles):
    for j in range(3):
        seabed_mesh.vectors[i][j] = output[f[j]]

# Store the seabed as a STL file
seabed_mesh.save('seabed.stl')

plt.show()
