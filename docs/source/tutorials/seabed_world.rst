.. _seabed_world:

Making a simple seabed world model
==================================

For the following tutorial, all the files can also be found in the
`uuv_tutorial_seabed_world <https://github.com/uuvsimulator/uuv_simulator/tree/master/uuv_tutorials/uuv_tutorial_seabed_world>`_ package.

The seabed can be critical in simulation scenarios where the objective is, for example, to use UUVs for bathymetric mapping or just to make
the scenario look more realistic.

Gazebo already has a feature to generate heightmaps from grayscale images (see the video below). This is a very quick way to
setup a heightmap, but it can happen that its complexity might make the simulation run slower when trying to interact with it.

.. raw:: html

  <iframe src="https://player.vimeo.com/video/58409707" width="640" height="360" frameborder="0" webkitallowfullscreen mozallowfullscreen allowfullscreen></iframe>
  <p><a href="https://vimeo.com/58409707">Gazebo: Heightmap Tutorial</a> from <a href="https://vimeo.com/osrfoundation">OSRF</a> on <a href="https://vimeo.com">Vimeo</a>.</p>

Another option is to take an existent point cloud of a seabed and convert it into a mesh that can be imported in Gazebo like any other model.

Creating the mesh file
----------------------

Measurement data can be sparse and have outliers that need to be removed before generating the mesh. The script below is an example on how the measurement
data can be interpolated into a grid and later converted into an STL file. For this example, we will use the test surface available in the **matplotlib**
package, but you should replace it with your own point cloud data. You will also need the packages **numpy**, **scipy** and **numpy-stl**.

To install the necessary packages, you can use **pip** ::

  sudo apt-get install python-pip
  sudo pip install numpy scipy matplotlib numpy-stl

To generate the mesh, change the code below to your needs

.. code-block:: python

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

  # Point clouds may not come in nice grids and can be sparse,
  # so let's make it a (N, 3) matrix just to show how it can be done.
  # If you have outliers or noise, you should treat those values now.
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

Below you can see the resulting heightmap as an image
