.. _description:

Description of the modules
==========================

In the sections below you can find a detailed description of the theory behind the algorithms implemented in the Gazebo plugins developed for the
UUV Simulator.

Underwater object plugin
------------------------

This plugin is the core of the simulation of underwater vehicles. It contains an implementation of Fossen's equation of motion for underwater
vehicles (see :cite:`Fossen_2011` or :cite:`Fossen_2011a` for more details). For underwater robots, oarine robotics in general, the SNAME
:cite:`SNAME` convention is used to represent poses and velocities using the `North East Down (NED) <https://en.wikipedia.org/wiki/North_east_down>`_
reference frame convention.

Theoretical background
^^^^^^^^^^^^^^^^^^^^^^

The nomenclature for position, orientation and velocity vectors can be seen in the table below :cite:`Antonelli_2008b`.

+---------------------------+-------+--------------------+------------+--------------------------+
|                           |       | Forces and moments | Velocities | Position and orientation |
+===========================+=======+====================+============+==========================+
| Motion in the x-direction | Surge | X                  | u          | x                        |
+---------------------------+-------+--------------------+------------+--------------------------+
| Motion in the y-direction | Sway  | Y                  | v          | y                        |
+---------------------------+-------+--------------------+------------+--------------------------+
| Motion in the z-direction | Heave | Z                  | w          | z                        |
+---------------------------+-------+--------------------+------------+--------------------------+
| Rotation about the x-axis | Roll  | K                  | p          | :math:`\phi`             |
+---------------------------+-------+--------------------+------------+--------------------------+
| Rotation about the y-axis | Pitch | M                  | q          | :math:`\theta`           |
+---------------------------+-------+--------------------+------------+--------------------------+
| Rotation about the z-axis | Yaw   | N                  | r          | :math:`\psi`             |
+---------------------------+-------+--------------------+------------+--------------------------+

The pose of the vehicle is represented by the vector :math:`\boldsymbol{\eta} = (x, y, z, \phi, \theta, \psi)^T` and the
velocity (represented in the BODY frame) is written as :math:`\boldsymbol{\nu} = (u, v, w, p, q, r)^T`. The transformation
between BODY and NED frame is achieved through a Jacobian matrix as follows (see :cite:`Fossen_2011a` for more details).

.. math::

  \boldsymbol{\dot{\eta}} = \mathbf{J} \boldsymbol{\nu}

To describe the motion of an object in the water, the influence of the fluid surrounding the object has to be considered. The most common
method used is Fossen's equation of motion, where you consider:

* **Added-mass:** with the acceleration of a body in a fluid, the surrounding volume of fluid will be accelerated during the motion.
* **Hydrodynamic damping:** damping has several sources of contribution, such as skin friction, viscous damping. It is usually divided into linear and quadratic damping for simplification.
* **Hydrostatic forces:** for an object in the water, buoyancy is a new force component has to be taken into account. It is dependent on the object's volume and the fluid's density. The resulting force between buoyancy and gravitational forces is known as *restoring* force. When an object has *positive buoyancy* for instance, it means that the buoyany force is greater than the gravitational force, and the object will float unless other forces are acting on the body.

Added mass and some damping contributions are in reality frequency dependent, but for underwater robotics it is often considered that these coefficients
are constant. The equation of motion :cite:`Fossen_2011` can be written as follows

.. math::

  (\mathbf{M}_{RB} + \mathbf{M}_{A}) \boldsymbol{\dot{\nu}} + (\mathbf{C}_{RB}(\boldsymbol{\nu}) + \mathbf{C}_{A}(\boldsymbol{\nu})) \boldsymbol{\nu} + \mathbf{D}(\boldsymbol{\nu}) \boldsymbol{\nu} + \mathbf{g}(\boldsymbol{\eta}) = \boldsymbol{\tau}

In this equation,

* :math:`\mathbf{M}_{RB}` and :math:`\mathbf{C}_{RB}` represent respectively the rigid-body inertial and Coriolis and centripetal matrices.
* :math:`\mathbf{M}_{A}` is the constant added-mass matrix and :math:`\mathbf{C}_A(\boldsymbol{\nu})` the resulting added-mass Coriolis and centripetal matrix.
* :math:`\mathbf{D}(\boldsymbol{\nu})` represents linear and quadratic damping components.
* :math:`\mathbf{g}(\boldsymbol{\eta})` is the vector of restoring forces.
* :math:`\mathbf{\tau}` is the generalized force vector for external and control forces and moments.

Implementation aspects
^^^^^^^^^^^^^^^^^^^^^^

`Gazebo <http://gazebosim.org/>`_ offers a wide range of physics engines to rigid-body dynamics simulation already and an API that can be used
to write plugins to extend the functionalities of the simulator. In general, packages that offer some extension on the dynamics simulation for
a certain type of entity have to implement `model plugins <http://gazebosim.org/tutorials?tut=plugins_model&cat=write_plugin>`_ that will
be called by Gazebo at each iteration to generate the necessary forces and torques.

Since the rigid-body dynamics and contact forces are simulated by Gazebo already, what is left to be computed are the hydrodynamic and hydrostatic
forces and moments described in the previous section, namely

.. math::

  \boldsymbol{\tau}_{h} = - \mathbf{M}_A \boldsymbol{\dot{\nu}} - \mathbf{C}_{A}(\boldsymbol{\nu}) \boldsymbol{\nu} - \mathbf{D}(\boldsymbol{\nu}) \boldsymbol{\nu} - \mathbf{g}_b(\boldsymbol{\eta})

where :math:`\mathbf{g}_b(\boldsymbol{\eta})` stands for the buoyancy force vector represented in the BODY frame, since the gravitational force
is already computed by Gazebo.

**Added-mass forces and moments**

**Damping forces and moments**

The damping matrices are implemented here following the equation described by :cite:`Refsnes_2007`.

**Buoyancy forces and moments**

The buoyancy force is the easiest implementation of this list. There are two options that can be setup in order to compute the buoyancy force. For
underwater vehicles, once the center of buoyancy :math:`\mathbf{r}_b`, fluid density, volume are provided, the buoyancy force vector is generated
and applied on the vehicle's center of buoyancy with respect to the Gazebo's inertial frame as

.. math::

  \mathbf{f}_b = [0, 0, \rho V g]^T

.. note::

  The :math:`\mathbf{f}_b` vector is generated in the plugin with respect to Gazebo's inertial frame, which follows the ENU (East North Up) convention,
  not NED.

Plugin's URDF description
^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: samples/plugin_description/underwater_object_plugin_fossen.xacro
  :language: xml
  :linenos:

Thruster plugin
---------------

Theoretical background
^^^^^^^^^^^^^^^^^^^^^^

The computation of the dynamics of the thrust force generated by the thruster is a very complex task. Many sources have already addressed it,
such as :cite:`Bessa_2006` and :cite:`Yoerger_1990`, but it is difficult to determine one single method to describe it, and the parametrization
of the equation, regardless which one you choose, will need the identification of the equation's parameters through `Bollard tests <https://en.wikipedia.org/wiki/Bollard_pull>`_.

The plugin implemented can be described by two modules:

* the **dynamics module** describes the dynamics of the thruster's rotor
* and the **conversion function**, which describes the steady-state relationship between the rotor's angular velocity and the thrust force output

These two models can be configured using the options below according to the model of thruster being used in the vehicle. The thruster plugin

Fin plugin
----------

Plugin's URDF description
^^^^^^^^^^^^^^^^^^^^^^^^^

Thruster manager
----------------
