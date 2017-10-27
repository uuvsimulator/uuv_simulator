.. UUV Simulator documentation master file, created by
   sphinx-quickstart on Thu Sep 28 11:58:00 2017.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to UUV Simulator's documentation!
=========================================

The UUV Simulator is a package containing the implementation of Gazebo plugins
and ROS nodes necessary for the simulation of unmanned underwater vehicles, such
as ROVs (remotely operated vehicles) and AUVs (autonomous underwater vehicles).

To send questions and/or issues, please refer to the `repository's issues page <https://github.com/uuvsimulator/uuv_simulator/issues>`_.

Click `here <https://github.com/uuvsimulator/uuv_simulator>`_ to access the
GitHub repository.

Contents
--------

.. toctree::
  :glob:
  :maxdepth: 2

  installation
  quick_start
  tutorials/index
  vehicles
  faq
  license
  .. description


Reference
---------

If you wish to use the UUV Simulator in a research project, please cite our paper ::

  @inproceedings{Manhaes_2016,
  	doi = {10.1109/oceans.2016.7761080},
  	url = {https://doi.org/10.1109%2Foceans.2016.7761080},
  	year = 2016,
  	month = {sep},
  	publisher = {{IEEE}},
  	author = {Musa Morena Marcusso Manh{\~{a}}es and Sebastian A. Scherer and Martin Voss and Luiz Ricardo Douat and Thomas Rauschenbach},
  	title = {{UUV} Simulator: A Gazebo-based package for underwater intervention and multi-robot simulation},
  	booktitle = {{OCEANS} 2016 {MTS}/{IEEE} Monterey}
  }

Purpose of the project
----------------------

This software is a research prototype, originally developed for the EU ECSEL
Project 662107 `SWARMs <http://swarms.eu/>`_.

The software is not ready for production use. It has neither been developed nor
tested for a specific use case. However, the license conditions of the applicable
Open Source licenses allow you to adapt the software to your needs. Before using
it in a safety relevant setting, make sure that the software fulfills your
requirements and adjust it according to any applicable safety standards (e.g.
ISO 26262).

License
-------

UUV Simulator is open-sourced under the Apache-2.0 license. See the
`LICENSE <https://github.com/uuvsimulator/uuv_simulator/blob/master/LICENSE>`_
file for details.

For a list of other open source components included in UUV Simulator, see the
file `3rd-party-licenses.txt <https://github.com/uuvsimulator/uuv_simulator/blob/master/3rd-party-licenses.txt>`_.

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

Useful bibliography
===================

.. bibliography:: refs.bib
  :all:
  :style: plain
