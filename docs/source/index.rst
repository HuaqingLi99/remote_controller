remote_controller
=================

Project Attribution
-------------------

This project was developed by Huaqing Li (go98niv) as part of his
Ingenieurpraxis, under the supervision of Yangsong Wu.

License
-------

The original code and documentation authored by Huaqing Li are licensed under
the MIT License; see the LICENSE file in the repository root. Third-party
materials retain their own licenses. The provenance and license of the bundled
Panda robot model assets must be verified before public distribution; see
THIRD_PARTY_NOTICES.md in the repository root.

Overview
--------

``remote_controller`` is a remote-control framework for a Franka Panda setup.
It includes a C++ XML-RPC server, a Python client package, UDP robot-state
streaming, gripper helpers, and Pinocchio-based kinematics utilities using the
packaged Panda URDF/assets.

.. warning::

   This project sends commands to a real robot. Start with small motions, low
   velocity limits, and a clear workspace. Keep the Franka desk/user stop
   reachable at all times.

What This Project Provides
--------------------------

* C++ XML-RPC server for arm and gripper commands.
* Python package named ``remote_controller``.
* UDP state stream containing ``q``, ``dq``, ``K_F_ext_hat``, ``arm_state``, and ``gripper_state``.
* Unified Python helpers for direct and queued motion commands.
* Panda URDF/assets packaged for Pinocchio kinematics.

Common Units
------------

* Joint position: radians.
* Joint velocity: radians/second.
* Cartesian translation: meters.
* Cartesian linear velocity: meters/second.
* Cartesian rotation: radians.
* Gripper width and tolerances: meters.
* Gripper force: newtons.

.. toctree::
   :maxdepth: 2
   :caption: User Guide

   installation
   quickstart
   safety
   examples
   udp
   kinematics
   rpc_api

.. toctree::
   :maxdepth: 2
   :caption: API Reference

   python_api

.. toctree::
   :maxdepth: 1
   :caption: Development

   development
