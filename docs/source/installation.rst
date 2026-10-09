Installation
============

Repository Layout
-----------------

The important folders are:

.. code-block:: text

   remote_controller/
   ├── pyproject.toml
   ├── run_server.sh
   ├── config/
   ├── src/remote_controller/
   ├── examples/
   ├── cpp_server/
   └── docs/

Python Environment
------------------

On Ubuntu, install ``venv`` if it is missing:

.. code-block:: bash

   sudo apt update
   sudo apt install python3.10-venv

Create and activate a virtual environment from the repository root:

.. code-block:: bash

   cd /home/popnut/remote_controller
   python3 -m venv .venv
   source .venv/bin/activate

Install the Python package in editable mode:

.. code-block:: bash

   python -m pip install -U pip setuptools wheel
   python -m pip install -e .

After this, examples can import:

.. code-block:: python

   from remote_controller import RemoteControllerClient, RobotModel

Documentation Dependencies
--------------------------

Install documentation dependencies only when you want to build docs:

.. code-block:: bash

   python -m pip install -e ".[docs]"

C++ Server Dependencies
-----------------------

The C++ server needs:

* CMake
* a C++ compiler
* Eigen3
* xmlrpc-c
* libfranka

On Ubuntu, the non-libfranka packages are commonly installed with apt:

.. code-block:: bash

   sudo apt install build-essential cmake libeigen3-dev libxmlrpc-c++8-dev

Install libfranka using the method recommended for your Franka setup.

Build The C++ Server
--------------------

From the repository root:

.. code-block:: bash

   cd cpp_server
   cmake -S . -B build
   cmake --build build -j

The server binary should be:

.. code-block:: text

   cpp_server/build/remote_controller_server

Configure Robot IP And Port
---------------------------

Create a local environment file:

.. code-block:: bash

   cp config/remote_controller.example.env config/remote_controller.env

Edit:

.. code-block:: text

   config/remote_controller.env

Example:

.. code-block:: bash

   ROBOT_IP=192.168.3.100
   GRIPPER_IP=192.168.3.100
   XMLRPC_PORT=8008
   SERVER_BINARY=cpp_server/build/remote_controller_server

``config/remote_controller.env`` is local machine configuration and should not
be committed.
