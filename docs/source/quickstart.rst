Quick Start
===========

This page shows the shortest useful flow: start the server, initialize the
Python client, receive UDP state, and compute the current TCP pose.

Start The Server
----------------

From the repository root:

.. code-block:: bash

   ./run_server.sh

This reads ``config/remote_controller.env`` and starts the C++ XML-RPC server.
The server prints the current robot and gripper state when it connects.

Run A First Python Example
--------------------------

In another terminal:

.. code-block:: bash

   cd /home/popnut/remote_controller
   source .venv/bin/activate
   cd examples
   python test_kinematics.py

Minimal Python Client
---------------------

.. code-block:: python

   from remote_controller import RemoteControllerClient, RobotModel

   SERVER_URL = "http://localhost:8008/RPC2"

   client = RemoteControllerClient(SERVER_URL)
   result = client.init(
       udp_ip="127.0.0.1",
       udp_port=9000,
       udp_frequency_hz=500,
       recover_before_init=True,
   )

   client.print_rpc_result(result, "init")

   robot_model = RobotModel()
   T_tcp = client.get_current_tcp_pose(robot_model, flush=True)
   print(T_tcp)

Try A Small Relative Cartesian Move
-----------------------------------

Use a very small first motion, for example 3 cm along the TCP/tool ``z`` axis:

.. code-block:: python

   from remote_controller import RemoteControllerClient, RobotModel

   client = RemoteControllerClient("http://localhost:8008/RPC2")
   robot_model = RobotModel()

   result = client.init(
       udp_ip="127.0.0.1",
       udp_port=9000,
       udp_frequency_hz=500,
       recover_before_init=True,
   )
   client.print_rpc_result(result, "init")

   result = client.movecart_relative(
       robot_model,
       dz=0.03,
       vmax=0.03,
       acc_max=0.05,
       reference_frame="tool",
       queue=False,
   )
   client.print_rpc_result(result, "moveC relative")

   client.close()

Recommended First Examples
--------------------------

Run these before larger motions:

.. code-block:: bash

   python test_kinematics.py
   python test_udp_idle.py
   python test_moveC_relative.py
