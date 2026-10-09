Kinematics
==========

The Python package includes Panda URDF assets used by ``RobotModel``. The
default URDF is packaged under ``src/remote_controller/assets`` and uses mesh
paths that resolve relative to that package asset root.

Default TCP Frame
-----------------

.. code-block:: text

   panda_hand_tcp

List Frames
-----------

.. code-block:: python

   from remote_controller import RobotModel

   robot_model = RobotModel()
   frames = robot_model.list_frames()
   print("panda_hand_tcp" in frames)

Compute TCP Pose
----------------

.. code-block:: python

   from remote_controller import RemoteControllerClient, RobotModel

   client = RemoteControllerClient("http://localhost:8008/RPC2")
   robot_model = RobotModel()

   result = client.init(
       udp_ip="127.0.0.1",
       udp_port=9000,
       udp_frequency_hz=500,
   )

   T_tcp = client.get_current_tcp_pose(robot_model, flush=True)
   print(T_tcp)

Basic Terms
-----------

``RobotModel.get_all_basic_terms`` returns:

.. list-table::
   :header-rows: 1

   * - Key
     - Meaning
   * - q
     - validated joint position
   * - dq
     - validated joint velocity
   * - ee_translation
     - frame translation
   * - ee_rotation
     - frame rotation matrix
   * - ee_T
     - 4x4 frame pose
   * - J
     - frame Jacobian
   * - M
     - joint-space mass matrix
   * - g
     - generalized gravity vector
   * - c
     - velocity-dependent generalized term

Frame Fallback
--------------

Some old examples requested ``panda_hand``. If that frame is absent, the helper
falls back to ``panda_link8``. New code should normally use the client's
``default_frame_name``, which defaults to ``panda_hand_tcp``.
