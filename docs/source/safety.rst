Safety
======

This project sends commands to a real Franka robot. Treat every example as a
real motion command, even if the motion looks small in code.

Before Running Motion Examples
------------------------------

* Keep the workspace clear.
* Start with very small Cartesian distances.
* Use low velocity and acceleration limits.
* Keep the Franka desk/user stop reachable.
* Do not run unattended.
* Check that all Cartesian distances are in meters.
* Check that the robot is not near joint limits or self-collision.
* Do not queue many commands until stop/recovery behavior has been tested.

Unit Examples
-------------

.. code-block:: text

   0.03 m = 3 cm
   0.10 m = 10 cm

Frame Convention
----------------

Cartesian poses sent to the C++ server are Franka ``O_T_EE`` style poses:

* Translation is expressed in the robot base/world frame.
* A flattened pose has 16 values from a row-major 4x4 matrix in the Python
  client.
* ``movecart_relative(reference_frame="tool")`` applies the relative transform
  in the current TCP/tool frame.
* ``movecart_relative(reference_frame="world")`` applies the relative transform
  in the base/world frame.

Queue Behavior
--------------

Queued commands return when accepted into the queue, not when the motion has
finished. Use state helpers such as:

.. code-block:: python

   client.wait_until_arm_idle_ok(timeout=20.0)
   client.wait_until_gripper_moving_finished(timeout=20.0)

Stop And Recovery
-----------------

``stopArmMotion`` requests the active arm controller to stop and clears queued
arm commands. If the robot enters an error state, use ``recoverSystem`` only
after understanding the cause of the error and making the workspace safe.
