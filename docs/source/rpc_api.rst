RPC API
=======

The C++ server exposes XML-RPC methods. The Python client wraps these methods,
so most users should call ``RemoteControllerClient`` instead of calling XML-RPC
directly.

Result Codes
------------

.. list-table::
   :header-rows: 1

   * - Code
     - Meaning
   * - 0
     - OK
   * - -2
     - DENIED_MOVING
   * - -3
     - DENIED_ERROR
   * - -4
     - EXECUTION_FAILED

Arm State Values
----------------

.. list-table::
   :header-rows: 1

   * - Value
     - Meaning
   * - 0
     - IDLE
   * - 1
     - MOVING
   * - 2
     - ERROR

Gripper State Values
--------------------

.. list-table::
   :header-rows: 1

   * - Value
     - Meaning
   * - 0
     - IDLE
   * - 1
     - MOVING
   * - 2
     - ERROR
   * - 3
     - WIDTH_TOO_LARGE
   * - 4
     - HOLDING
   * - 5
     - OPEN_FAILED

Queue Semantics
---------------

Queue methods return after a task is accepted into the queue. They do not wait
for motion completion. No-queue methods execute immediately and return the
execution result code.

Joint Motion
------------

moveJ
~~~~~

Direct single-target joint motion.

Inputs:

* ``q_d``: 7 joint targets in radians
* ``stiffness``: 7x7 joint stiffness matrix
* ``vmax``: 7 joint velocity limits in rad/s
* ``acc_max``: 7 joint acceleration limits in rad/s^2

Returns direct execution result code.

moveJ_queue
~~~~~~~~~~~

Same inputs as ``moveJ``, but enqueues the command and returns an accept code.

moveJPath
~~~~~~~~~

Direct multi-waypoint joint path.

Inputs:

* ``waypoints``: Nx7 joint waypoints in radians
* ``samples_per_segment``: number of base replay samples per segment
* ``path_mode``: ``linear`` or ``catmull_rom``
* ``stiffness``: 7x7 joint stiffness matrix
* ``vmax``: 7 joint velocity limits in rad/s
* ``acc_max``: accepted for API compatibility; path replay currently checks
  velocity limits and uses ``samples_per_segment`` plus the runtime motion
  slowdown factor for timing

Returns direct execution result code.

moveJPath_queue
~~~~~~~~~~~~~~~

Same inputs as ``moveJPath``, but enqueues the path and returns an accept code.

Cartesian Motion
----------------

moveCartesian
~~~~~~~~~~~~~

Direct single-target Cartesian impedance motion.

Inputs:

* ``T_d``: 16-element row-major 4x4 target pose in base/world frame
* ``stiffness``: 6x6 Cartesian stiffness matrix
* ``vmax_linear``: linear velocity limit in m/s
* ``acc_max_linear``: linear acceleration limit in m/s^2
* ``vmax_angular``: angular velocity limit in rad/s
* ``acc_max_angular``: angular acceleration limit in rad/s^2
* ``nullspace_stiffness``: joint nullspace stiffness

Returns direct execution result code.

moveCartesian_queue
~~~~~~~~~~~~~~~~~~~

Same inputs as ``moveCartesian``, but enqueues the command and returns an
accept code.

moveCartesianPath
~~~~~~~~~~~~~~~~~

Direct multi-waypoint Cartesian path.

Inputs:

* ``waypoints``: Nx16 row-major 4x4 poses in base/world frame
* ``samples_per_segment``: number of base replay samples per segment
* ``path_mode``: ``linear`` or ``catmull_rom``
* ``stiffness``: 6x6 Cartesian stiffness matrix
* ``vmax_linear``: linear velocity limit in m/s
* ``acc_max_linear``: accepted for API compatibility; not used by current path
  replay timing
* ``vmax_angular``: angular velocity limit in rad/s
* ``acc_max_angular``: accepted for API compatibility; not used by current path
  replay timing
* ``nullspace_stiffness``: joint nullspace stiffness

Returns direct execution result code.

moveCartesianPath_queue
~~~~~~~~~~~~~~~~~~~~~~~

Same inputs as ``moveCartesianPath``, but enqueues the path and returns an
accept code.

Gripper Commands
----------------

graspO
~~~~~~

Direct gripper grasp command.

Inputs:

* ``width``: target grasp width in meters
* ``speed``: gripper speed in m/s
* ``force``: grasping force in newtons
* ``epsilon_inner``: inner tolerance in meters
* ``epsilon_outer``: outer tolerance in meters

Returns direct execution result code.

graspO_queue
~~~~~~~~~~~~

Same inputs as ``graspO``, but enqueues the command and returns an accept code.

gripperRelease
~~~~~~~~~~~~~~

Inputs:

* ``speed``: opening speed in m/s

Returns direct execution result code.

gripperRelease_queue
~~~~~~~~~~~~~~~~~~~~

Same inputs as ``gripperRelease``, but enqueues the command and returns an
accept code.

gripperHome
~~~~~~~~~~~

Homes the gripper.

Inputs: none.

Returns direct execution result code.

gripperHome_queue
~~~~~~~~~~~~~~~~~

Same inputs as ``gripperHome``, but enqueues the command and returns an accept
code.

State And Session Methods
-------------------------

getArmState
~~~~~~~~~~~

Inputs: none.

Returns the integer arm state enum value.

getGripperState
~~~~~~~~~~~~~~~

Inputs: none.

Returns the integer gripper state enum value.

getGripperWidth
~~~~~~~~~~~~~~~

Inputs: none.

Returns the cached gripper width in meters.

initSession
~~~~~~~~~~~

Initializes server-side UDP target, UDP frequency, and collision thresholds.

Inputs:

* ``udp_ip``: client UDP receive IP
* ``udp_port``: client UDP receive port
* ``udp_frequency_hz``: requested UDP streaming frequency
* ``lower_torque``: 7 lower torque thresholds
* ``upper_torque``: 7 upper torque thresholds
* ``lower_force``: 6 lower Cartesian force thresholds
* ``upper_force``: 6 upper Cartesian force thresholds

Returns ``OK`` on success.

recoverSystem
~~~~~~~~~~~~~

Inputs: none.

Attempts to recover arm and gripper manager state. Returns a result code.

stopArmMotion
~~~~~~~~~~~~~

Inputs: none.

Requests the active arm controller to stop and clears queued arm commands.
Returns a result code.

setSlowdownFactor
~~~~~~~~~~~~~~~~~

Inputs:

* ``slowdown_factor``: runtime arm-motion replay multiplier. ``1.0`` is the
  original speed; ``10.0`` advances motion references about 10 times slower.

Can be called while ``moveJ``, ``moveCartesian``, ``moveJPath``, or
``moveCartesianPath`` is moving. Returns ``OK`` on success. From a single Python
thread, enqueue the motion first so the motion RPC returns before sending
runtime slowdown updates.

getSlowdownFactor
~~~~~~~~~~~~~~~~~

Inputs: none.

Returns the current runtime arm-motion slowdown factor.

setIdleStatePollFrequency
~~~~~~~~~~~~~~~~~~~~~~~~~

Inputs:

* ``frequency_hz``: idle state polling frequency in Hz

Returns ``OK`` on success.

XML-RPC Type Note
-----------------

The current C++ parsers expect numeric arrays as XML-RPC doubles. The Python
client handles this for normal use. If calling XML-RPC manually, send floats
rather than integer literals in numeric arrays.
