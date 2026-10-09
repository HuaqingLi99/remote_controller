UDP State Stream
================

The C++ server streams robot state to the Python client over UDP. UDP is used
only for state feedback; commands still go through XML-RPC.

Packet Layout
-------------

The UDP packet contains 20 doubles plus 2 integer state ids. By default,
``get_padded_data()`` uses all 22 values from each frame. Set
``sensor_size=20`` only if you want a q/dq/force-only history window.

.. list-table::
   :header-rows: 1

   * - Field
     - Size
     - Unit
     - Meaning
   * - q
     - 7
     - rad
     - joint position
   * - dq
     - 7
     - rad/s
     - joint velocity
   * - K_F_ext_hat
     - 6
     - Franka wrench estimate
     - estimated external wrench from robot state
   * - arm_state
     - 1
     - enum id
     - arm command lifecycle state
   * - gripper_state
     - 1
     - enum id
     - gripper command lifecycle state

Target Frequency
----------------

``udp_frequency_hz`` is a target frequency. During active robot motion, the
measured rate may be slightly lower because the server is also running the
control callback. UDP is best-effort, so consumers should check freshness.

Freshness Levels
----------------

The Python receiver classifies frame age as:

.. list-table::
   :header-rows: 1

   * - Level
     - Meaning
   * - fresh
     - newest frame is recent
   * - stale
     - frame is old but may still be accepted if ``allow_stale=True``
   * - expired
     - frame is too old for normal use
   * - too_old
     - frame should be rejected

Latest State
------------

Use this when you need the newest state:

.. code-block:: python

   state, info = client.get_latest_state()
   if state is not None:
       q = state["q"]
       dq = state["dq"]
       arm_state = state["arm_state"]
       gripper_state = state["gripper_state"]

History Window For VLA/Policy Inference
---------------------------------------

For VLA or policy inference, prefer:

.. code-block:: python

   data, meta = client.get_padded_data()

This returns a fixed-size flattened history window from the local
``DataBuffer``. For inference, keep the buffer small, for example:

.. code-block:: python

   client = RemoteControllerClient(
       "http://localhost:8008/RPC2",
       capacity=16,
       horizon_prev=7,
       sensor_size=22,
   )

Use larger capacities only for plotting or offline analysis.

Flush Old State
---------------

Before measuring a new motion, clear old local UDP packets:

.. code-block:: python

   client.empty_buffer()
   state = client.wait_for_first_udp(timeout=2.0)
