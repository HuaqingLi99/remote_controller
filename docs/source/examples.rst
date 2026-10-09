Examples
========

Examples live in:

.. code-block:: text

   examples/

Run them from an activated environment:

.. code-block:: bash

   cd /home/popnut/remote_controller
   source .venv/bin/activate
   cd examples

Basic Checks
------------

Recommended first tests:

.. code-block:: bash

   python test_kinematics.py
   python test_udp_idle.py

Motion Examples
---------------

.. code-block:: bash

   python test_moveJ_path.py
   python test_moveC_path.py
   python test_moveC_relative.py

Stop And Queue Behavior
-----------------------

.. code-block:: bash

   python test_stop_motion.py
   python test_moveJ_no_queue_denied.py
   python test_moveC_no_queue_denied.py

UDP Examples
------------

.. code-block:: bash

   python test_udp_idle.py
   python test_udp_moving.py

Gripper Example
---------------

.. code-block:: bash

   python test_gripper_queue.py

Plotting Example
----------------

``test_moveC_path_plot.py`` compares linear and Catmull-Rom Cartesian path
behavior and writes an image under ``examples/output/``. Install matplotlib if
you want to run this example:

.. code-block:: bash

   python -m pip install matplotlib
   python test_moveC_path_plot.py

Notes
-----

Motion examples should be run slowly and with a clear workspace. Start with
small radii and low velocity limits before trying larger paths.
