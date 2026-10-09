Development
===========

Install For Development
-----------------------

.. code-block:: bash

   cd /home/popnut/remote_controller
   python -m pip install -e ".[docs]"

Build The C++ Server
--------------------

.. code-block:: bash

   cmake -S cpp_server -B cpp_server/build
   cmake --build cpp_server/build -j

Build The Documentation
-----------------------

.. code-block:: bash

   sphinx-build -M html docs/source docs/build

The generated HTML is:

.. code-block:: text

   docs/build/html/index.html

Documentation Rules
-------------------

This project uses pure reStructuredText for Sphinx documentation:

* Write source pages under ``docs/source/*.rst``.
* Do not commit ``docs/build/``.
* Keep user-facing API units explicit.
* Keep Python docstrings in Google style so ``sphinx.ext.napoleon`` can render
  them.

Useful Checks
-------------

.. code-block:: bash

   python -m py_compile src/remote_controller/RemoteControllerClient.py
   git diff --check
   cmake --build cpp_server/build
   sphinx-build -M html docs/source docs/build
