API Reference
=============

This section contains API documentation for ros2_medkit.

.. toctree::
   :maxdepth: 2

   rest
   locking
   warning_codes
   messages
   cpp

REST API
--------

:doc:`rest`
   Complete REST API reference with request/response schemas, error codes,
   and usage examples.

:doc:`locking`
   SOVD resource locking API - acquire, extend, release locks on components
   and apps with scoped access control and automatic expiry.

:doc:`warning_codes`
   Stable aggregation warning codes surfaced via the ``/health.warnings``
   x-medkit extension, with remediation guidance.

Message Definitions
-------------------

:doc:`messages`
   ROS 2 message and service interfaces for fault reporting, querying,
   and event notifications.

C++ API
-------

:doc:`cpp`
   Doxygen-generated C++ class reference for extending ros2_medkit.
