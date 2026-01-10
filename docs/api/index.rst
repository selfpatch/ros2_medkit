C++ API Reference
=================

This section contains the automatically generated API documentation for the ros2_medkit C++ codebase.

.. note::

   This documentation is generated from source code comments using Doxygen and Breathe.
   The Doxygen XML is generated during the documentation build process.

ros2_medkit_gateway
-------------------

The HTTP/REST gateway that exposes ROS 2 graph via SOVD-compatible API.

.. doxygennamespace:: ros2_medkit_gateway
   :outline:

GatewayNode
~~~~~~~~~~~

The main ROS 2 node that orchestrates the HTTP gateway.

.. doxygenclass:: ros2_medkit_gateway::GatewayNode
   :members:
   :outline:

RestServer
~~~~~~~~~~

HTTP server implementation using cpp-httplib.

.. doxygenclass:: ros2_medkit_gateway::RestServer
   :members:
   :outline:

DiscoveryManager
~~~~~~~~~~~~~~~~

Manages discovery of ROS 2 nodes, topics, services, and actions.

.. doxygenclass:: ros2_medkit_gateway::DiscoveryManager
   :members:
   :outline:

Data Models
~~~~~~~~~~~

.. doxygenstruct:: ros2_medkit_gateway::QosProfile
   :members:

.. doxygenstruct:: ros2_medkit_gateway::TopicEndpoint
   :members:

.. doxygenstruct:: ros2_medkit_gateway::TopicConnection
   :members:

ros2_medkit_fault_manager
-------------------------

Central fault storage and management node.

.. doxygennamespace:: ros2_medkit_fault_manager
   :outline:

FaultManagerNode
~~~~~~~~~~~~~~~~

The main ROS 2 node providing fault management services.

.. doxygenclass:: ros2_medkit_fault_manager::FaultManagerNode
   :members:
   :outline:

FaultStorage
~~~~~~~~~~~~

Abstract interface for fault storage backends.

.. doxygenclass:: ros2_medkit_fault_manager::FaultStorage
   :members:
   :outline:

InMemoryFaultStorage
~~~~~~~~~~~~~~~~~~~~

Thread-safe in-memory implementation of FaultStorage.

.. doxygenclass:: ros2_medkit_fault_manager::InMemoryFaultStorage
   :members:
   :outline:

ros2_medkit_fault_reporter
--------------------------

Client library for reporting faults to the fault manager.

.. doxygennamespace:: ros2_medkit_fault_reporter
   :outline:

FaultReporter
~~~~~~~~~~~~~

Main API for fault reporting from ROS 2 nodes.

.. doxygenclass:: ros2_medkit_fault_reporter::FaultReporter
   :members:
   :outline:

LocalFilter
~~~~~~~~~~~

Per-fault-code filtering with threshold and time window.

.. doxygenclass:: ros2_medkit_fault_reporter::LocalFilter
   :members:
   :outline:
