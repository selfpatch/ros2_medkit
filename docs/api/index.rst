C++ API Reference
=================

This section contains the automatically generated API documentation for the ros2_medkit C++ codebase.

.. note::

   This documentation is generated from source code comments using Doxygen and Breathe.
   The Doxygen XML is generated during the documentation build process.

ros2_medkit_gateway
-------------------

The HTTP/REST gateway that exposes ROS 2 graph via SOVD-compatible API.

Classes
~~~~~~~

.. doxygenclass:: ros2_medkit_gateway::GatewayNode
   :members:

.. doxygenclass:: ros2_medkit_gateway::DiscoveryManager
   :members:

.. doxygenclass:: ros2_medkit_gateway::DataAccessManager
   :members:

.. doxygenclass:: ros2_medkit_gateway::OperationManager
   :members:

.. doxygenclass:: ros2_medkit_gateway::ConfigurationManager
   :members:

Data Models
~~~~~~~~~~~

.. doxygenstruct:: ros2_medkit_gateway::QosProfile
   :members:

.. doxygenstruct:: ros2_medkit_gateway::TopicEndpoint
   :members:

.. doxygenstruct:: ros2_medkit_gateway::TopicConnection
   :members:

.. doxygenstruct:: ros2_medkit_gateway::Area
   :members:

.. doxygenstruct:: ros2_medkit_gateway::Component
   :members:

ros2_medkit_fault_manager
-------------------------

Central fault storage and management node.

.. doxygenclass:: ros2_medkit_fault_manager::FaultManagerNode
   :members:

.. doxygenclass:: ros2_medkit_fault_manager::FaultStorage
   :members:

.. doxygenclass:: ros2_medkit_fault_manager::InMemoryFaultStorage
   :members:

ros2_medkit_fault_reporter
--------------------------

Client library for reporting faults to the fault manager.

.. doxygenclass:: ros2_medkit_fault_reporter::FaultReporter
   :members:

.. doxygenclass:: ros2_medkit_fault_reporter::LocalFilter
   :members:

ros2_medkit_diagnostic_bridge
-----------------------------

Bridge node that converts ROS 2 /diagnostics messages to FaultManager faults.

.. doxygenclass:: ros2_medkit_diagnostic_bridge::DiagnosticBridgeNode
   :members:

ros2_medkit_serialization
-------------------------

Runtime JSON ↔ ROS 2 message serialization library.

.. doxygenclass:: ros2_medkit_serialization::JsonSerializer
   :members:

.. doxygenclass:: ros2_medkit_serialization::TypeCache
   :members:

.. doxygenclass:: ros2_medkit_serialization::SerializationError
   :members:

.. doxygenclass:: ros2_medkit_serialization::TypeNotFoundError
   :members:

.. doxygenclass:: ros2_medkit_serialization::JsonConversionError
   :members:
