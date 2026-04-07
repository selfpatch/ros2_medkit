ros2_medkit_msgs
=================

This section contains design documentation for the ros2_medkit_msgs package.

Overview
--------

The ``ros2_medkit_msgs`` package defines the ROS 2 message and service interfaces
used across the ros2_medkit system. It has no runtime code - only ``.msg`` and
``.srv`` definitions that are compiled by ``rosidl`` into C++ and Python bindings.

Message Definitions
-------------------

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Message
     - Purpose
   * - ``Fault.msg``
     - Core fault representation: code, severity, status, entity, timestamps, description
   * - ``FaultEvent.msg``
     - Fault lifecycle event published on SSE streams (fault + event type)
   * - ``Snapshot.msg``
     - Rosbag snapshot metadata: entity, fault code, timestamps, bag path
   * - ``ClusterInfo.msg``
     - Fault cluster information for correlation engine
   * - ``EnvironmentData.msg``
     - Environment context attached to faults (system state at time of fault)
   * - ``ExtendedDataRecords.msg``
     - SOVD-aligned extended data records for fault snapshots
   * - ``MutedFaultInfo.msg``
     - Muted fault tracking (code, entity, mute reason, expiry)
   * - ``MedkitDiscoveryHint.msg``
     - Beacon discovery hint published by nodes for the topic beacon plugin
   * - ``EntityInfo.msg``
     - SOVD entity representation for service interface: ID, type, name, parent, FQN, capabilities

Service Definitions
-------------------

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Service
     - Purpose
   * - ``ReportFault.srv``
     - Report a fault to the fault manager (used by fault_reporter library)
   * - ``GetFault.srv``
     - Retrieve details of a single fault by ID
   * - ``ListFaults.srv``
     - List all faults with optional filtering
   * - ``ListFaultsForEntity.srv``
     - List faults scoped to a specific entity
   * - ``ClearFault.srv``
     - Clear/delete a specific fault
   * - ``GetSnapshots.srv``
     - Retrieve rosbag snapshot metadata
   * - ``GetRosbag.srv``
     - Retrieve rosbag file path for download
   * - ``ListRosbags.srv``
     - List all available rosbag snapshots
   * - ``ListEntities.srv``
     - List entities with optional type filtering (for SOVD service interface)
   * - ``GetEntityData.srv``
     - Get data items for a specific entity (for SOVD service interface)
   * - ``GetCapabilities.srv``
     - Get capabilities/resource collections for a specific entity (for SOVD service interface)

Design Decisions
----------------

Separate Package
~~~~~~~~~~~~~~~~

Messages live in a dedicated package so that lightweight clients (such as
``ros2_medkit_fault_reporter``) can depend on the interface definitions without
pulling in the full gateway or fault manager implementations.

SOVD Alignment
~~~~~~~~~~~~~~

Fault severity levels and status strings follow the SOVD specification where
applicable. Constants are defined directly in ``Fault.msg`` (e.g.,
``SEVERITY_INFO``, ``STATUS_CONFIRMED``) so that all consumers share the same
canonical values.
