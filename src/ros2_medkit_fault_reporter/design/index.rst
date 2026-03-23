ros2_medkit_fault_reporter
==========================

This section contains design documentation for the ros2_medkit_fault_reporter library.

Overview
--------

The FaultReporter library provides a simple, reusable API for ROS 2 nodes to report
faults to the central ``ros2_medkit_fault_manager``. It includes optional local
filtering to reduce noise from repeated fault occurrences.

Architecture
------------

The following diagram shows the relationships between the main components.

.. plantuml::
   :caption: ROS 2 Medkit Fault Reporter Class Architecture

   @startuml ros2_medkit_fault_reporter_architecture

   skinparam linetype ortho
   skinparam classAttributeIconSize 0

   title ROS 2 Medkit Fault Reporter - Class Architecture

   package "ROS 2 Framework" {
       class "rclcpp::Node" {
           +create_client()
           +get_logger()
           +declare_parameter()
           +get_parameter()
       }
   }

   package "ros2_medkit_msgs" {
       class "srv::ReportFault" {
           +Request: fault_code, severity, description, source_id
           +Response: success, message
       }
   }

   package "ros2_medkit_fault_reporter" {

       class FaultReporter {
           - node_: rclcpp::Node::SharedPtr
           - source_id_: string
           - client_: ServiceClient<ReportFault>
           - filter_: LocalFilter
           - logger_: rclcpp::Logger
           --
           + FaultReporter(node, source_id, service_name)
           + report(fault_code, severity, description): void
           + is_service_ready(): bool
           + filter(): const LocalFilter&
           --
           - load_parameters(): void
           - send_report(fault_code, severity, description): void
       }

       class LocalFilter {
           - config_: FilterConfig
           - trackers_: map<string, FaultTracker>
           - mutex_: std::mutex
           --
           + LocalFilter(config)
           + should_forward(fault_code, severity): bool
           + reset(fault_code): void
           + reset_all(): void
           + config(): const FilterConfig&
           + set_config(config): void
           --
           - cleanup_expired(tracker, now): void
       }

       class FilterConfig <<struct>> {
           + enabled: bool = true
           + default_threshold: int = 3
           + default_window_sec: double = 10.0
           + bypass_severity: uint8 = 2
       }

       class FaultTracker <<struct>> {
           + timestamps: vector<time_point>
       }
   }

   package "ros2_medkit_fault_manager" {
       class FaultManagerNode {
           ~/report_fault service
       }
   }

   ' Relationships
   FaultReporter --> "rclcpp::Node" : uses
   FaultReporter *-down-> LocalFilter : owns
   LocalFilter --> FilterConfig : configured by
   LocalFilter o--> FaultTracker : per fault_code
   FaultReporter ..> "srv::ReportFault" : calls
   FaultReporter ..> FaultManagerNode : calls service

   @enduml

Main Components
---------------

1. **FaultReporter** - The main public API for fault reporting

   - Takes a ``rclcpp::Node::SharedPtr`` and ``source_id`` in constructor
   - Creates a service client to ``/fault_manager/report_fault``
   - Loads filter configuration from ROS parameters
   - Provides simple ``report(fault_code, severity, description)`` method
   - Integrates ``LocalFilter`` to reduce noise from repeated faults
   - Fire-and-forget service calls (non-blocking)

2. **LocalFilter** - Per-fault-code filtering with threshold and time window

   - Tracks fault occurrences per ``fault_code``
   - Only forwards to FaultManager when threshold is met within time window
   - High-severity faults (ERROR, CRITICAL) bypass filtering by default
   - Thread-safe: protected by mutex for concurrent access
   - Configurable via ``FilterConfig``

3. **FilterConfig** - Configuration for local filtering behavior

   - ``enabled``: Enable/disable filtering (default: true)
   - ``default_threshold``: Number of reports before forwarding (default: 3)
   - ``default_window_sec``: Time window in seconds (default: 10.0)
   - ``bypass_severity``: Severity level that bypasses filtering (default: ERROR=2)

4. **FaultTracker** - Internal state for per-fault-code tracking

   - Maintains vector of timestamps for recent reports
   - Expired timestamps are cleaned up on each ``should_forward()`` call

Usage
-----

Basic Example
~~~~~~~~~~~~~

.. code-block:: cpp

   #include "ros2_medkit_fault_reporter/fault_reporter.hpp"

   class MyNode : public rclcpp::Node {
    public:
     MyNode() : Node("my_node") {
       // Create reporter after node is fully constructed
       reporter_ = std::make_unique<ros2_medkit_fault_reporter::FaultReporter>(
           shared_from_this(), get_fully_qualified_name());
     }

     void check_sensor() {
       if (sensor_error_detected()) {
         reporter_->report("SENSOR_FAILURE",
                           ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR,
                           "Sensor communication timeout");
       }
     }

    private:
     std::unique_ptr<ros2_medkit_fault_reporter::FaultReporter> reporter_;
   };

Configuration
~~~~~~~~~~~~~

Parameters are loaded from the node's parameter server:

.. code-block:: yaml

   my_node:
     ros__parameters:
       fault_reporter:
         local_filtering:
           enabled: true
           default_threshold: 3
           default_window_sec: 10.0
           bypass_severity: 2

Design Decisions
----------------

Local Filtering Rationale
~~~~~~~~~~~~~~~~~~~~~~~~~

Local filtering is enabled by default to address common scenarios where sensors
or subsystems may produce repeated fault reports in quick succession. Without
filtering, this could flood the central FaultManager and obscure other issues.

The default threshold of 3 reports within 10 seconds balances:

- **Noise reduction**: Transient single occurrences are filtered out
- **Responsiveness**: Persistent issues are reported within seconds
- **Safety**: High-severity faults (ERROR, CRITICAL) bypass filtering entirely

Fire-and-Forget Service Calls
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The ``report()`` method uses asynchronous service calls without waiting for
responses. This ensures fault reporting never blocks the calling node, which
is important for real-time systems. If the FaultManager service is unavailable,
reports are silently dropped (logged at DEBUG level).

Thread Safety
~~~~~~~~~~~~~

``LocalFilter`` is protected by a mutex to support nodes that may call
``report()`` from multiple threads (e.g., different callback groups or timers).
The service client from ``rclcpp`` is also thread-safe.

Integration with FaultManager
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The FaultReporter is designed as a thin client to the FaultManager node:

- **Decoupled**: FaultReporter only depends on ``ros2_medkit_msgs``, not the manager
- **Lightweight**: Nodes get simple API without pulling in server-side code
- **Flexible**: Service name is configurable for custom deployments
