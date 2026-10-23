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
           - core_: ReporterCore
           - logger_: rclcpp::Logger
           --
           + FaultReporter(node, source_id, service_name)
           + report(fault_code, severity, description): void
           + is_service_ready(): bool
           + filter(): const LocalFilter&
           --
           - load_parameters(node_params, logger): FilterConfig {static}
           - send_report(report: FaultReport): void
       }

       class ReporterCore {
           - source_id_: string
           - filter_: LocalFilter
           - sink_: ReportSink
           --
           + report(fault_code, severity, description): bool
           + report_passed(fault_code): bool
           + filter(): const LocalFilter&
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
   FaultReporter *-down-> ReporterCore : owns
   ReporterCore *-down-> LocalFilter : owns
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
   - Integrates ``LocalFilter`` to reduce noise from repeated faults, through ``ReporterCore``
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

4. **ReporterCore** - ROS-free filtering and report building (``core/reporter_core.hpp``)

   - Owns the ``LocalFilter`` and a sink function that receives each ``FaultReport`` that passed
   - ``validate_filter_params()`` checks int64 parameter values before narrowing and replaces invalid ones
   - Built as the static library ``fault_reporter_core`` from ``cmake/fault_reporter_core.cmake``
     (plain CMake, no ament); the neutral types come from ``ros2_medkit_fault_detection/fault_types.hpp``

   - Everything in the core links only the header-only ``ros2_medkit_fault_detection``; the ROS adapter
     (``FaultReporter``: parameters, ``ReportFault`` client) stays in ``fault_reporter_lib``

5. **FaultTracker** - Internal state for per-fault-code tracking

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

Plain-CMake Build
~~~~~~~~~~~~~~~~~

``cmake/fault_reporter_core.cmake`` builds the core with plain CMake (CMake 3.16, C++17, no ament) as the
target ``fault_reporter_core``. The ament build includes it, and so does
``scripts/check_cores_without_ros``. The file finds ``fault_types.hpp`` at
``../ros2_medkit_fault_detection/include``, so the package directories must stay side by side. The
target is not installed or exported, and the core needs no library besides the standard library. The
ROS adapter passes a ``ReportSink`` that sends each report to the fault manager. The check passes a
counting sink.

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

- **Decoupled**: FaultReporter only depends on ``ros2_medkit_msgs`` and ``ros2_medkit_fault_detection``, not the manager
- **Lightweight**: Nodes get simple API without pulling in server-side code
- **Flexible**: Service name is configurable for custom deployments
