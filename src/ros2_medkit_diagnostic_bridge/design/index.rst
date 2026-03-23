ros2_medkit_diagnostic_bridge
=============================

This section contains design documentation for the ros2_medkit_diagnostic_bridge package.

Overview
--------

The Diagnostic Bridge converts standard ROS 2 ``/diagnostics`` messages into fault events
for the FaultManager. This enables integration with existing ROS 2 diagnostic infrastructure
while leveraging the centralized fault management capabilities of ros2_medkit.

Architecture
------------

The following diagram shows the data flow and component relationships.

.. plantuml::
   :caption: ROS 2 Medkit Diagnostic Bridge Architecture

   @startuml ros2_medkit_diagnostic_bridge_architecture

   skinparam linetype ortho
   skinparam classAttributeIconSize 0

   title ROS 2 Medkit Diagnostic Bridge - Architecture

   package "ROS 2 Framework" {
       class "rclcpp::Node" {
           +create_subscription()
           +get_logger()
       }
   }

   package "diagnostic_msgs" {
       class "msg::DiagnosticArray" {
           +header: Header
           +status: DiagnosticStatus[]
       }

       class "msg::DiagnosticStatus" {
           +OK = 0
           +WARN = 1
           +ERROR = 2
           +STALE = 3
           --
           +level: byte
           +name: string
           +message: string
           +hardware_id: string
       }
   }

   package "ros2_medkit_fault_reporter" {
       class FaultReporter {
           +report(fault_code, severity, description)
           +report_passed(fault_code)
       }
   }

   package "ros2_medkit_diagnostic_bridge" {

       class DiagnosticBridgeNode {
           +map_to_fault_code(name): string
           +map_to_severity(level): optional<uint8>
           +is_ok_level(level): bool
           --
           -diagnostics_callback(msg)
           -process_diagnostic(status)
           -generate_fault_code(name): string
           -load_parameters()
       }
   }

   ' Relationships
   DiagnosticBridgeNode -up-|> "rclcpp::Node" : extends
   DiagnosticBridgeNode o-right-> FaultReporter : uses
   DiagnosticBridgeNode ..> "msg::DiagnosticArray" : subscribes to
   "msg::DiagnosticArray" *--> "msg::DiagnosticStatus" : contains

   @enduml

Data Flow
---------

.. plantuml::
   :caption: Diagnostic Bridge Data Flow

   @startuml diagnostic_bridge_flow

   participant "Diagnostic\nPublisher" as Pub
   participant "/diagnostics\ntopic" as Topic
   participant "DiagnosticBridgeNode" as Bridge
   participant "FaultReporter" as Reporter
   participant "FaultManager" as FM

   Pub -> Topic : DiagnosticArray
   Topic -> Bridge : callback

   alt level == OK
       Bridge -> Reporter : report_passed(fault_code)
       Reporter -> FM : ReportFault(PASSED)
       note right: Triggers healing
   else level == WARN/ERROR/STALE
       Bridge -> Reporter : report(fault_code, severity, msg)
       Reporter -> FM : ReportFault(FAILED)
       note right: Creates/updates fault
   end

   @enduml

Severity Mapping
----------------

The bridge maps DiagnosticStatus levels to FaultManager severities:

.. list-table:: Severity Mapping
   :header-rows: 1
   :widths: 20 20 40

   * - DiagnosticStatus
     - Fault Severity
     - Description
   * - OK (0)
     - PASSED event
     - Sends healing event to clear fault
   * - WARN (1)
     - WARN (1)
     - Warning-level fault
   * - ERROR (2)
     - ERROR (2)
     - Error-level fault
   * - STALE (3)
     - CRITICAL (3)
     - Stale data treated as critical

Main Components
---------------

1. **DiagnosticBridgeNode** - The main ROS 2 node
   - Subscribes to ``/diagnostics`` topic (configurable)
   - Processes each DiagnosticStatus in incoming arrays

2. **Fault Code Generation** - Converts diagnostic names to fault codes
   - Custom mappings via ``name_to_code.<name>`` parameters take precedence
   - Auto-generation converts to UPPER_SNAKE_CASE: ``"motor temp"`` → ``"MOTOR_TEMP"``
   - Can be disabled with ``auto_generate_codes: false``

3. **FaultReporter Integration** - Bridges to FaultManager
   - OK status → ``report_passed()`` for healing
   - WARN/ERROR/STALE → ``report()`` with mapped severity

Configuration
-------------

Parameters
~~~~~~~~~~

.. list-table:: Node Parameters
   :header-rows: 1
   :widths: 30 15 55

   * - Parameter
     - Default
     - Description
   * - ``diagnostics_topic``
     - ``/diagnostics``
     - Topic to subscribe for diagnostic messages
   * - ``auto_generate_codes``
     - ``true``
     - Auto-generate fault codes from diagnostic names
   * - ``name_to_code.<name>``
     - (none)
     - Custom mapping from diagnostic name to fault code

Example Launch Configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: yaml

   diagnostic_bridge:
     ros__parameters:
       diagnostics_topic: "/diagnostics"
       auto_generate_codes: true
       # Custom mappings override auto-generation
       "name_to_code.motor_controller: Status": "MOTOR_001"
       "name_to_code.battery_monitor": "BATTERY_LOW"

Design Decisions
----------------

Fault Code Auto-Generation
~~~~~~~~~~~~~~~~~~~~~~~~~~

Diagnostic names are converted to fault codes using these rules:

- Convert to uppercase
- Replace non-alphanumeric characters with underscores
- Collapse multiple separators to single underscore
- Remove leading/trailing underscores

Examples:

- ``"motor temp"`` → ``"MOTOR_TEMP"``
- ``"/robot/arm"`` → ``"ROBOT_ARM"``
- ``"sensor: status"`` → ``"SENSOR_STATUS"``

STALE as CRITICAL
~~~~~~~~~~~~~~~~~

STALE diagnostics are mapped to CRITICAL severity because stale data typically indicates
a communication failure or node crash, which is a serious system health issue.
