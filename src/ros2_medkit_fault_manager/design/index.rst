ros2_medkit_fault_manager
=========================

This section contains design documentation for the ros2_medkit_fault_manager project.

Architecture
------------

The following diagram shows the relationships between the main components of the fault manager.

.. plantuml::
   :caption: ROS 2 Medkit Fault Manager Class Architecture

   @startuml ros2_medkit_fault_manager_architecture

   skinparam linetype ortho
   skinparam classAttributeIconSize 0

   title ROS 2 Medkit Fault Manager - Class Architecture

   package "ROS 2 Framework" {
       class "rclcpp::Node" {
           +create_service()
           +get_logger()
           +now()
       }
   }

   package "ros2_medkit_msgs" {
       class "msg::Fault" {
           +fault_code: string
           +severity: uint8
           +description: string
           +first_occurred: Time
           +last_occurred: Time
           +occurrence_count: uint32
           +status: string
           +reporting_sources: string[]
       }

       class "srv::ReportFault" {
           +Request: fault_code, severity, description, source_id
           +Response: success, message
       }

       class "srv::ListFaults" {
           +Request: filter_by_severity, severity, statuses
           +Response: faults[]
       }

       class "srv::ClearFault" {
           +Request: fault_code
           +Response: success, message
       }
   }

   package "ros2_medkit_fault_manager" {

       class FaultManagerNode {
           + get_storage(): FaultStorage&
       }

       abstract class FaultStorage <<interface>> {
           + {abstract} report_fault(): bool
           + {abstract} list_faults(): vector<Fault>
           + {abstract} get_fault(): optional<Fault>
           + {abstract} clear_fault(): bool
           + {abstract} size(): size_t
           + {abstract} contains(): bool
       }

       class InMemoryFaultStorage {
           + report_fault(): bool
           + list_faults(): vector<Fault>
           + get_fault(): optional<Fault>
           + clear_fault(): bool
           + size(): size_t
           + contains(): bool
       }

       class FaultState <<struct>> {
           + to_msg(): Fault
       }
   }

   ' Relationships

   ' Inheritance
   FaultManagerNode -up-|> "rclcpp::Node" : extends
   InMemoryFaultStorage -up-|> FaultStorage : implements

   ' Composition
   FaultManagerNode *-down-> InMemoryFaultStorage : owns

   ' InMemoryFaultStorage contains FaultStates
   InMemoryFaultStorage o-right-> FaultState : contains many

   ' FaultState converts to message
   FaultState ..> "msg::Fault" : converts to

   ' Node uses service types
   FaultManagerNode ..> "srv::ReportFault" : handles
   FaultManagerNode ..> "srv::ListFaults" : handles
   FaultManagerNode ..> "srv::ClearFault" : handles

   @enduml

Main Components
---------------

1. **FaultManagerNode** - The main ROS 2 node that provides fault management services
   - Extends ``rclcpp::Node``
   - Owns a ``FaultStorage`` implementation for fault state persistence
   - Provides three ROS 2 services for fault reporting, querying, and clearing
   - Validates input parameters (fault_code, severity, source_id)
   - Logs fault lifecycle events at appropriate severity levels

2. **FaultStorage** - Abstract interface for fault storage backends
   - Defines the contract for fault storage implementations
   - Enables pluggable storage backends (in-memory, persistent, distributed)
   - Future implementations can be added in Issue #8: Fault Persistence Options

3. **InMemoryFaultStorage** - Thread-safe in-memory implementation of FaultStorage
   - Uses ``std::map`` keyed by ``fault_code`` for O(log n) lookups
   - Protected by ``std::mutex`` for concurrent service request handling
   - Aggregates reports from multiple sources into single fault entries
   - Implements severity escalation (higher severity overwrites lower)
   - Tracks occurrence counts and all reporting sources

4. **FaultState** - Internal representation of a fault entry
   - Maps directly to ``ros2_medkit_msgs::msg::Fault`` via ``to_msg()``
   - Uses ``std::set`` for reporting_sources to ensure uniqueness
   - Tracks first and last occurrence timestamps
   - Manages fault status lifecycle with debounce (PREFAILED → CONFIRMED → CLEARED)

Services
--------

~/report_fault
~~~~~~~~~~~~~~

Reports a new fault or updates an existing one.

- **Input validation**: fault_code and source_id cannot be empty, event_type must be valid
- **Event types**: FAILED (fault detected) or PASSED (fault condition cleared)
- **Debounce**: FAILED events decrement counter, PASSED events increment counter
- **Aggregation**: Same fault_code from different sources creates a single fault entry
- **Severity escalation**: Fault severity is updated if a higher severity is reported
- **Returns**: ``accepted=true`` if event was processed

~/list_faults
~~~~~~~~~~~~~

Queries faults with optional filtering.

- **Status filter**: Filter by status (PREFAILED, PREPASSED, CONFIRMED, HEALED, CLEARED); defaults to CONFIRMED
- **Severity filter**: When ``filter_by_severity=true``, returns only faults of specified severity
- **Returns**: List of ``Fault`` messages matching the filter criteria

~/clear_fault
~~~~~~~~~~~~~

Clears (acknowledges) a fault by setting its status to CLEARED.

- **Input validation**: fault_code cannot be empty
- **Idempotent**: Clearing an already-cleared fault succeeds
- **Returns**: ``success=true`` if fault existed, ``success=false`` if not found

Design Decisions
----------------

Thread Safety
~~~~~~~~~~~~~

All ``FaultStorage`` public methods acquire a mutex lock to ensure thread safety
when handling concurrent service requests. This is essential since ROS 2 service
callbacks may execute on different threads.

Fault Aggregation
~~~~~~~~~~~~~~~~~

Multiple reports of the same ``fault_code`` (from same or different sources) are
aggregated into a single fault entry. This provides:

- **Deduplication**: Prevents fault flooding from repeated reports
- **Source tracking**: Identifies all sources reporting the same fault
- **Occurrence counting**: Tracks how many times a fault was reported

Severity Escalation
~~~~~~~~~~~~~~~~~~~

When a fault is re-reported with a higher severity, the stored severity is updated.
This ensures the fault reflects the worst-case condition. Severity levels are ordered:
``INFO(0) < WARN(1) < ERROR(2) < CRITICAL(3)``.

Status Lifecycle (Debounce Model)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Faults follow an AUTOSAR DEM-style debounce lifecycle:

- **PREFAILED**: Debounce counter < 0 but above confirmation threshold (fault trending towards confirmation)
- **PREPASSED**: Debounce counter > 0 but below healing threshold (fault trending towards healing)
- **CONFIRMED**: Debounce counter <= confirmation threshold (e.g., -3). Fault is active and verified.
- **HEALED**: Debounce counter >= healing threshold (if healing enabled). Fault resolved by PASSED events.
- **CLEARED**: Fault manually acknowledged via ClearFault service

FAILED events decrement the debounce counter (towards confirmation).
PASSED events increment the debounce counter (towards healing).
CRITICAL severity bypasses debounce and confirms immediately.
