Message Definitions
===================

This page documents the ROS 2 message and service interfaces provided by
``ros2_medkit_msgs``. These interfaces are used for fault reporting, querying,
and real-time event notifications.

.. contents:: Table of Contents
   :local:
   :depth: 2

Messages
--------

Fault.msg
~~~~~~~~~

Core fault data model representing an aggregated fault condition.

.. code-block:: text

   # Global fault identifier (e.g., "MOTOR_OVERHEAT", "SENSOR_FAILURE_001")
   string fault_code

   # Fault severity level (use SEVERITY_* constants)
   uint8 severity

   # Human-readable description of the fault condition
   string description

   # Timestamp when this fault was first reported
   builtin_interfaces/Time first_occurred

   # Timestamp when this fault was last reported
   builtin_interfaces/Time last_occurred

   # Total number of FAILED events aggregated across all sources
   uint32 occurrence_count

   # Current fault status (PREFAILED, PREPASSED, CONFIRMED, HEALED, CLEARED)
   string status

   # List of source identifiers that have reported this fault
   string[] reporting_sources

**Severity Constants:**

.. list-table::
   :header-rows: 1
   :widths: 25 10 65

   * - Constant
     - Value
     - Description
   * - ``SEVERITY_INFO``
     - 0
     - Informational message, no action required
   * - ``SEVERITY_WARN``
     - 1
     - Warning, may require attention
   * - ``SEVERITY_ERROR``
     - 2
     - Error, impacts functionality
   * - ``SEVERITY_CRITICAL``
     - 3
     - Critical error, requires immediate attention. Bypasses debounce.

**Status Constants:**

.. list-table::
   :header-rows: 1
   :widths: 20 80

   * - Status
     - Description
   * - ``PREFAILED``
     - Debounce counter < 0 but above threshold. Fault detected but not confirmed.
   * - ``PREPASSED``
     - Debounce counter > 0 but below threshold. Trending towards resolution.
   * - ``CONFIRMED``
     - Debounce counter <= confirmation threshold. Fault active and verified.
   * - ``HEALED``
     - Debounce counter >= healing threshold. Resolved by PASSED events.
   * - ``CLEARED``
     - Manually acknowledged via ClearFault service.

**Debounce Lifecycle:**

.. code-block:: text

   PREFAILED ←────────────────→ PREPASSED
       │      (counter crosses 0)    │
       ▼                             ▼
   CONFIRMED                      HEALED
       │                          (retained)
       ▼
   CLEARED (manual)

FaultEvent.msg
~~~~~~~~~~~~~~

Real-time fault event notifications published on ``/fault_manager/events``.

.. code-block:: text

   # Event type (fault_confirmed, fault_cleared, fault_updated)
   string event_type

   # The fault data (current state after the event)
   Fault fault

   # Timestamp when this event was generated
   builtin_interfaces/Time timestamp

   # Symptom codes auto-cleared with root cause (correlation feature)
   string[] auto_cleared_codes

**Event Types:**

.. list-table::
   :header-rows: 1
   :widths: 25 75

   * - Event
     - Description
   * - ``fault_confirmed``
     - Fault transitioned from PREFAILED to CONFIRMED
   * - ``fault_cleared``
     - Fault cleared via ClearFault service
   * - ``fault_updated``
     - Fault data changed without status transition (e.g., new occurrence)

MutedFaultInfo.msg
~~~~~~~~~~~~~~~~~~

Information about correlated (muted) symptom faults.

.. code-block:: text

   string fault_code       # The muted symptom's fault code
   string root_cause_code  # Root cause that triggered muting
   string rule_id          # Correlation rule ID that matched
   uint32 delay_ms         # Time delay from root cause [ms]

ClusterInfo.msg
~~~~~~~~~~~~~~~

Auto-detected fault cluster information.

.. code-block:: text

   string cluster_id              # Unique cluster ID
   string rule_id                 # Correlation rule ID
   string rule_name               # Human-readable rule name
   string label                   # Cluster label (e.g., "Communication Storm")
   string representative_code     # Primary fault code for display
   string representative_severity # Severity of representative fault
   string[] fault_codes           # All fault codes in cluster
   uint32 count                   # Number of faults
   builtin_interfaces/Time first_at  # First fault timestamp
   builtin_interfaces/Time last_at   # Last fault timestamp

Services
--------

ReportFault.srv
~~~~~~~~~~~~~~~

Report a fault event to the FaultManager.

**Request:**

.. code-block:: text

   string fault_code   # Global fault identifier (UPPER_SNAKE_CASE)
   uint8 event_type    # EVENT_FAILED (0) or EVENT_PASSED (1)
   uint8 severity      # Fault.SEVERITY_* constant (for FAILED events)
   string description  # Human-readable description
   string source_id    # Fully qualified node name (e.g., "/powertrain/temp_sensor")

**Response:**

.. code-block:: text

   bool accepted       # True if event was accepted

**Example Usage:**

.. code-block:: cpp

   #include "ros2_medkit_msgs/srv/report_fault.hpp"

   auto request = std::make_shared<ros2_medkit_msgs::srv::ReportFault::Request>();
   request->fault_code = "MOTOR_OVERHEAT";
   request->event_type = ros2_medkit_msgs::srv::ReportFault::Request::EVENT_FAILED;
   request->severity = ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR;
   request->description = "Motor temperature exceeded 85°C";
   request->source_id = get_fully_qualified_name();

   auto result = client->async_send_request(request);

ClearFault.srv
~~~~~~~~~~~~~~

Clear/acknowledge a fault.

**Request:**

.. code-block:: text

   string fault_code   # Fault code to clear

**Response:**

.. code-block:: text

   bool success            # True if fault was found and cleared
   string message          # Status message or error description
   string[] auto_cleared_codes  # Symptoms auto-cleared with root cause

ListFaults.srv
~~~~~~~~~~~~~~

Query faults from the FaultManager with optional filtering.

**Request:**

.. code-block:: text

   bool filter_by_severity   # Whether to filter by severity
   uint8 severity            # Severity to filter by (if filter_by_severity=true)
   string[] statuses         # Status filter (empty = CONFIRMED only)
   bool include_muted        # Include correlated symptoms
   bool include_clusters     # Include cluster information

**Response:**

.. code-block:: text

   Fault[] faults             # Matching faults
   uint32 muted_count         # Total muted faults
   MutedFaultInfo[] muted_faults  # Muted fault details (if requested)
   uint32 cluster_count       # Total clusters
   ClusterInfo[] clusters     # Cluster details (if requested)

**Example: Query all confirmed and pre-failed faults:**

.. code-block:: cpp

   auto request = std::make_shared<ros2_medkit_msgs::srv::ListFaults::Request>();
   request->filter_by_severity = false;
   request->statuses = {"CONFIRMED", "PREFAILED"};

   auto result = client->async_send_request(request);

GetSnapshots.srv
~~~~~~~~~~~~~~~~

Retrieve diagnostic snapshots captured at fault occurrence time.

See :doc:`/tutorials/snapshots` for detailed usage.

See Also
--------

- :doc:`/design/ros2_medkit_fault_reporter/index` - How to report faults from your nodes
- :doc:`/tutorials/fault-correlation` - Configure fault correlation rules
- :doc:`/tutorials/snapshots` - Diagnostic snapshot capture
- :doc:`/design/ros2_medkit_fault_manager/index` - FaultManager design documentation
