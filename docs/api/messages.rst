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

Core fault data model representing one fault record.

A record is identified by the pair (``fault_code``, owner). A code listed with
``per_source: true`` in ``fault_thresholds.yaml`` has one record per reporting source, owned by
the ``source_id`` of the ``ReportFault`` call. Every other code has one record shared by all
its sources, owned by the reserved id ``*``. Each record has its own status, debounce counter,
``occurrence_count``, severity and timestamps, and is cleared on its own.

.. code-block:: text

   # Global fault identifier (e.g., "MOTOR_OVERHEAT", "SENSOR_FAILURE_001")
   string fault_code

   # Fault severity level (use SEVERITY_* constants)
   uint8 severity

   # Human-readable description of the fault condition
   string description

   # Timestamp when the current occurrence started (reset when a FAILED event
   # reactivates a CLEARED fault, so it moves with occurrence_count instead of
   # marking the fault's first report ever)
   builtin_interfaces/Time first_occurred

   # Timestamp when this fault last occurred (FAILED events only; a PASSED
   # event is the fault ending, not occurring, and does not touch this field)
   builtin_interfaces/Time last_occurred

   # Timestamp of the last PASSED event reported for this fault (zero = never)
   builtin_interfaces/Time last_passed

   # Number of times this fault has occurred, counted on edges: one for the first
   # FAILED event, and one more each time a FAILED event arrives while the fault is
   # CLEARED. Repeated FAILED events within one occurrence do not increment it;
   # last_occurred is what advances on those.
   uint32 occurrence_count

   # Current fault status (PREFAILED, PREPASSED, CONFIRMED, HEALED, CLEARED)
   string status

   # Every source that reported this record: one for a per-source record, all of
   # them for a shared one.
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

Real-time fault event notifications published on ``/fault_manager/events`` by default.
When the fault manager runs under an additional namespace, the topic follows the same
prefix (for example ``/robot1/fault_manager/events``).

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
     - Fault ended: cleared via the ClearFault service, or healed when PASSED
       events crossed the healing threshold (``fault.status`` distinguishes
       ``CLEARED`` from ``HEALED``)
   * - ``fault_updated``
     - Fault data changed without status transition (e.g., new occurrence)

MutedFaultInfo.msg
~~~~~~~~~~~~~~~~~~

Information about a correlated (muted) symptom record. One entry per muted RECORD:
a root cause mutes the matching symptom records of every owner, so two per-source records
muted on one ``fault_code`` produce two entries carrying that code, told apart by
``source_id``.

.. code-block:: text

   string fault_code       # The muted symptom's fault code
   string root_cause_code  # Root cause that triggered muting
   string rule_id          # Correlation rule ID that matched
   uint32 delay_ms         # Time delay from root cause [ms]
   string source_id        # Owner of the muted record (``*`` for a shared record)

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
   string[] fault_owners          # Owner of each member, parallel to fault_codes
   string representative_owner    # Owner of the representative record

A cluster holds the matching records of every owner, so a member is the record
``(fault_codes[i], fault_owners[i])``. Pass the owner as ``source_id`` to the
single-record services. The owner of a shared record is ``*``.

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
   string source_id    # Fully qualified node name (e.g., "/powertrain/temp_sensor").
                       # Owns the record of a per_source code. "*" is reserved and refused.

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

Clear/acknowledge one fault record.

**Request:**

.. code-block:: text

   string fault_code                # Fault code to clear
   bool   skip_correlation_auto_clear  # Opt out of correlation cascade clear
   string source_id                 # Owner of the record, or a source that reports into it

**Response:**

.. code-block:: text

   bool success            # True if the record was found and cleared
   string message          # Status message or error description
   string[] auto_cleared_codes  # Symptoms auto-cleared with root cause

``source_id`` names the owner of the record to clear (``*`` for a shared record). When no
record has that owner, it names the record that source reports into. Leaving it empty is
unscoped: the call applies only when exactly one record carries the ``fault_code``, and fails
otherwise with ``success=false`` and a ``message`` beginning ``ambiguous:`` that lists the owners.
An ambiguous call clears nothing. ``GetFault``, ``GetSnapshots`` and ``GetRosbag`` carry
the same field and resolve the same way. On ``GetRosbag`` it scopes the ``fault_code``
lookup only, and the ``recording_id`` path ignores it.

``GetRosbag`` answers with ``fault_codes`` and ``owners``, parallel arrays with one entry per
fault record the recording is attached to. A burst of several per-source records of one code
lists the code once per owner. A shared record is one entry carrying the owner ``*``.

When ``skip_correlation_auto_clear`` is ``false`` (default), clearing a
root-cause fault also clears every symptom that the correlation engine
attributes to it via ``auto_clear_with_root`` rules; the cleared
symptom codes are returned in ``auto_cleared_codes``. When ``true``,
only the requested ``fault_code`` is cleared and ``auto_cleared_codes``
is empty. The gateway's per-entity ``DELETE
/{entity-path}/faults/{fault_code}`` route sets this to ``true`` so
that an operator with access to one entity cannot cascade-clear
correlated symptoms reported by apps in other entities. The global
``DELETE /api/v1/faults/{fault_code}`` route leaves it ``false`` so
cluster-wide clearing still works.

.. note::

   ``skip_correlation_auto_clear`` was added in ``ros2_medkit_msgs`` post-0.4.0,
   and ``source_id`` after it (the same field was added to ``GetFault``,
   ``GetSnapshots`` and ``GetRosbag``). Adding a field changes the type hash, and
   a field added inside an array element changes the wire layout, so every client
   of the fault manager's services and topics must be rebuilt against this
   ``ros2_medkit_msgs``. This includes out-of-tree callers that use ``ros2
   service call`` or a generated client. The case that breaks without an error is
   ``ListFaults``: ``MutedFaultInfo.source_id`` and ``ClusterInfo.fault_owners`` and
   ``representative_owner`` sit inside the elements of the ``muted_faults`` and
   ``clusters`` arrays, CDR has no per-element length, and a client built against
   an earlier ``ros2_medkit_msgs`` misreads any response that carries a muted or
   clustered entry. Trailing fields do not keep old clients working.

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

MedkitDiscoveryHint.msg
~~~~~~~~~~~~~~~~~~~~~~~

Push-based entity enrichment hints published by ROS 2 nodes to the gateway
via the ``/ros2_medkit/discovery`` topic. The topic beacon plugin
(``ros2_medkit_topic_beacon``) subscribes to this topic and enriches the
discovery merge pipeline with the received metadata.

.. code-block:: text

   # Required
   string entity_id                        # Target entity (App or Component ID)

   # Identity hints
   string stable_id                        # Stable ID alias (x-medkit-stable-id metadata)
   string display_name                     # Human-friendly display name

   # Topology hints
   string[] function_ids                   # Function membership (entity belongs to these Functions)
   string[] depends_on                     # IDs of entities this entity depends on
   string component_id                     # Parent Component binding

   # Transport hints
   string transport_type                   # "nitros_zero_copy", "shared_memory", "intra_process"
   string negotiated_format               # "nitros_image_bgr8", etc.

   # Process diagnostics
   uint32 process_id                       # OS process ID (PID), 0 = not provided
   string process_name                     # Process name (e.g. "component_container")
   string hostname                         # Host identifier (for distributed systems)

   # Freeform metadata
   diagnostic_msgs/KeyValue[] metadata    # Arbitrary key-value pairs

   # Timing
   builtin_interfaces/Time stamp           # Timestamp for TTL calculation

All fields except ``entity_id`` are optional. Empty strings and empty arrays
mean "not provided" and are ignored by the plugin.

The same field semantics apply to ``ParameterBeaconPlugin`` via node
parameters. See :doc:`/config/discovery-options` for parameter naming.

The ``stamp`` field is used by the beacon TTL lifecycle:

- **ACTIVE** - hint is within the configured TTL (enrichment applied)
- **STALE** - hint is past TTL but within expiry (diagnostic data preserved,
  enrichment still applied with stale flag)
- **EXPIRED** - hint is past expiry (removed from enrichment)

**Example publisher (C++):**

.. code-block:: cpp

   #include "ros2_medkit_msgs/msg/medkit_discovery_hint.hpp"

   auto pub = node->create_publisher<ros2_medkit_msgs::msg::MedkitDiscoveryHint>(
     "/ros2_medkit/discovery", 10);

   ros2_medkit_msgs::msg::MedkitDiscoveryHint hint;
   hint.entity_id = "my_sensor_node";
   hint.display_name = "Temperature Sensor";
   hint.function_ids = {"thermal_monitoring"};
   hint.process_id = getpid();
   hint.stamp = node->now();
   pub->publish(hint);

See Also
--------

- :doc:`/design/ros2_medkit_fault_reporter/index` - How to report faults from your nodes
- :doc:`/tutorials/fault-correlation` - Configure fault correlation rules
- :doc:`/tutorials/snapshots` - Diagnostic snapshot capture
- :doc:`/design/ros2_medkit_fault_manager/index` - FaultManager design documentation
- :doc:`/tutorials/plugin-system` - Beacon plugin configuration and development
