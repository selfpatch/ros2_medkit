Faults
======

.. req:: GET /{entity}/faults
   :id: REQ_INTEROP_012
   :status: verified
   :tags: Faults

   The endpoint shall return the list of diagnostic fault entries stored for the addressed entity, possibly including active and stored faults.

.. req:: GET /{entity}/faults/{code}
   :id: REQ_INTEROP_013
   :status: verified
   :tags: Faults, SOVD

   The endpoint shall return detailed information for the addressed diagnostic fault code,
   including environment data captured at fault confirmation.

   Response includes:

   - ``item``: Fault details with SOVD-compliant ``status`` object (aggregatedStatus, testFailed, confirmedDTC, pendingDTC)
   - ``environment_data``: Extended data records (timestamps) and snapshots array
   - ``environment_data.snapshots[]``: Array of freeze_frame (topic data) and rosbag (bulk-data reference) entries
   - ``x-medkit``: Extension fields (occurrence_count, reporting_sources, severity_label)

.. req:: DELETE /{entity}/faults
   :id: REQ_INTEROP_014
   :status: verified
   :tags: Faults

   The endpoint shall clear all diagnostic fault entries stored for the addressed entity, if permitted.

.. req:: DELETE /{entity}/faults/{code}
   :id: REQ_INTEROP_015
   :status: verified
   :tags: Faults

   The endpoint shall clear the addressed diagnostic fault code for the entity, if permitted.

.. req:: Per-Entity Debounce Thresholds
   :id: REQ_INTEROP_095
   :status: verified
   :tags: Faults

   The fault manager shall support per-entity debounce threshold configuration using
   longest-prefix matching on the reporting entity's ``source_id``. Entity-specific
   overrides for ``confirmation_threshold``, ``healing_enabled``, and ``healing_threshold``
   shall take precedence over global defaults. Unspecified fields shall inherit from global
   configuration. When no entity prefix matches, global defaults shall apply.

.. req:: Per-Fault-Code Debounce Thresholds
   :id: REQ_INTEROP_107
   :status: verified
   :tags: Faults

   The fault manager shall support per-fault_code debounce threshold configuration using
   exact matching on the reported fault code. A fault-code override for
   ``confirmation_threshold``, ``healing_enabled``, or ``healing_threshold`` shall take
   precedence over both the per-entity override resolved for the reporting source and the
   global defaults. Unspecified fields shall inherit from the layer below. When a fault code
   has no override, the resolved per-entity or global configuration shall apply unchanged.

.. req:: Conflicting Debounce Policies Are Reported
   :id: REQ_INTEROP_108
   :status: verified
   :tags: Faults

   The debounce counter belongs to the fault code while a per-entity override is resolved
   from the reporting source. When two sources report one fault code and resolve to
   different debounce policies, the fault manager shall warn, naming the fault code, both
   sources and both resolved policies, at most once per fault code for the life of the node.

.. req:: Fault Snapshot and Rosbag Capture
   :id: REQ_INTEROP_088
   :status: verified
   :tags: Faults, BulkData

   When a fault is confirmed, the system shall automatically capture diagnostic snapshots
   (freeze-frame topic data) and rosbag recordings as configured. Captured data shall be
   stored as environment data associated with the fault and accessible via the bulk-data
   endpoints on the fault's reporting entity. For faults already confirmed when the
   capturing component starts (e.g. the gateway restarts while a fault is standing), an
   equivalent snapshot shall be captured at startup and marked with its capture origin.

