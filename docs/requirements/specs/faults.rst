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

.. req:: Fault Snapshot and Rosbag Capture
   :id: REQ_INTEROP_088
   :status: verified
   :tags: Faults, BulkData

   When a fault is confirmed, the system shall automatically capture diagnostic snapshots
   (freeze-frame topic data) and rosbag recordings as configured. Captured data shall be
   stored as environment data associated with the fault and accessible via the bulk-data
   endpoints on the fault's reporting entity.

