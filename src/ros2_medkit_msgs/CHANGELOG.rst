^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

0.1.0 (2026-02-01)
------------------
* Initial release
* Fault management messages:

  * Fault.msg - Core fault data model with severity levels (INFO/WARN/ERROR/CRITICAL)
    and debounce-based status lifecycle (PREFAILED/PREPASSED/CONFIRMED/HEALED/CLEARED)
  * FaultEvent.msg - Real-time event notifications (EVENT_CONFIRMED/EVENT_CLEARED/EVENT_UPDATED)
    with auto-cleared correlation codes
  * MutedFaultInfo.msg - Fault correlation muting metadata
  * ClusterInfo.msg - Fault clustering information

* Fault management services:

  * ReportFault.srv - Report fault events with FAILED/PASSED event model
  * GetFaults.srv - Query faults with filtering by severity, status, and correlation data
  * ClearFault.srv - Clear/acknowledge faults by fault_code
  * GetSnapshots.srv - Retrieve topic snapshots captured at fault time
  * GetRosbag.srv - Retrieve rosbag recordings associated with faults

* Contributors: Bartosz Burda, Michal Faferek
