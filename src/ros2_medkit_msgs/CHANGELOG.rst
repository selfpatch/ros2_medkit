^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.0 (2026-02-27)
------------------
* Multi-distro CI support for ROS 2 Humble, Jazzy, and Rolling (`#219 <https://github.com/selfpatch/ros2_medkit/pull/219>`_, `#242 <https://github.com/selfpatch/ros2_medkit/pull/242>`_)
* Contributors: @bburda

0.2.0 (2026-02-07)
------------------
* Initial rosdistro release
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
