^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_fault_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Initial rosdistro release
* Central fault management node with ROS 2 services:

  * ReportFault - report FAILED/PASSED events with debounce filtering
  * GetFaults - query faults with filtering by severity, status, correlation
  * ClearFault - clear/acknowledge faults

* Debounce filtering with configurable thresholds:

  * FAILED events decrement counter, PASSED events increment
  * Configurable confirmation_threshold (default: -1, immediate)
  * Optional healing support (healing_enabled, healing_threshold)
  * Time-based auto-confirmation (auto_confirm_after_sec)
  * CRITICAL severity bypasses debounce

* Dual storage backends:

  * SQLite persistent storage with WAL mode (default)
  * In-memory storage for testing/lightweight deployments

* Snapshot capture on fault confirmation:

  * Topic data captured as JSON with configurable topic resolution
  * Priority: fault_specific > patterns > default_topics
  * Stored in SQLite with indexed fault_code lookup
  * Auto-cleanup on fault clear

* Rosbag capture with ring buffer:

  * Configurable duration, post-fault recording, topic selection
  * Lazy start mode (start on PREFAILED) or immediate
  * Auto-cleanup of bag files, storage limits (max_bag_size_mb)
  * GetRosbag service for bag file metadata

* Fault correlation engine:

  * Hierarchical mode: root cause to symptom relationships
  * Auto-cluster mode: group similar faults within time window
  * YAML-based configuration with pattern wildcards
  * Muted faults tracking, auto-clear on root cause resolution

* FaultEvent publishing on ~/events topic for SSE streaming
* Wall clock timestamps (compatible with use_sim_time)
* Contributors: Bartosz Burda, Michal Faferek
