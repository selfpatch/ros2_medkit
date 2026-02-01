Fault Manager Configuration
===========================

The ``ros2_medkit_fault_manager`` node aggregates and manages faults from multiple sources.
This page documents all configuration parameters.

.. contents:: Table of Contents
   :local:
   :depth: 2

Basic Configuration
-------------------

Storage
~~~~~~~

.. code-block:: yaml

   fault_manager:
     ros__parameters:
       storage_type: "sqlite"              # Storage backend: "sqlite" or "memory"
       database_path: "/var/lib/ros2_medkit/faults.db"  # Path for sqlite storage

.. list-table::
   :header-rows: 1
   :widths: 25 15 60

   * - Parameter
     - Default
     - Description
   * - ``storage_type``
     - ``sqlite``
     - Storage backend. ``sqlite`` persists faults to disk, ``memory`` keeps in RAM only.
   * - ``database_path``
     - ``/var/lib/ros2_medkit/faults.db``
     - File path for SQLite database. Directory must exist and be writable.

Debounce Settings
~~~~~~~~~~~~~~~~~

The fault manager uses AUTOSAR DEM-style debounce filtering to prevent fault flapping.

.. code-block:: yaml

   fault_manager:
     ros__parameters:
       confirmation_threshold: -1          # Counter threshold to confirm fault
       healing_enabled: false              # Enable auto-healing via PASSED events
       healing_threshold: 3                # Counter threshold to heal fault
       auto_confirm_after_sec: 0.0         # Auto-confirm timeout (0 = disabled)

.. list-table::
   :header-rows: 1
   :widths: 30 12 58

   * - Parameter
     - Default
     - Description
   * - ``confirmation_threshold``
     - ``-1``
     - Number of FAILED events to confirm fault. Negative values mean more events needed.
       Use ``-3`` to require 3 FAILED events before confirmation.
   * - ``healing_enabled``
     - ``false``
     - When true, PASSED events can heal confirmed faults.
   * - ``healing_threshold``
     - ``3``
     - Number of PASSED events to transition from CONFIRMED to HEALED.
   * - ``auto_confirm_after_sec``
     - ``0.0``
     - Auto-confirm prefailed faults after this duration. Set to 0 to disable.

.. tip::

   For immediate fault confirmation (no debounce), set ``confirmation_threshold: 0``.
   Faults with ``SEVERITY_CRITICAL`` always bypass debounce regardless of this setting.

Snapshot Configuration
----------------------

Snapshots capture diagnostic data when faults occur.

Basic Snapshot Settings
~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: yaml

   fault_manager:
     ros__parameters:
       snapshots:
         enabled: true                     # Enable snapshot capture
         background_capture: false         # Capture in background thread
         timeout_sec: 1.0                  # Timeout for topic sampling
         max_message_size: 65536           # Max message size in bytes (64KB)
         default_topics: []                # Topics to capture for all faults
         config_file: ""                   # Path to YAML config file

.. list-table::
   :header-rows: 1
   :widths: 30 15 55

   * - Parameter
     - Default
     - Description
   * - ``snapshots.enabled``
     - ``true``
     - Master switch to enable/disable snapshot capture.
   * - ``snapshots.background_capture``
     - ``false``
     - Capture snapshots in background thread (non-blocking).
   * - ``snapshots.timeout_sec``
     - ``1.0``
     - Timeout for sampling each topic.
   * - ``snapshots.max_message_size``
     - ``65536``
     - Maximum message size to capture (bytes). Larger messages are truncated.
   * - ``snapshots.default_topics``
     - ``[]``
     - List of topics to capture for all faults.
   * - ``snapshots.config_file``
     - ``""``
     - Path to YAML file with fault-specific snapshot configurations.

Rosbag Recording
~~~~~~~~~~~~~~~~

Capture continuous rosbag recordings around fault events.

.. code-block:: yaml

   fault_manager:
     ros__parameters:
       snapshots:
         rosbag:
           enabled: false                  # Enable rosbag recording
           duration_sec: 5.0               # Pre-fault buffer duration
           duration_after_sec: 1.0         # Post-fault recording duration
           topics: "config"                # Topic selection: "config", "all", or "none"
           include_topics: []              # Additional topics to include
           exclude_topics: []              # Topics to exclude
           lazy_start: false               # Start recording on first fault
           format: "sqlite3"               # Storage format
           storage_path: ""                # Custom storage path
           max_bag_size_mb: 50             # Max size per bag file
           max_total_storage_mb: 500       # Max total storage
           auto_cleanup: true              # Auto-delete old bags

.. list-table::
   :header-rows: 1
   :widths: 35 15 50

   * - Parameter
     - Default
     - Description
   * - ``rosbag.enabled``
     - ``false``
     - Enable rosbag recording for snapshots.
   * - ``rosbag.duration_sec``
     - ``5.0``
     - Duration of pre-fault circular buffer.
   * - ``rosbag.duration_after_sec``
     - ``1.0``
     - How long to record after fault.
   * - ``rosbag.topics``
     - ``config``
     - Topic selection mode: ``config`` (per-fault), ``all``, or ``none``.
   * - ``rosbag.lazy_start``
     - ``false``
     - Start recording only when first fault occurs.
   * - ``rosbag.max_bag_size_mb``
     - ``50``
     - Maximum size per rosbag file (MB).
   * - ``rosbag.max_total_storage_mb``
     - ``500``
     - Maximum total storage for all rosbags (MB).
   * - ``rosbag.auto_cleanup``
     - ``true``
     - Automatically delete oldest rosbags when storage limit reached.

.. seealso::

   :doc:`/tutorials/snapshots` for detailed snapshot configuration examples.

Correlation Configuration
-------------------------

Fault correlation identifies root causes and filters symptom faults.

.. code-block:: yaml

   fault_manager:
     ros__parameters:
       correlation:
         config_file: "/path/to/correlation_rules.yaml"
         cleanup_interval_sec: 5.0         # Interval for cleanup tasks

.. list-table::
   :header-rows: 1
   :widths: 35 15 50

   * - Parameter
     - Default
     - Description
   * - ``correlation.config_file``
     - ``""``
     - Path to YAML file defining correlation rules.
   * - ``correlation.cleanup_interval_sec``
     - ``5.0``
     - Interval for running correlation cleanup tasks.

.. seealso::

   :doc:`/tutorials/fault-correlation` for correlation rule syntax and examples.

Complete Example
----------------

.. code-block:: yaml

   fault_manager:
     ros__parameters:
       # Storage
       storage_type: "sqlite"
       database_path: "/var/lib/ros2_medkit/faults.db"

       # Debounce (require 3 FAILED events to confirm)
       confirmation_threshold: -3
       healing_enabled: true
       healing_threshold: 3
       auto_confirm_after_sec: 30.0

       # Snapshots
       snapshots:
         enabled: true
         background_capture: true
         timeout_sec: 2.0
         max_message_size: 131072
         default_topics:
           - /diagnostics
           - /rosout
         config_file: "/etc/ros2_medkit/snapshot_config.yaml"
         rosbag:
           enabled: true
           duration_sec: 10.0
           duration_after_sec: 2.0
           topics: "config"
           max_bag_size_mb: 100
           max_total_storage_mb: 1000
           auto_cleanup: true

       # Correlation
       correlation:
         config_file: "/etc/ros2_medkit/correlation_rules.yaml"
         cleanup_interval_sec: 10.0

See Also
--------

- :doc:`/tutorials/snapshots` - Diagnostic snapshot configuration
- :doc:`/tutorials/fault-correlation` - Fault correlation rules
- :doc:`/api/messages` - Message definitions (Fault.msg, FaultEvent.msg)
- :doc:`/design/ros2_medkit_fault_manager/index` - FaultManager architecture
