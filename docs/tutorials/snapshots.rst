Configuring Snapshot Capture
============================

This tutorial shows how to configure snapshot capture to automatically
preserve topic data when faults are confirmed, enabling post-mortem debugging.

.. contents:: Table of Contents
   :local:
   :depth: 2

Overview
--------

When a fault transitions to CONFIRMED status, the system can automatically
capture data from configured ROS 2 topics. This snapshot preserves the
system state at the moment of fault occurrence, similar to:

- **AUTOSAR DEM freeze frames** - diagnostic data captured at fault detection
- **SOVD environment data** - system context for fault analysis

Snapshots are useful for:

- Debugging intermittent faults that are hard to reproduce
- Understanding system state when a fault occurred
- Post-mortem analysis without real-time access to the robot

.. note::

   Snapshots are automatically deleted when a fault is cleared via the
   ``DELETE /api/v1/faults/{code}`` endpoint or ``~/clear_fault`` service.

Quick Start
-----------

1. **Start the fault manager with snapshot capture enabled:**

   .. code-block:: bash

      ros2 run ros2_medkit_fault_manager fault_manager_node --ros-args \
        -p snapshots.enabled:=true \
        -p snapshots.default_topics:="['/odom', '/battery_state']"

2. **Start the gateway:**

   .. code-block:: bash

      ros2 launch ros2_medkit_gateway gateway.launch.py

3. **When a fault is confirmed, query its snapshots:**

   .. code-block:: bash

      curl http://localhost:8080/api/v1/faults/MOTOR_OVERHEAT/snapshots

Configuration Options
---------------------

Configure snapshot capture via fault manager parameters:

.. list-table::
   :widths: 30 15 55
   :header-rows: 1

   * - Parameter
     - Default
     - Description
   * - ``snapshots.enabled``
     - ``true``
     - Enable/disable snapshot capture
   * - ``snapshots.default_topics``
     - ``[]``
     - Topics to capture for all faults
   * - ``snapshots.config_file``
     - ``""``
     - Path to YAML config for fault-specific topics
   * - ``snapshots.timeout_sec``
     - ``1.0``
     - Timeout waiting for topic message
   * - ``snapshots.max_message_size``
     - ``65536``
     - Maximum message size in bytes (larger messages skipped)
   * - ``snapshots.background_capture``
     - ``false``
     - Use background subscriptions (caches latest message)

Advanced Configuration
----------------------

For fault-specific topic capture, create a YAML configuration file:

.. code-block:: yaml

   # snapshots.yaml
   fault_specific:
     MOTOR_OVERHEAT:
       - /joint_states
       - /motor/temperature
     BATTERY_LOW:
       - /battery_state
       - /power_management/status

   patterns:
     "MOTOR_.*":
       - /joint_states
       - /cmd_vel
     "SENSOR_.*":
       - /diagnostics

**Topic Resolution Priority:**

1. ``fault_specific`` - Exact match for fault code
2. ``patterns`` - Regex pattern match (first matching pattern wins)
3. ``default_topics`` - Fallback for all faults

**Launch with config file:**

.. code-block:: bash

   ros2 run ros2_medkit_fault_manager fault_manager_node --ros-args \
     -p snapshots.enabled:=true \
     -p snapshots.config_file:=/path/to/snapshots.yaml \
     -p snapshots.default_topics:="['/diagnostics']"

Querying Snapshots
------------------

**Get all snapshots for a fault:**

.. code-block:: bash

   curl http://localhost:8080/api/v1/faults/MOTOR_OVERHEAT/snapshots

**Response:**

.. code-block:: json

   {
     "fault_code": "MOTOR_OVERHEAT",
     "captured_at": 1735830000.123,
     "topics": {
       "/joint_states": {
         "message_type": "sensor_msgs/msg/JointState",
         "data": {
           "name": ["joint1", "joint2"],
           "position": [1.57, 0.0]
         }
       },
       "/motor/temperature": {
         "message_type": "sensor_msgs/msg/Temperature",
         "data": {
           "temperature": 85.5,
           "variance": 0.1
         }
       }
     }
   }

**Filter by specific topic:**

.. code-block:: bash

   curl "http://localhost:8080/api/v1/faults/MOTOR_OVERHEAT/snapshots?topic=/joint_states"

**Component-scoped snapshots:**

.. code-block:: bash

   curl http://localhost:8080/api/v1/components/motor_controller/faults/MOTOR_OVERHEAT/snapshots

Example Workflow
----------------

This example demonstrates the complete snapshot capture workflow.

**1. Configure and start the fault manager:**

.. code-block:: bash

   ros2 run ros2_medkit_fault_manager fault_manager_node --ros-args \
     -p snapshots.enabled:=true \
     -p snapshots.default_topics:="['/odom']"

**2. Start a node that publishes odometry:**

.. code-block:: bash

   ros2 topic pub /odom nav_msgs/msg/Odometry \
     "{pose: {pose: {position: {x: 1.5, y: 2.0}}}}" -r 10

**3. Report a fault (it will be confirmed immediately by default):**

.. code-block:: bash

   ros2 service call /fault_manager/report_fault ros2_medkit_msgs/srv/ReportFault \
     "{fault_code: 'NAV_ERROR', event_type: 0, severity: 2, \
       description: 'Navigation failed', source_id: '/nav_node'}"

**4. Query the captured snapshot:**

.. code-block:: bash

   curl http://localhost:8080/api/v1/faults/NAV_ERROR/snapshots

The response will contain the odometry data that was captured at the
moment the fault was confirmed.

Troubleshooting
---------------

**No snapshots captured**

- Verify ``snapshots.enabled`` is ``true``
- Check that configured topics exist and are publishing
- Increase ``snapshots.timeout_sec`` for slow-publishing topics
- Check fault manager logs for capture errors

**Empty topics object in response**

- The fault may have been cleared (snapshots are deleted on clear)
- No topics were configured for this fault code
- All configured topics timed out or exceeded size limit

**Snapshot data truncated**

- Message exceeded ``snapshots.max_message_size``
- Increase the limit or filter to smaller topics

**Wrong topics captured**

- Check topic resolution priority (fault_specific > patterns > default)
- Verify regex patterns in config file are correct

Rosbag Capture (Time-Window Recording)
--------------------------------------

In addition to JSON snapshots, you can enable **rosbag capture** for "black box"
style recording. This continuously buffers messages in memory and flushes them
to a bag file when a fault is confirmed.

**Key differences from JSON snapshots:**

.. list-table::
   :widths: 25 35 40
   :header-rows: 1

   * - Feature
     - JSON Snapshots
     - Rosbag Capture
   * - Data format
     - JSON (human-readable)
     - Binary (native ROS 2)
   * - Time coverage
     - Point-in-time (at confirmation)
     - Time window (before + after fault)
   * - Message fidelity
     - Converted to JSON
     - Original serialization preserved
   * - Playback
     - N/A
     - ``ros2 bag play``
   * - Default
     - Enabled
     - Disabled

Enabling Rosbag Capture
^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   ros2 run ros2_medkit_fault_manager fault_manager_node --ros-args \
     -p snapshots.rosbag.enabled:=true \
     -p snapshots.rosbag.duration_sec:=5.0 \
     -p snapshots.rosbag.duration_after_sec:=1.0

This captures 5 seconds of data **before** the fault and 1 second **after**.

Rosbag Configuration Options
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table::
   :widths: 35 15 50
   :header-rows: 1

   * - Parameter
     - Default
     - Description
   * - ``snapshots.rosbag.enabled``
     - ``false``
     - Enable rosbag capture
   * - ``snapshots.rosbag.duration_sec``
     - ``5.0``
     - Ring buffer duration (seconds before fault)
   * - ``snapshots.rosbag.duration_after_sec``
     - ``1.0``
     - Recording after fault confirmed
   * - ``snapshots.rosbag.topics``
     - ``"config"``
     - Topic selection: ``"config"``, ``"all"``, or ``"explicit"``
   * - ``snapshots.rosbag.format``
     - ``"sqlite3"``
     - Bag format: ``"sqlite3"`` or ``"mcap"``
   * - ``snapshots.rosbag.auto_cleanup``
     - ``true``
     - Delete bag when fault is cleared
   * - ``snapshots.rosbag.lazy_start``
     - ``false``
     - Start buffer only on PREFAILED state
   * - ``snapshots.rosbag.max_bag_size_mb``
     - ``50``
     - Maximum size per bag file
   * - ``snapshots.rosbag.max_total_storage_mb``
     - ``500``
     - Total storage limit for all bags

.. note::

   The ``"mcap"`` format requires ``rosbag2_storage_mcap`` to be installed.
   If not available, use ``"sqlite3"`` (default).

   .. code-block:: bash

      # Install MCAP support (optional)
      sudo apt install ros-${ROS_DISTRO}-rosbag2-storage-mcap

Downloading Rosbag Files
^^^^^^^^^^^^^^^^^^^^^^^^

**Via REST API:**

.. code-block:: bash

   # Download bag file
   curl -O -J http://localhost:8080/api/v1/faults/MOTOR_OVERHEAT/snapshots/bag

   # Inspect the downloaded bag
   ros2 bag info MOTOR_OVERHEAT_1735830000

   # Play back the bag
   ros2 bag play MOTOR_OVERHEAT_1735830000

**Via ROS 2 service:**

.. code-block:: bash

   ros2 service call /fault_manager/get_rosbag ros2_medkit_msgs/srv/GetRosbag \
     "{fault_code: 'MOTOR_OVERHEAT'}"

Example: Production Configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For production use with conservative resource usage:

.. code-block:: yaml

   # config/snapshots.yaml
   rosbag:
     enabled: true
     duration_sec: 3.0
     duration_after_sec: 0.5
     topics: "config"           # Use same topics as JSON snapshots
     lazy_start: true           # Save resources until fault detected
     format: "sqlite3"
     max_bag_size_mb: 25
     max_total_storage_mb: 200
     auto_cleanup: true

   # Exclude high-bandwidth topics
   # exclude_topics:
   #   - /camera/image_raw
   #   - /pointcloud

Example: Debugging Configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For development with maximum context:

.. code-block:: yaml

   rosbag:
     enabled: true
     duration_sec: 10.0         # 10 seconds before fault
     duration_after_sec: 2.0    # 2 seconds after
     topics: "config"
     lazy_start: false          # Always recording
     format: "sqlite3"
     storage_path: "/var/log/ros2_medkit/rosbags"
     max_bag_size_mb: 100
     max_total_storage_mb: 1000
     auto_cleanup: false        # Keep bags for analysis

See Also
--------

- :doc:`../requirements/specs/faults` - Fault API requirements
- `Gateway README <https://github.com/selfpatch/ros2_medkit/blob/main/src/ros2_medkit_gateway/README.md>`_ - REST API reference
- `config/snapshots.yaml <https://github.com/selfpatch/ros2_medkit/blob/main/src/ros2_medkit_fault_manager/config/snapshots.yaml>`_ - Full configuration reference
