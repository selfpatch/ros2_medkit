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

See Also
--------

- :doc:`../requirements/specs/faults` - Fault API requirements
- `Gateway README <https://github.com/selfpatch/ros2_medkit/blob/main/src/ros2_medkit_gateway/README.md>`_ - REST API reference
