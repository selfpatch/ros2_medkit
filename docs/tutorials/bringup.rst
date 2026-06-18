Single-command bringup
=======================

``bringup.launch.py`` starts the full local medkit stack - the gateway, the
fault_manager, and the generic fault bridges - as one process group, with no
manual topic/service wiring. It ships a shared params file that turns on the
headline value the conservative per-node defaults leave off (fault healing and
crash-safe black-box rosbag capture).

Install
-------

Build the workspace (the gateway brings the fault_manager and bridges in as
runtime dependencies):

.. code-block:: bash

   colcon build --packages-up-to ros2_medkit_gateway
   source install/setup.bash

Run
---

One command starts the whole stack:

.. code-block:: bash

   ros2 launch ros2_medkit_gateway bringup.launch.py

The gateway serves its REST API on ``http://127.0.0.1:8080`` by default. Common
launch arguments:

.. list-table::
   :header-rows: 1
   :widths: 35 15 50

   * - Argument
     - Default
     - Description
   * - ``params_file``
     - ``config/bringup_params.yaml``
     - Parameter file applied to the fault_manager (healing + black-box rosbag).
       Point this at your own file to override. The gateway and bridges run with
       their own configs; tune those via their launch args.
   * - ``server_host``
     - ``127.0.0.1``
     - Host to bind the gateway REST server (use ``0.0.0.0`` to expose it).
   * - ``server_port``
     - ``8080``
     - Gateway REST API port.
   * - ``enable_fault_manager``
     - ``true``
     - Start the fault_manager node.
   * - ``enable_log_bridge``
     - ``true``
     - Start the log_bridge (``/rosout`` -> faults).
   * - ``enable_action_status_bridge``
     - ``true``
     - Start the action_status_bridge (aborted action goals -> faults).
   * - ``enable_diagnostic_bridge``
     - ``false``
     - Start the diagnostic_bridge (``/diagnostics`` -> faults). Opt-in, for
       legacy ``diagnostic_updater`` publishers.

Verify
------

Induce a fault and see it surface through the gateway with a black-box bag, with
no further wiring. Report a fault against any node on your stack:

.. code-block:: bash

   ros2 service call /fault_manager/report_fault ros2_medkit_msgs/srv/ReportFault \
     "{fault_code: 'DEMO_FAULT', event_type: 0, severity: 2, description: 'bringup test', source_id: '/your_node'}"

Then query the gateway and the black-box:

.. code-block:: bash

   # The fault is visible via the gateway REST API
   curl http://127.0.0.1:8080/api/v1/faults

   # A non-empty black-box bag was captured at confirmation
   ros2 service call /fault_manager/get_rosbag ros2_medkit_msgs/srv/GetRosbag \
     "{fault_code: 'DEMO_FAULT'}"

With ``healing_enabled`` on, a recovery signal (e.g. an action transitioning to
SUCCEEDED) clears the fault automatically. Log faults have no recovery signal, so
``LOG_*`` faults do not auto-clear.

Bridge runtime cost and tuning
------------------------------

The bridges are the first thing this bringup starts by default, so size them for
your stack. Each bridge has ``include_only_*`` / ``exclude_*`` filters to scope
what it watches, and these knobs for cost:

- **log_bridge** subscribes ``/rosout`` and processes every log line, so its cost
  scales with log volume. Keep ``severity_floor`` at WARN (20) by default; raise
  it to ERROR (40) on chatty stacks. ``report_cooldown_sec`` already bounds
  ERROR/FATAL floods.
- **action_status_bridge** rescans the full ROS graph every ``rescan_period_sec``
  (default 2.0s) plus low-volume per-status processing. Raise ``rescan_period_sec``
  on large graphs.

Opt a bridge out entirely with its ``enable_*`` launch argument, e.g.
``ros2 launch ros2_medkit_gateway bringup.launch.py enable_log_bridge:=false``.

Multi-domain topology
---------------------

A robot that spans several ``ROS_DOMAIN_ID`` / ``RMW_IMPLEMENTATION`` values runs
**one bringup per domain**. The gateways are then federated into a single tree /
API / UI by peer aggregation (set ``aggregation.enabled`` plus ``peer_urls`` or
mDNS on the gateways). Each bringup must share its target stack's
``ROS_DOMAIN_ID``, ``RMW_IMPLEMENTATION`` and network to see it.
