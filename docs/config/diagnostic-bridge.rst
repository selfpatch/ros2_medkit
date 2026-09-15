Diagnostic Bridge Configuration
================================

The ``ros2_medkit_diagnostic_bridge`` node converts standard ROS 2 ``/diagnostics`` messages
to fault events, providing a migration path for existing diagnostic infrastructure.

.. contents:: Table of Contents
   :local:
   :depth: 2

Overview
--------

The diagnostic bridge:

1. Subscribes to ``/diagnostics`` (or custom topic)
2. Maps ``DiagnosticStatus`` names to fault codes
3. Reports faults to the FaultManager via ``ReportFault`` service

**Status Mapping:**

.. list-table::
   :header-rows: 1
   :widths: 25 25 50

   * - Diagnostic Level
     - Fault Event
     - Severity
   * - OK
     - PASSED
     - (healing event)
   * - WARN
     - FAILED
     - SEVERITY_WARN (1)
   * - ERROR
     - FAILED
     - SEVERITY_ERROR (2)
   * - STALE
     - FAILED
     - ``stale_severity``, SEVERITY_CRITICAL (3) by default

.. warning::

   This table previously documented STALE as SEVERITY_ERROR. The bridge has always sent
   SEVERITY_CRITICAL, and CRITICAL **bypasses debounce** in the fault manager, so a STALE
   status confirms a CRITICAL fault on its first sample.

   That matters because STALE is the one level a node can reach by design: a GPS goes STALE
   in every tunnel, an IMU reports covariance -1 while it settles. Left unconfigured, every
   such outage is a confirmed CRITICAL fault. Use ``stale_severity`` to set the level, or
   ``stale_severity_overrides`` to name just the sources that go STALE on purpose; those
   statuses then debounce like any other.

Parameters
----------

.. code-block:: yaml

   diagnostic_bridge:
     ros__parameters:
       diagnostics_topic: "/diagnostics"   # Topic to subscribe to
       auto_generate_codes: true           # Auto-generate fault codes from names
       keyvalue_codes: ["fault_code"]      # Take the code from these key-value keys
       stale_severity: "WARN"              # What a STALE status reports at
       # Per-source, longest matching prefix wins:
       "stale_severity_overrides.gps": "WARN"

.. list-table::
   :header-rows: 1
   :widths: 30 15 55

   * - Parameter
     - Default
     - Description
   * - ``diagnostics_topic``
     - ``/diagnostics``
     - Topic to subscribe for ``DiagnosticArray`` messages.
   * - ``auto_generate_codes``
     - ``true``
     - Automatically generate fault codes from diagnostic names when no explicit
       mapping exists.
   * - ``keyvalue_codes``
     - ``[]``
     - Keys to look for inside the ``DiagnosticStatus`` key-value pairs. The
       first key in this list that the message carries supplies the fault code
       as its value, so a publisher can name its own code instead of relying on
       a mapping. Checked after ``name_to_code`` and before auto-generation.
       Empty strings in the list are ignored.
   * - ``stale_severity``
     - ``CRITICAL``
     - Severity a STALE status reports at, one of ``INFO``, ``WARN``, ``ERROR``,
       ``CRITICAL`` (case-insensitive). Applies to STALE only: the other levels are
       facts about the status, not deployment decisions. A name that does not parse
       is reported and ``CRITICAL`` is used.
   * - ``stale_severity_overrides.<name>``
     - ``-``
     - Severity for STALE statuses whose name starts with ``<name>``. Diagnostic names
       are conventionally ``<component>: <check>``, so a component prefix covers every
       check it publishes. The **longest matching prefix** wins. An override that does
       not parse is reported and **ignored**, leaving ``stale_severity`` in force -
       applying ``CRITICAL`` to a typo would restore the immediate-confirm behaviour the
       operator was configuring their way out of.

Evidence
--------

A FAILED report carries the ``DiagnosticStatus`` key-values to the fault manager, which keeps
them in the fault's freeze frame and serves them from
``GET /api/v1/apps/{app}/faults/{code}``:

.. code-block:: console

   $ curl -s localhost:8080/api/v1/apps/sensor_fusion/faults/FUSION_DIVERGED | jq '.environment_data.snapshots[0].data | fromjson'
   {
     "x-reported": {
       "rejected_fixes": "37",
       "nis": "0.03"
     }
   }

A node that publishes outlier counts and gate statistics has already computed why it is
unhappy; before this the bridge read those values only to pick a fault code and dropped the
rest, so the fault record said a node complained but not what it saw.

Evidence is written on **every** FAILED report, not only the one that confirms the fault, so a
code that keeps approaching confirmation without reaching it still carries the numbers behind
its near misses.

The frame's other keys are topic names sampled by the fault manager, always fully qualified
and so always starting with ``/``. ``x-reported`` cannot collide with one, which is what lets
a reader tell a value the reporter asserted from one the fault manager sampled.

.. note::

   Evidence is bounded per fault code: at most 32 entries, and values longer than 512
   characters are dropped whole rather than truncated, because half a number read back later
   is worse than a logged absence. Drops are reported with a throttled warning and the fault
   is recorded either way. A key already stored can always be updated, so a steady reporter
   at the bound can still refresh its own numbers.

Custom Fault Code Mappings
--------------------------

Map specific diagnostic names to custom fault codes using the ``name_to_code`` parameter prefix:

.. code-block:: yaml

   diagnostic_bridge:
     ros__parameters:
       diagnostics_topic: "/diagnostics"
       auto_generate_codes: true
       name_to_code:
         motor_temp: "MOTOR_OVERHEAT"
         battery_level: "LOW_BATTERY"
         camera_driver: "CAMERA_FAILURE"

Or via command line:

.. code-block:: bash

   ros2 run ros2_medkit_diagnostic_bridge diagnostic_bridge \
     --ros-args \
     -p "name_to_code.motor_temp:=MOTOR_OVERHEAT" \
     -p "name_to_code.battery_level:=LOW_BATTERY"

Auto-Generated Fault Codes
~~~~~~~~~~~~~~~~~~~~~~~~~~

When ``auto_generate_codes: true`` and no explicit mapping exists, fault codes are
generated from the diagnostic name:

1. Convert to uppercase
2. Replace spaces, slashes, and dashes with underscores
3. Remove non-alphanumeric characters (except underscore)
4. Prepend ``DIAG_`` prefix

**Examples:**

.. list-table::
   :header-rows: 1
   :widths: 50 50

   * - Diagnostic Name
     - Generated Fault Code
   * - ``motor_temp``
     - ``DIAG_MOTOR_TEMP``
   * - ``/camera/driver``
     - ``DIAG_CAMERA_DRIVER``
   * - ``Battery Level Monitor``
     - ``DIAG_BATTERY_LEVEL_MONITOR``

Launch File Configuration
-------------------------

Example launch file:

.. code-block:: python

   from launch import LaunchDescription
   from launch_ros.actions import Node

   def generate_launch_description():
       return LaunchDescription([
           Node(
               package='ros2_medkit_diagnostic_bridge',
               executable='diagnostic_bridge_node',
               name='diagnostic_bridge',
               parameters=[{
                   'diagnostics_topic': '/diagnostics',
                   'auto_generate_codes': True,
                   'name_to_code': {
                       'motor_temp': 'MOTOR_OVERHEAT',
                       'battery_level': 'LOW_BATTERY',
                   }
               }],
           ),
       ])

Integration with FaultManager
-----------------------------

The diagnostic bridge requires a running FaultManager to report faults. Ensure the
FaultManager is started before the bridge:

.. code-block:: bash

   # Terminal 1: Start FaultManager
   ros2 run ros2_medkit_fault_manager fault_manager_node

   # Terminal 2: Start Diagnostic Bridge
   ros2 run ros2_medkit_diagnostic_bridge diagnostic_bridge_node

Or use a combined launch file:

.. code-block:: python

   from launch import LaunchDescription
   from launch_ros.actions import Node

   def generate_launch_description():
       return LaunchDescription([
           Node(
               package='ros2_medkit_fault_manager',
               executable='fault_manager_node',
               name='fault_manager',
           ),
           Node(
               package='ros2_medkit_diagnostic_bridge',
               executable='diagnostic_bridge_node',
               name='diagnostic_bridge',
           ),
       ])

Migration Strategy
------------------

For transitioning from standard ROS 2 diagnostics to direct fault reporting:

1. **Phase 1**: Deploy diagnostic bridge alongside existing diagnostics infrastructure
2. **Phase 2**: Create explicit mappings for important diagnostics
3. **Phase 3**: Migrate critical nodes to direct FaultReporter usage
4. **Phase 4**: Disable auto_generate_codes, rely only on explicit mappings
5. **Phase 5**: Remove diagnostic bridge when all nodes use FaultReporter

.. seealso::

   :doc:`/design/ros2_medkit_fault_reporter/index` - Direct fault reporting with FaultReporter

See Also
--------

- :doc:`fault-manager` - FaultManager configuration
- :doc:`/design/ros2_medkit_fault_reporter/index` - Direct fault reporting guide
- :doc:`/api/messages` - ReportFault service definition
- :doc:`/design/ros2_medkit_diagnostic_bridge/index` - Bridge architecture
