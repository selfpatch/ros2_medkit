Quick Start Demo
================

This tutorial demonstrates ros2_medkit using the built-in demo nodes — the
fastest way to explore the gateway without Docker or external dependencies.

.. contents:: Table of Contents
   :local:
   :depth: 2

Overview
--------

The quick start demo includes simple automotive-style nodes built into the
gateway package for testing and demonstration:

**Sensors:**

- Engine temperature sensor (``/powertrain/engine/temp_sensor``)
- Engine RPM sensor (``/powertrain/engine/rpm_sensor``)
- Brake pressure sensor (``/chassis/brakes/pressure_sensor``)
- Door status sensor (``/body/door/front_left/status_sensor``)
- LIDAR sensor with fault injection (``/perception/lidar/lidar_sensor``)

**Actuators:**

- Brake actuator (``/chassis/brakes/actuator``)
- Light controller (``/body/lights/controller``)

**Operations:**

- Calibration service (synchronous)
- Long calibration action (asynchronous)
- LIDAR calibration service

**Key Features:**

- No Docker required — runs directly in your ROS 2 workspace
- Fast startup — seconds, not minutes
- Intentional fault scenarios for testing
- Namespace-based area organization (powertrain, chassis, body, perception)
- Examples of sync services and async actions

Prerequisites
-------------

- ROS 2 Jazzy installed and sourced
- ros2_medkit built in your workspace

.. code-block:: bash

   # If not already built
   cd ~/ros2_medkit_ws
   source /opt/ros/jazzy/setup.bash
   colcon build --symlink-install
   source install/setup.bash

Starting the Demo
-----------------

Open two terminals and source your workspace in each:

.. code-block:: bash

   cd ~/ros2_medkit_ws
   source install/setup.bash

**Terminal 1 — Gateway:**

.. code-block:: bash

   ros2 launch ros2_medkit_gateway gateway.launch.py

**Terminal 2 — Demo Nodes:**

.. code-block:: bash

   ros2 launch ros2_medkit_gateway demo_nodes.launch.py

The demo nodes will start and begin publishing data. You should see log
messages indicating nodes are running.

Exploring with ROS 2 CLI
-------------------------

Verify the demo is running using standard ROS 2 tools:

.. code-block:: bash

   # List all running nodes
   ros2 node list

   # List all topics
   ros2 topic list

   # Echo temperature data
   ros2 topic echo /powertrain/engine/temperature

   # Call calibration service
   ros2 service call /powertrain/engine/calibrate std_srvs/srv/Trigger

Exploring via REST API
----------------------

The gateway exposes all demo nodes through its REST API:

.. code-block:: bash

   # Check gateway health
   curl http://localhost:8080/api/v1/health | jq

   # List all areas
   curl http://localhost:8080/api/v1/areas | jq

   # List components in powertrain area
   curl http://localhost:8080/api/v1/areas/powertrain/components | jq

**Read Sensor Data:**

.. code-block:: bash

   # Get engine temperature
   curl http://localhost:8080/api/v1/apps/temp_sensor/data/temperature | jq

   # Get RPM reading
   curl http://localhost:8080/api/v1/apps/rpm_sensor/data/rpm | jq

   # Get brake pressure
   curl http://localhost:8080/api/v1/apps/pressure_sensor/data/pressure | jq

**Call Operations:**

.. code-block:: bash

   # Calibrate engine sensors (sync service)
   curl -X POST http://localhost:8080/api/v1/apps/calibration/operations/calibrate/executions \
     -H "Content-Type: application/json" \
     -d '{}'

   # Start long calibration (async action)
   curl -X POST http://localhost:8080/api/v1/apps/long_calibration/operations/calibrate/executions \
     -H "Content-Type: application/json" \
     -d '{"duration_seconds": 5}'

**Control Actuators:**

.. code-block:: bash

   # Set brake pressure
   curl -X PUT http://localhost:8080/api/v1/apps/actuator/data/brake_command \
     -H "Content-Type: application/json" \
     -d '{"pressure": 50.0}'

   # Control lights
   curl -X PUT http://localhost:8080/api/v1/apps/controller/data/light_command \
     -H "Content-Type: application/json" \
     -d '{"headlights": true, "brake_lights": false}'

Exploring with Web UI
---------------------

For a visual interface, run the web UI:

.. code-block:: bash

   docker run -p 8081:80 ghcr.io/selfpatch/sovd_web_ui:latest

Open http://localhost:8081 and connect to ``http://localhost:8080``.

You'll see the hierarchical structure:

- **powertrain** area → engine components (temp_sensor, rpm_sensor, calibration)
- **chassis** area → brake components (pressure_sensor, actuator)
- **body** area → door and light components
- **perception** area → lidar_sensor

Click on any component to explore:

- **data/** — Published topics with live values
- **operations/** — Available services and actions
- **configurations/** — ROS 2 parameters

Working with Faults
-------------------

The LIDAR sensor is configured with intentionally invalid parameters to
demonstrate fault detection:

**Initial Faults:**

1. ``LIDAR_RANGE_INVALID`` (ERROR) — min_range (10.0) > max_range (5.0)
2. ``LIDAR_FREQ_UNSUPPORTED`` (WARN) — scan_frequency (25.0) exceeds 20.0 Hz limit
3. ``LIDAR_CALIBRATION_REQUIRED`` (INFO) — sensor not calibrated

**View Faults:**

.. code-block:: bash

   # List all faults
   curl http://localhost:8080/api/v1/faults | jq

   # View LIDAR faults
   curl http://localhost:8080/api/v1/apps/lidar_sensor/faults | jq

**Fix the Faults:**

1. Correct the range parameters:

   .. code-block:: bash

      # Fix min_range
      curl -X PUT http://localhost:8080/api/v1/apps/lidar_sensor/configurations/min_range \
        -H "Content-Type: application/json" \
        -d '{"value": 0.1}'

      # Fix max_range
      curl -X PUT http://localhost:8080/api/v1/apps/lidar_sensor/configurations/max_range \
        -H "Content-Type: application/json" \
        -d '{"value": 10.0}'

2. Correct the scan frequency:

   .. code-block:: bash

      curl -X PUT http://localhost:8080/api/v1/apps/lidar_sensor/configurations/scan_frequency \
        -H "Content-Type: application/json" \
        -d '{"value": 10.0}'

3. Run calibration:

   .. code-block:: bash

      curl -X POST http://localhost:8080/api/v1/apps/lidar_sensor/operations/calibrate/executions \
        -H "Content-Type: application/json" \
        -d '{}'

4. Verify faults are cleared:

   .. code-block:: bash

      curl http://localhost:8080/api/v1/apps/lidar_sensor/faults | jq

Demo Architecture
-----------------

The demo implements an automotive-inspired namespace hierarchy:

.. code-block:: text

   Root (/)
   ├── powertrain/
   │   └── engine/
   │       ├── temp_sensor      # Engine temperature publisher
   │       ├── rpm_sensor       # RPM publisher
   │       ├── calibration      # Sync calibration service
   │       └── long_calibration # Async calibration action
   ├── chassis/
   │   └── brakes/
   │       ├── pressure_sensor  # Brake pressure publisher
   │       └── actuator         # Brake command subscriber
   ├── body/
   │   ├── door/
   │   │   └── front_left/
   │   │       └── status_sensor # Door status publisher
   │   └── lights/
   │       └── controller       # Light command subscriber
   └── perception/
       └── lidar/
           └── lidar_sensor     # Laser scan with faults

**Features Demonstrated:**

- **Data Access** — Read sensor topics, write actuator commands
- **Operations** — Synchronous services and asynchronous actions
- **Configurations** — ROS 2 parameter management
- **Fault Management** — Detection, reporting, and clearing
- **Area Hierarchy** — Namespace-based organization

Next Steps
----------

After exploring the quick start demo, try more advanced scenarios:

1. **Sensor Diagnostics Demo** (:doc:`demo-sensor`)

   - Docker-based deployment
   - Multiple sensor types (LIDAR, camera, IMU, GPS)
   - Fault injection scripts
   - Dual fault reporting paths

2. **TurtleBot3 Demo** (:doc:`demo-turtlebot3`)

   - Full robot simulation in Gazebo
   - Nav2 navigation integration
   - Real-world complexity

See Also
--------

- :doc:`/getting_started` — Installation and setup
- :doc:`/api/rest` — Complete REST API reference
- :doc:`/config/server` — Gateway configuration
- :doc:`demo-sensor` — Sensor diagnostics demo
- :doc:`demo-turtlebot3` — TurtleBot3 simulation demo
