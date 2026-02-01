Sensor Diagnostics Demo
=======================

This tutorial walks through the **sensor_diagnostics** demo — a lightweight
demonstration of ros2_medkit's monitoring, configuration, and fault detection
capabilities.

.. contents:: Table of Contents
   :local:
   :depth: 2

Overview
--------

The sensor diagnostics demo showcases ros2_medkit with simulated sensor nodes:

- **LiDAR Simulator** — 2D laser scanner with fault injection
- **Camera Simulator** — RGB camera with noise and brightness control
- **IMU Simulator** — 9-DOF inertial measurement unit
- **GPS Simulator** — GPS receiver with position drift
- **Anomaly Detector** — Monitors sensor data for faults

**Key Features:**

- Runs anywhere — no Gazebo, no GPU required
- Fast startup — seconds vs minutes
- Docker-based deployment with web UI included
- Dual fault reporting paths (legacy diagnostics + modern direct)
- Runtime fault injection via REST API

Prerequisites
-------------

- Docker and Docker Compose installed
- Git (to clone the demo repository)

Starting the Demo
-----------------

Clone the demo repository and run the startup script:

.. code-block:: bash

   git clone https://github.com/selfpatch/selfpatch_demos.git
   cd selfpatch_demos/demos/sensor_diagnostics

   # Start the demo (daemon mode)
   ./run-demo.sh

.. figure:: /_static/images/20_sensor_demo_run_terminal.png
   :alt: Demo startup terminal output
   :align: center
   :width: 600px

   Terminal showing demo services starting up.

The script will build and start Docker containers with:

- ros2_medkit gateway (REST API on port 8080)
- sovd_web_ui (Web interface on port 3000)
- Simulated sensor nodes (lidar, camera, imu, gps)
- Anomaly detector for fault monitoring
- Diagnostic bridge for legacy fault reporting

**Startup Options:**

.. code-block:: bash

   ./run-demo.sh --attached    # Run in foreground with logs
   ./run-demo.sh --update      # Pull latest images
   ./run-demo.sh --no-cache    # Build without cache

Exploring the Demo
------------------

Open the web UI at http://localhost:3000 (automatically connects to gateway).

.. figure:: /_static/images/21_sensor_demo_ui_view.png
   :alt: Sensor demo in web UI
   :align: center
   :width: 600px

   Web UI showing sensor demo entity hierarchy.

The demo exposes entities organized by namespace:

- ``/sensors`` — Sensor simulator nodes (lidar, camera, imu, gps)
- ``/processing`` — Anomaly detector
- ``/bridge`` — Diagnostic bridge
- ``/diagnostics`` — ros2_medkit gateway

Interactive API exploration:

.. code-block:: bash

   # Run the interactive check script
   ./check-demo.sh

Reading Sensor Data
-------------------

Navigate to a sensor app in the web UI and explore the **data** folder to see published topics.

Query sensor data via REST API:

.. code-block:: bash

   # Get LiDAR scan
   curl http://localhost:8080/api/v1/apps/lidar-sim/data/scan | jq '.ranges[:5]'

   # Get IMU data
   curl http://localhost:8080/api/v1/apps/imu-sim/data/imu | jq '.linear_acceleration'

   # Get GPS fix
   curl http://localhost:8080/api/v1/apps/gps-sim/data/fix | jq '{lat: .latitude, lon: .longitude}'

   # Get camera image info
   curl http://localhost:8080/api/v1/apps/camera-sim/data/image | jq '{width, height, encoding}'

Managing Configurations
-----------------------

Click on the **configurations** folder in the web UI to see ROS 2 parameters for each sensor.

View and modify sensor parameters:

.. code-block:: bash

   # List all LiDAR configurations
   curl http://localhost:8080/api/v1/apps/lidar-sim/configurations | jq

   # Get specific parameter
   curl http://localhost:8080/api/v1/apps/lidar-sim/configurations/noise_stddev | jq

   # Change scan rate
   curl -X PUT http://localhost:8080/api/v1/apps/lidar-sim/configurations/scan_rate \
     -H "Content-Type: application/json" \
     -d '{"value": 20.0}'

**Key Parameters:**

- ``scan_rate`` / ``rate`` — Publishing frequency (Hz)
- ``noise_stddev`` / ``noise_level`` — Sensor noise magnitude
- ``drift_rate`` — Gradual sensor drift
- ``failure_probability`` — Probability of sensor timeout
- ``inject_nan`` / ``inject_black_frames`` — Fault injection flags

Working with Faults
-------------------

The LiDAR sensor demo starts with intentionally invalid parameters that
generate faults:

- ``LIDAR_RANGE_INVALID`` (ERROR): min_range > max_range
- ``LIDAR_FREQ_UNSUPPORTED`` (WARN): scan_frequency > 20.0 Hz
- ``LIDAR_CALIBRATION_REQUIRED`` (INFO): sensor not calibrated

View faults:

.. code-block:: bash

   curl http://localhost:8080/api/v1/components/lidar_sensor/faults

**Fix the faults:**

1. Correct the parameters:

   .. code-block:: bash

      # Fix min_range
      curl -X PUT http://localhost:8080/api/v1/components/lidar_sensor/configurations/min_range \
        -H "Content-Type: application/json" -d '{"value": 0.1}'

      # Fix max_range
      curl -X PUT http://localhost:8080/api/v1/components/lidar_sensor/configurations/max_range \
        -H "Content-Type: application/json" -d '{"value": 10.0}'

      # Fix scan_frequency
      curl -X PUT http://localhost:8080/api/v1/components/lidar_sensor/configurations/scan_frequency \
        -H "Content-Type: application/json" -d '{"value": 10.0}'

2. Clear the faults:

   .. code-block:: bash

      curl -X DELETE http://localhost:8080/api/v1/components/lidar_sensor/faults/LIDAR_RANGE_INVALID
      curl -X DELETE http://localhost:8080/api/v1/components/lidar_sensor/faults/LIDAR_FREQ_UNSUPPORTED

3. Run calibration and clear the calibration fault:

   .. code-block:: bash

      curl -X POST http://localhost:8080/api/v1/components/lidar_sensor/operations/calibrate/executions \
        -H "Content-Type: application/json" -d '{}'

      curl -X DELETE http://localhost:8080/api/v1/components/lidar_sensor/faults/LIDAR_CALIBRATION_REQUIRED

Stopping the Demo
-----------------

To stop all services:

.. code-block:: bash

   ./stop-demo.sh

Or manually:

.. code-block:: bash

   docker compose down

Architecture
------------

The demo implements the following architecture:

.. code-block:: text

   Sensor Diagnostics Demo
   ├── /sensors                    # Simulated sensor nodes
   │   ├── lidar_sim              # 2D LiDAR (Legacy path)
   │   ├── camera_sim             # RGB camera (Legacy path)
   │   ├── imu_sim                # 9-DOF IMU (Modern path)
   │   └── gps_sim                # GPS receiver (Modern path)
   ├── /processing                 # Data processing
   │   └── anomaly_detector       # Fault detection
   ├── /bridge                     # Diagnostic conversion
   │   └── diagnostic_bridge      # /diagnostics → FaultManager
   └── /diagnostics                # Monitoring
       └── ros2_medkit_gateway    # REST API gateway

**Comparison with TurtleBot3 Demo:**

.. list-table::
   :header-rows: 1
   :widths: 30 35 35

   * - Feature
     - Sensor Demo
     - TurtleBot3 Demo
   * - Docker image size
     - ~500 MB
     - ~4 GB
   * - Startup time
     - ~5 seconds
     - ~60 seconds
   * - GPU required
     - No
     - Recommended
   * - CI compatible
     - Yes
     - Difficult
   * - Focus
     - Diagnostics & faults
     - Navigation & robotics

See Also
--------

- :doc:`/getting_started` — Basic gateway setup
- :doc:`/api/rest` — REST API reference
- :doc:`/config/fault-manager` — Fault Manager configuration
- :doc:`demo-turtlebot3` — TurtleBot3 simulation demo
- `selfpatch_demos <https://github.com/selfpatch/selfpatch_demos>`_ — Demo repository
