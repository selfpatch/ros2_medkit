Sensor Demo
===========

This tutorial walks through the built-in sensor demo nodes to explore
ros2_medkit's data access, operations, configurations, and fault management.

.. contents:: Table of Contents
   :local:
   :depth: 2

Overview
--------

The sensor demo includes automotive-style nodes:

- **Temperature Sensor** — Publishes engine temperature readings
- **Brake Actuator** — Accepts brake pressure commands
- **LiDAR Sensor** — Simulates a laser scanner with configurable parameters
- **Calibration Service** — Provides calibration operations

Starting the Demo
-----------------

Open three terminals and source your workspace in each:

.. code-block:: bash

   source ~/ros2_medkit_ws/install/setup.bash

**Terminal 1 — Gateway:**

.. code-block:: bash

   ros2 launch ros2_medkit_gateway gateway.launch.py

**Terminal 2 — Demo Nodes:**

.. code-block:: bash

   ros2 launch ros2_medkit_gateway demo_nodes.launch.py

.. figure:: /_static/images/20_sensor_demo_run_terminal.png
   :alt: Demo nodes terminal output
   :align: center
   :width: 100%

   Terminal showing demo nodes starting up.

**Terminal 3 — Fault Manager (optional):**

.. code-block:: bash

   ros2 run ros2_medkit_fault_manager fault_manager_node

Required for testing the Faults API.

Exploring with Web UI
---------------------

Start the web UI and connect to the gateway:

.. code-block:: bash

   docker run -p 8081:80 ghcr.io/selfpatch/sovd_web_ui:latest

Open http://localhost:8081 and connect to ``http://localhost:8080``.

.. figure:: /_static/images/21_sensor_demo_ui_view.png
   :alt: Sensor demo in web UI
   :align: center
   :width: 100%

   Web UI showing sensor demo entity hierarchy.

You'll see the demo nodes organized into areas based on their namespaces.

Working with Data
-----------------

Navigate to a sensor component and click the **data** folder to see topics.
Click on a topic to view its current value:

.. figure:: /_static/images/06_topic_data_view.png
   :alt: Topic data view
   :align: center
   :width: 100%

   Temperature reading from the sensor.

Using curl:

.. code-block:: bash

   # List all data from temp_sensor
   curl http://localhost:8080/api/v1/components/temp_sensor/data

   # Get specific topic
   curl http://localhost:8080/api/v1/components/temp_sensor/data/powertrain%2Fengine%2Ftemperature

Calling Operations
------------------

Navigate to the **operations** folder to see available services:

.. figure:: /_static/images/07_operations_panel.png
   :alt: Operations panel
   :align: center
   :width: 100%

   Available operations for a component.

Call the calibration service:

.. code-block:: bash

   curl -X POST http://localhost:8080/api/v1/components/calibration/operations/calibrate/executions \
     -H "Content-Type: application/json" \
     -d '{}'

.. figure:: /_static/images/08_operations_execution.png
   :alt: Operation execution
   :align: center
   :width: 100%

   Service call result showing success response.

Managing Parameters
-------------------

Click on the **configurations** folder to see ROS 2 parameters:

.. figure:: /_static/images/09_configurations_list.png
   :alt: Configurations list
   :align: center
   :width: 100%

   Parameters for the temperature sensor.

Change the publish rate:

.. code-block:: bash

   # Get current value
   curl http://localhost:8080/api/v1/components/temp_sensor/configurations/publish_rate

   # Set new value
   curl -X PUT http://localhost:8080/api/v1/components/temp_sensor/configurations/publish_rate \
     -H "Content-Type: application/json" \
     -d '{"value": 5.0}'

.. figure:: /_static/images/10_configuration_edit.png
   :alt: Configuration edit
   :align: center
   :width: 100%

   Editing a parameter value in the web UI.

Working with Faults
-------------------

The LiDAR sensor demo starts with intentionally invalid parameters that
generate faults:

- ``LIDAR_RANGE_INVALID`` (ERROR): min_range > max_range
- ``LIDAR_FREQ_UNSUPPORTED`` (WARN): scan_frequency > 20.0 Hz
- ``LIDAR_CALIBRATION_REQUIRED`` (INFO): sensor not calibrated

.. figure:: /_static/images/18_faults_injected_dashboard.png
   :alt: Faults dashboard
   :align: center
   :width: 100%

   Dashboard showing active faults.

View faults:

.. code-block:: bash

   curl http://localhost:8080/api/v1/components/lidar_sensor/faults

.. figure:: /_static/images/19_faults_injected_app_view.png
   :alt: Faults in app view
   :align: center
   :width: 100%

   Fault details in the entity view.

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

See Also
--------

- :doc:`/getting_started` — Basic gateway setup
- :doc:`/api/rest` — REST API reference
- :doc:`demo-turtlebot3` — TurtleBot3 simulation demo
