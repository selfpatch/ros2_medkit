TurtleBot3 Demo
===============

This tutorial shows how to connect ros2_medkit to a TurtleBot3 simulation,
demonstrating integration with a realistic robotics system.

.. contents:: Table of Contents
   :local:
   :depth: 2

Overview
--------

TurtleBot3 is a popular ROS 2 robot platform used for education and research.
This demo connects ros2_medkit to TurtleBot3 running in Gazebo simulation with
Nav2 navigation stack, exposing:

- **Sensor data**: LiDAR scans, odometry, camera images
- **Operations**: Navigation goals, robot control
- **Configurations**: Node parameters
- **Transforms**: TF2 coordinate frames

Prerequisites
-------------

- TurtleBot3 packages installed
- Gazebo simulation environment
- Nav2 navigation stack (optional)

.. code-block:: bash

   sudo apt install ros-jazzy-turtlebot3* ros-jazzy-nav2-bringup

Starting the Simulation
-----------------------

**Terminal 1 — Gazebo Simulation:**

.. code-block:: bash

   export TURTLEBOT3_MODEL=waffle
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

.. figure:: /_static/images/16_turtlebot_gazebo.png
   :alt: TurtleBot3 in Gazebo
   :align: center
   :width: 600px

   TurtleBot3 Waffle in Gazebo simulation.

**Terminal 2 — ros2_medkit Gateway:**

.. code-block:: bash

   ros2 launch ros2_medkit_gateway gateway.launch.py

.. figure:: /_static/images/17_turtlebot_run_demo_terminal.png
   :alt: Gateway terminal
   :align: center
   :width: 600px

   Gateway discovering TurtleBot3 nodes.

**Terminal 3 — Navigation (optional):**

.. code-block:: bash

   ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true

Exploring the System
--------------------

Connect the web UI to see all discovered entities:

.. code-block:: bash

   docker run -p 8081:80 ghcr.io/selfpatch/sovd_web_ui:latest

Open http://localhost:8081 and connect to ``http://localhost:8080``.

You'll see areas organized by namespace:

- ``/`` (root) — Core robot nodes
- ``/gazebo`` — Simulation interface
- ``/diff_drive_controller`` — Motor control
- ``/joint_state_broadcaster`` — Joint states

Querying via API:

.. code-block:: bash

   curl http://localhost:8080/api/v1/areas | jq

.. figure:: /_static/images/13_curl_areas_turtlebot3.png
   :alt: Areas response
   :align: center
   :width: 600px

   Areas discovered from TurtleBot3 system.

Reading Sensor Data
-------------------

Get odometry data:

.. code-block:: bash

   curl http://localhost:8080/api/v1/components/diff_drive_controller/data/odom | jq

.. figure:: /_static/images/14_curl_topic_odom.png
   :alt: Odometry data
   :align: center
   :width: 600px

   Real-time odometry from the robot.

Via the web UI, navigate to a component and click the **data** folder to see topics:

.. figure:: /_static/images/06_topic_data_view.png
   :alt: Topic data view
   :align: center
   :width: 600px

   Viewing topic data in the web UI.

Get LiDAR scan:

.. code-block:: bash

   curl http://localhost:8080/api/v1/components/turtlebot3_node/data/scan | jq '.data.ranges[:10]'

Controlling the Robot
---------------------

Navigate to the **operations** folder to see available services:

.. figure:: /_static/images/07_operations_panel.png
   :alt: Operations panel
   :align: center
   :width: 600px

   Available operations for a component.

Publish velocity commands:

.. code-block:: bash

   # Move forward
   curl -X PUT http://localhost:8080/api/v1/components/diff_drive_controller/data/cmd_vel \
     -H "Content-Type: application/json" \
     -d '{"linear": {"x": 0.2, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}'

   # Rotate
   curl -X PUT http://localhost:8080/api/v1/components/diff_drive_controller/data/cmd_vel \
     -H "Content-Type: application/json" \
     -d '{"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.5}}'

   # Stop
   curl -X PUT http://localhost:8080/api/v1/components/diff_drive_controller/data/cmd_vel \
     -H "Content-Type: application/json" \
     -d '{"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}'

.. figure:: /_static/images/08_operations_execution.png
   :alt: Operation execution
   :align: center
   :width: 600px

   Result of executing an operation.

Managing Parameters
-------------------

Click on the **configurations** folder to see ROS 2 parameters:

.. figure:: /_static/images/09_configurations_list.png
   :alt: Configurations list
   :align: center
   :width: 600px

   Parameters for TurtleBot3 components.

Change a parameter value using the API:

.. code-block:: bash

   # Example: Change a parameter
   curl -X PUT http://localhost:8080/api/v1/components/turtlebot3_node/configurations/use_sim_time \
     -H "Content-Type: application/json" \
     -d '{"value": true}'

.. figure:: /_static/images/10_configuration_edit.png
   :alt: Configuration edit
   :align: center
   :width: 600px

   Editing a parameter value in the web UI.

Working with Faults
-------------------

The gateway automatically discovers diagnostic information from nodes.
View faults in the web UI dashboard:

.. figure:: /_static/images/18_faults_injected_dashboard.png
   :alt: Faults dashboard
   :align: center
   :width: 600px

   Dashboard showing active faults.

You can also view faults for specific components:

.. figure:: /_static/images/19_faults_injected_app_view.png
   :alt: Faults in app view
   :align: center
   :width: 600px

   Fault details in the entity view.

Query faults via API:

.. code-block:: bash

   curl http://localhost:8080/api/v1/components/turtlebot3_node/faults

Docker Deployment
-----------------

For containerized deployment, see the `selfpatch_demos <https://github.com/selfpatch/selfpatch_demos>`_
repository which provides ready-to-use Docker Compose configurations:

.. code-block:: bash

   git clone https://github.com/selfpatch/selfpatch_demos.git
   cd selfpatch_demos/demos/turtlebot3_nav2

   # Start all services
   docker compose up

This launches:

- TurtleBot3 Gazebo simulation
- Nav2 navigation stack
- ros2_medkit gateway
- sovd_web_ui

See Also
--------

- :doc:`demo-sensor` — Built-in sensor demo
- :doc:`/tutorials/docker` — Docker deployment guide
- `selfpatch_demos <https://github.com/selfpatch/selfpatch_demos>`_ — Integration demos repository
