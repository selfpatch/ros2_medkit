Customizing the Entity Hierarchy
=================================

This tutorial explains how ros2_medkit organizes ROS 2 entities and
how to customize the hierarchy for your robot.

.. contents:: Table of Contents
   :local:
   :depth: 2

Understanding the Entity Model
------------------------------

ros2_medkit uses a hierarchical entity model inspired by SOVD
(Service-Oriented Vehicle Diagnostics):

.. code-block:: text

   System
   └── Areas (logical/physical domains)
       └── Components (nodes, devices)
           ├── Data (topics)
           ├── Operations (services, actions)
           └── Configurations (parameters)

**Areas** represent logical or physical domains of your robot:

- ``/perception`` - Sensors and perception processing
- ``/navigation`` - Path planning and control
- ``/manipulation`` - Arm and gripper control
- ``/safety`` - Emergency stops and safety systems

**Components** are individual software or hardware units:

- ROS 2 nodes
- Virtual components from topic namespaces

Default Namespace Mapping
-------------------------

By default, ros2_medkit extracts Areas from the first namespace level:

.. list-table::
   :widths: 50 25 25
   :header-rows: 1

   * - Node FQN
     - Area
     - Component
   * - ``/perception/camera/driver``
     - perception
     - driver
   * - ``/nav2/controller``
     - nav2
     - controller
   * - ``/arm/joint_trajectory_controller``
     - arm
     - joint_trajectory_controller
   * - ``/my_node`` (no namespace)
     - root
     - my_node

Designing Your Namespace Structure
----------------------------------

**Option 1: Domain-based (recommended)**

Organize by functional domain:

.. code-block:: text

   /perception/
   ├── cameras/front_camera
   ├── cameras/rear_camera
   ├── lidar/velodyne
   └── fusion/obstacle_detector

   /navigation/
   ├── planner
   ├── controller
   └── costmap

   /manipulation/
   ├── arm/move_group
   └── gripper/controller

**Option 2: Hardware-based**

Organize by physical location:

.. code-block:: text

   /base/
   ├── motors/left_wheel
   ├── motors/right_wheel
   └── imu

   /turret/
   ├── pan_motor
   └── tilt_motor

   /arm/
   ├── joint1
   ├── joint2
   └── gripper

**Option 3: Automotive-style (SOVD-compatible)**

For vehicle-like robots:

.. code-block:: text

   /powertrain/
   ├── engine/controller
   └── transmission/controller

   /chassis/
   ├── brakes/controller
   └── suspension/controller

   /body/
   ├── lights/controller
   └── doors/controller

Implementing in Launch Files
----------------------------

**Using PushRosNamespace:**

.. code-block:: python

   from launch import LaunchDescription
   from launch.actions import GroupAction, PushRosNamespace
   from launch_ros.actions import Node

   def generate_launch_description():
       perception_nodes = GroupAction([
           PushRosNamespace('perception'),
           Node(package='camera_driver', executable='node', name='front_camera',
                namespace='cameras'),
           Node(package='lidar_driver', executable='node', name='velodyne',
                namespace='lidar'),
       ])

       navigation_nodes = GroupAction([
           PushRosNamespace('navigation'),
           Node(package='nav2_planner', executable='planner', name='planner'),
           Node(package='nav2_controller', executable='controller', name='controller'),
       ])

       return LaunchDescription([
           perception_nodes,
           navigation_nodes,
       ])

**Using ComposableNodeContainer:**

.. code-block:: python

   from launch_ros.actions import ComposableNodeContainer
   from launch_ros.descriptions import ComposableNode

   container = ComposableNodeContainer(
       name='perception_container',
       namespace='/perception',
       package='rclcpp_components',
       executable='component_container',
       composable_node_descriptions=[
           ComposableNode(
               package='camera_driver',
               plugin='camera_driver::CameraNode',
               name='front_camera',
               namespace='cameras'
           ),
       ]
   )

Topic-Based Discovery
---------------------

ros2_medkit also discovers components from topics without nodes.
This is useful for:

- Hardware bridges that publish topics directly
- Simulation tools (Isaac Sim, Gazebo with custom plugins)
- External systems publishing to ROS 2

**How it works:**

1. Gateway scans all topics
2. Extracts unique namespace prefixes
3. Creates virtual components for namespaces without nodes

**Example:** Isaac Sim publishes ``/carter1/odom``, ``/carter1/cmd_vel``

Result:

.. code-block:: json

   {
     "id": "carter1",
     "type": "Component",
     "source": "topic",
     "area": "carter1"
   }

Filtered Topics
---------------

Some system topics are filtered from discovery:

- ``/parameter_events``
- ``/rosout``
- ``/clock``

Note: ``/tf`` and ``/tf_static`` are NOT filtered (useful for diagnostics).

Best Practices
--------------

1. **Be consistent** - Use the same namespace pattern across your system

2. **Keep it shallow** - 2-3 levels of nesting is usually enough:

   .. code-block:: text

      /{area}/{subsystem}/{component}

3. **Use meaningful names** - ``/perception/lidar/velodyne`` > ``/ns1/ns2/node3``

4. **Document your hierarchy** - Create a diagram for your team

5. **Consider SOVD compatibility** - If integrating with automotive tools,
   use Area/Component terminology

Example: Complete Robot Hierarchy
---------------------------------

.. code-block:: text

   TurtleBot3 with Manipulation Arm

   /perception/
   ├── cameras/
   │   └── realsense         # Intel RealSense camera
   └── lidar/
       └── lds               # LIDAR Distance Sensor

   /navigation/
   ├── bt_navigator          # Behavior tree navigator
   ├── controller_server     # Path following controller
   ├── planner_server        # Global path planner
   └── costmap/
       ├── global_costmap
       └── local_costmap

   /manipulation/
   ├── arm/
   │   └── move_group        # MoveIt2 planning
   └── gripper/
       └── controller        # Gripper control

   /safety/
   └── emergency_stop        # E-stop handler

   /diagnostics/
   └── aggregator            # Diagnostic aggregator

See Also
--------

- :doc:`integration` - Basic integration guide
- `SOVD Standard <https://www.autosar.org>`_ - AUTOSAR SOVD specification
- `REP-149 <https://ros.org/reps/rep-0149.html>`_ - ROS 2 package naming conventions
