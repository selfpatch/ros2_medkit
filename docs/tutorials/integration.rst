Integrating with Your ROS 2 System
===================================

This tutorial shows how to integrate ros2_medkit with your existing ROS 2 application.

.. contents:: Table of Contents
   :local:
   :depth: 2

Overview
--------

ros2_medkit works with any ROS 2 system out of the box through automatic discovery.
This tutorial covers:

- Basic integration (zero configuration)
- Organizing nodes with namespaces
- Reporting custom faults
- Best practices for production

Zero-Configuration Integration
------------------------------

The simplest integration requires no changes to your existing code.
Just run the gateway alongside your nodes:

.. code-block:: bash

   # Terminal 1: Your existing ROS 2 application
   ros2 launch my_robot robot.launch.py

   # Terminal 2: ros2_medkit gateway
   ros2 launch ros2_medkit_gateway gateway.launch.py

The gateway will automatically discover:

- All running nodes → Components
- Node namespaces → Areas
- Topics, services, actions → Data, Operations
- Parameters → Configurations

Organizing with Namespaces
--------------------------

ros2_medkit maps ROS 2 namespaces to Areas. To create a logical hierarchy,
use namespaces in your launch files:

.. code-block:: python

   # my_robot.launch.py
   from launch import LaunchDescription
   from launch_ros.actions import Node

   def generate_launch_description():
       return LaunchDescription([
           # Perception nodes → /perception area
           Node(
               package='camera_driver',
               executable='camera_node',
               namespace='/perception/cameras',
               name='front_camera'
           ),
           Node(
               package='lidar_driver',
               executable='lidar_node',
               namespace='/perception/lidar',
               name='velodyne'
           ),

           # Navigation nodes → /navigation area
           Node(
               package='nav2_controller',
               executable='controller_server',
               namespace='/navigation',
               name='controller'
           ),

           # Manipulation nodes → /manipulation area
           Node(
               package='moveit2',
               executable='move_group',
               namespace='/manipulation',
               name='move_group'
           ),
       ])

This creates the hierarchy:

.. code-block:: text

   Areas:
   ├── perception
   │   ├── Component: front_camera
   │   └── Component: velodyne
   ├── navigation
   │   └── Component: controller
   └── manipulation
       └── Component: move_group

Reporting Custom Faults
-----------------------

To report faults from your nodes, use the ``ros2_medkit_fault_reporter`` package:

**Add dependency to package.xml:**

.. code-block:: xml

   <depend>ros2_medkit_fault_reporter</depend>

**Report faults in your node:**

.. code-block:: cpp

   #include "ros2_medkit_fault_reporter/fault_reporter.hpp"
   #include "ros2_medkit_msgs/msg/fault.hpp"

   using ros2_medkit_fault_reporter::FaultReporter;
   using ros2_medkit_msgs::msg::Fault;

   class MyNode : public rclcpp::Node
   {
   public:
     MyNode() : Node("my_node")
     {
       // FaultReporter requires shared_from_this() and a source identifier
       fault_reporter_ = std::make_shared<FaultReporter>(
         this->shared_from_this(),
         this->get_fully_qualified_name());
     }

     void check_sensor()
     {
       if (!sensor_ok_) {
         fault_reporter_->report(
           "SENSOR_DISCONNECTED",
           Fault::SEVERITY_ERROR,
           "Front camera not responding"
         );
       }
     }

   private:
     std::shared_ptr<FaultReporter> fault_reporter_;
   };

.. note::

   Faults are cleared through the FaultManager service, not the reporter.
   Use ``ros2 service call /fault_manager/clear_fault`` or the REST API
   ``DELETE /api/v1/components/{id}/faults/{fault_code}``.

**Fault severity levels:**

- ``Fault::SEVERITY_INFO`` - Informational, no action required
- ``Fault::SEVERITY_WARN`` - Warning, may need attention
- ``Fault::SEVERITY_ERROR`` - Error, requires attention
- ``Fault::SEVERITY_CRITICAL`` - Critical, immediate action required

Exposing Custom Data
--------------------

Any topic under your node's namespace is automatically exposed via the Data API:

.. code-block:: cpp

   // These topics will be accessible via:
   // GET /api/v1/components/my_node/data

   pub_status_ = create_publisher<MyStatus>("status", 10);
   pub_diagnostics_ = create_publisher<DiagnosticArray>("diagnostics", 10);

**Best practices for data exposure:**

1. Use meaningful topic names
2. Publish regularly for real-time monitoring
3. Include timestamps in messages
4. Use standard message types when possible

Exposing Services and Actions
-----------------------------

Services and actions under your namespace become Operations:

.. code-block:: cpp

   // Service: POST /api/v1/components/my_node/operations/reset
   srv_reset_ = create_service<std_srvs::srv::Trigger>(
     "reset",
     std::bind(&MyNode::handle_reset, this, _1, _2));

   // Action: POST /api/v1/components/my_node/operations/calibrate
   action_calibrate_ = rclcpp_action::create_server<Calibrate>(
     this, "calibrate",
     std::bind(&MyNode::handle_goal, this, _1, _2),
     std::bind(&MyNode::handle_cancel, this, _1),
     std::bind(&MyNode::handle_accepted, this, _1));

Exposing Parameters
-------------------

ROS 2 parameters become Configurations automatically:

.. code-block:: cpp

   // These parameters will be accessible via:
   // GET/PUT /api/v1/components/my_node/configurations/update_rate

   declare_parameter("update_rate", 10.0);
   declare_parameter("enabled", true);
   declare_parameter("mode", "auto");

**Make parameters dynamically reconfigurable:**

.. code-block:: cpp

   auto param_callback = [this](const std::vector<rclcpp::Parameter>& params) {
     for (const auto& param : params) {
       if (param.get_name() == "update_rate") {
         update_rate_ = param.as_double();
       }
     }
     return rcl_interfaces::msg::SetParametersResult{.successful = true};
   };
   param_callback_handle_ = add_on_set_parameters_callback(param_callback);

Launch File Integration
-----------------------

Example launch file combining your robot with ros2_medkit:

.. code-block:: python

   # robot_with_diagnostics.launch.py
   from launch import LaunchDescription
   from launch.actions import IncludeLaunchDescription
   from launch.launch_description_sources import PythonLaunchDescriptionSource
   from launch_ros.actions import Node
   from ament_index_python.packages import get_package_share_directory
   import os

   def generate_launch_description():
       medkit_share = get_package_share_directory('ros2_medkit_gateway')

       return LaunchDescription([
           # Your robot launch
           IncludeLaunchDescription(
               PythonLaunchDescriptionSource([
                   get_package_share_directory('my_robot'),
                   '/launch/robot.launch.py'
               ])
           ),

           # Fault manager for fault tracking
           Node(
               package='ros2_medkit_fault_manager',
               executable='fault_manager_node',
               name='fault_manager'
           ),

           # Gateway with custom config
           IncludeLaunchDescription(
               PythonLaunchDescriptionSource([
                   medkit_share, '/launch/gateway.launch.py'
               ]),
               launch_arguments={
                   'server_host': '0.0.0.0',
                   'server_port': '8080'
               }.items()
           ),
       ])

Production Checklist
--------------------

Before deploying to production:

.. list-table::
   :widths: 10 90
   :header-rows: 0

   * - ☐
     - Enable TLS (:doc:`https`)
   * - ☐
     - Configure authentication (:doc:`authentication`)
   * - ☐
     - Set appropriate CORS origins
   * - ☐
     - Use namespaces for logical organization
   * - ☐
     - Implement fault reporting for critical components
   * - ☐
     - Test with expected load
   * - ☐
     - Set up monitoring and alerting
   * - ☐
     - Document your Area/Component hierarchy

See Also
--------

- :doc:`custom_areas` - Advanced area customization
- :doc:`docker` - Docker deployment
- :doc:`authentication` - Security configuration
