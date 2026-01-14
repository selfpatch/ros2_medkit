Introduction
============

ros2_medkit
-----------

**Modern, SOVD-compatible diagnostics for ROS 2 robots**, built around an entity tree
(Area / Component / Function / App) for runtime discovery, health modeling, and troubleshooting.

What is ros2_medkit?
--------------------

ros2_medkit provides **modern diagnostics for ROS 2–based systems**.

Instead of hardcoding knowledge about every node, topic, or ECU, ros2_medkit models a robot
as a **diagnostic entity tree**:

.. list-table::
   :widths: 20 40 40
   :header-rows: 1

   * - Entity
     - Description
     - Example
   * - **Area**
     - Physical or logical domain
     - ``base``, ``arm``, ``safety``, ``navigation``
   * - **Component**
     - Hardware or software component within an area
     - ``motor_controller``, ``lidar_driver``
   * - **Function**
     - Capability provided by one or more components
     - ``localization``, ``obstacle_detection``
   * - **App**
     - Deployable software unit
     - node, container, process

The goal is to make this tree **compatible with the SOVD (Service-Oriented Vehicle Diagnostics) model**,
so the same concepts can be used across robots, vehicles, and other embedded systems.

Packages
--------

ros2_medkit consists of several ROS 2 packages:

**ros2_medkit_gateway**
   The main HTTP gateway node. Discovers ROS 2 entities and exposes them via REST API.

**ros2_medkit_serialization**
   Runtime JSON ↔ ROS 2 message serialization library using dynmsg. Enables native
   message handling without compile-time type dependencies.

**ros2_medkit_fault_manager**
   Stores and manages fault lifecycle. Provides ROS 2 services for fault operations.

**ros2_medkit_fault_reporter**
   Client library for reporting faults from your ROS 2 nodes.

**ros2_medkit_diagnostic_bridge**
   Bridge node that converts standard ROS 2 ``/diagnostics`` messages to fault manager faults.

**ros2_medkit_msgs**
   Message and service definitions for fault management.

Next Steps
----------

- :doc:`installation` — Install ros2_medkit
- :doc:`getting_started` — Hands-on tutorial
- :doc:`tutorials/index` — Deep-dive guides
- :doc:`design/index` — Architecture documentation

Community
---------

- 💬 **Discord**: `Join our server <https://discord.gg/fEbWKTah>`_
- 🐛 **Issues**: `Report bugs <https://github.com/selfpatch/ros2_medkit/issues>`_
- 💡 **Discussions**: `GitHub Discussions <https://github.com/selfpatch/ros2_medkit/discussions>`_

