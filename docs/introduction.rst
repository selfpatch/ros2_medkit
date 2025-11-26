Introduction
============

ros2_medkit
-----------

Modern, SOVD-compatible diagnostics for ROS 2 robots, built around an entity tree
(Area / Component / Function / App) for runtime discovery, health modeling, and troubleshooting.

What is ros2_medkit?
--------------------

ros2_medkit is an experiment in **modern diagnostics for ROS 2–based systems**.

Instead of hardcoding knowledge about every node, topic, or ECU, ros2_medkit models a robot
as a **diagnostic entity tree**:

- **Area** – physical or logical domain (e.g. ``base``, ``arm``, ``safety``, ``navigation``)
- **Component** – hardware or software component within an area
- **Function** – capability provided by one or more components
- **App** – deployable software unit (node, container, process)

The goal is to make this tree **compatible with the SOVD (Service-Oriented Vehicle Diagnostics) model**,
so the same concepts can be used across robots, vehicles, and other embedded systems.

Status
------

**Early prototype / work in progress**

This is an open source project exploring diagnostic patterns for ROS 2.
APIs, architecture, and naming may change as the project evolves.

The current focus is on covering the SOVD API surface to enable early integration
and compliance validation. See the :doc:`roadmap` for planned milestones and priorities.

Target Use Cases
----------------

- Runtime discovery of what is actually running on the robot
- Health state modeled per Area / Component / Function / App
- Better remote troubleshooting and fleet-level observability for ROS 2 robots
