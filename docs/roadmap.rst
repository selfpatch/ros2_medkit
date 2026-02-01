Roadmap
=======

This document outlines the proposed development roadmap for ros2_medkit.
The plan is subject to change based on community feedback and project priorities.

For the latest status, see the `GitHub Milestones <https://github.com/selfpatch/ros2_medkit/milestones>`_.

Strategy
--------

**Developer-first, then system-wide.**

The project follows a phased approach that prioritizes developer experience:

1. **Start with Apps (ROS 2 nodes)** — The level closest to developers. Provide tooling
   that makes their daily work easier: discovering nodes, reading data, checking faults.

2. **Expand to full system** — Once app-level features are solid, extend coverage to
   Components, Areas, and system-wide operations.

3. **API coverage first, quality later** — The initial goal is to cover the entire
   standard API surface using available ROS 2 capabilities. This enables early integration
   with real projects and compliance validation. Code hardening and production-ready
   implementations will follow in later phases.

Milestones
----------

MS1: Full Discovery & Data Access
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Goal:** Provide a complete view of the ROS 2 system and enable basic data operations.

Developers can discover all running nodes, explore the entity hierarchy, and read/write
diagnostic data. This milestone establishes the foundation for all subsequent features.

**Standard API coverage:**

- Discovery (11 endpoints) — version info, entity collections, hierarchy navigation
- Data (5 endpoints) — data categories, groups, read/write operations

`MS1 on GitHub <https://github.com/selfpatch/ros2_medkit/milestone/1>`_

MS2: App Observability
~~~~~~~~~~~~~~~~~~~~~~

**Goal:** Enable developers to monitor application health and diagnose issues.

With observability features, developers can check fault codes, browse logs, and monitor
the lifecycle state of their nodes — essential for debugging and troubleshooting.

**Standard API coverage:**

- Faults (4 endpoints) — list, inspect, and clear diagnostic faults
- Logs (5 endpoints) — browse and configure application logs
- Lifecycle (6 endpoints) — status, start, restart, shutdown operations

`MS2 on GitHub <https://github.com/selfpatch/ros2_medkit/milestone/2>`_

MS3: Configuration & Control
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Goal:** Allow developers to configure and control running applications.

This milestone brings configuration management (backed by ROS 2 parameters) and
operation execution, giving developers runtime control over their nodes.

**Standard API coverage:**

- Configuration (5 endpoints) — read/write configuration values
- Operations (7 endpoints) — define, execute, and monitor operations
- Modes (3 endpoints) — entity operating modes

`MS3 on GitHub <https://github.com/selfpatch/ros2_medkit/milestone/3>`_

MS4: Automation & Subscriptions
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Goal:** Enable automated workflows and reactive data collection.

Developers can set up scripts for automated diagnostics, subscribe to data changes,
and define triggers for event-driven workflows.

**Standard API coverage:**

- Scripts (8 endpoints) — upload, execute, and manage diagnostic scripts
- Subscriptions (8 endpoints) — cyclic data subscriptions and triggers
- Datasets (4 endpoints) — dynamic data lists for subscription

`MS4 on GitHub <https://github.com/selfpatch/ros2_medkit/milestone/4>`_

MS5: Fleet & Advanced Features
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Goal:** Complete SOVD API coverage with fleet-oriented and advanced features.

The final milestone adds capabilities for fleet management, bulk data transfer,
software updates, and authentication — completing the standard specification coverage.

**Standard API coverage:**

- Bulk Data (5 endpoints) — large data transfers
- Communication Logs (5 endpoints) — protocol-level logging
- Clear Data (5 endpoints) — cache and learned data management
- Updates (4 endpoints) — software update management
- Auth (2 endpoints) — authorization and token management

`MS5 on GitHub <https://github.com/selfpatch/ros2_medkit/milestone/5>`_

MS6: Fault Management System
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Goal:** Add fault management to ros2_medkit with fault reporting, two-level filtering
(local + central), persistent storage, lifecycle management, and REST API access.

This milestone introduces a complete fault management architecture that allows ROS 2
nodes to report faults through a simple client library, with intelligent filtering
and central aggregation.

**Key features:**

- **Two-level filtering**: FaultReporter (local) + FaultManager (central)
- **Multi-source aggregation**: Same fault code from multiple sources combined into single entry
- **Persistent storage**: Fault state survives restarts
- **REST API + SSE**: Real-time fault monitoring via HTTP
- **Backwards compatibility**: Integration with ``diagnostic_updater``

**Success criteria:**

- FaultReporter library with local filtering (default enabled)
- FaultManager with central aggregation and lifecycle management
- Storage survives node restarts
- REST API endpoints for fault CRUD operations
- Server-Sent Events (SSE) for real-time fault updates

See :doc:`design/ros2_medkit_fault_manager/index` and :doc:`design/ros2_medkit_fault_reporter/index`
for detailed architecture documentation.

`MS6 on GitHub <https://github.com/selfpatch/ros2_medkit/milestone/6>`_

Future Directions
-----------------

After achieving full standard API coverage, the project will focus on:

**Code Hardening**
   Replace initial implementations with production-ready code. Remove workarounds
   and refactor into smaller, well-tested packages.

**Additional Diagnostic Protocols**
   Extend support to other diagnostic standards relevant to robotics and embedded
   systems.

**Transport Protocols**
   Add support for alternative transport mechanisms beyond HTTP/REST.

We welcome community input on prioritization. Feel free to open an issue or
join the discussion on `Discord <https://discord.gg/6CXPMApAyq>`_!
