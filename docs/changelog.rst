Changelog
=========

All notable changes to ros2_medkit are documented in this file.

The format is based on `Keep a Changelog <https://keepachangelog.com/en/1.1.0/>`_,
and this project adheres to `Semantic Versioning <https://semver.org/spec/v2.0.0.html>`_.

[0.1.0] - 2026-02-01
--------------------

First public release of ros2_medkit.

Added
~~~~~

**Gateway (ros2_medkit_gateway)**

- REST API gateway exposing ROS 2 graph via SOVD-compatible endpoints
- Discovery endpoints: ``/areas``, ``/components``, ``/apps``, ``/functions``
- Data access: Read topic data, publish to topics
- Operations: Call ROS 2 services and actions with execution tracking
- Configurations: Read/write/reset ROS 2 node parameters
- Faults: Query and clear faults from fault manager
- Three discovery modes: runtime_only, hybrid, manifest_only
- Manifest-based discovery with YAML system definitions
- Heuristic app detection in runtime mode
- JWT authentication with RBAC (viewer, operator, configurator, admin roles)
- TLS/HTTPS support with configurable TLS 1.2/1.3
- CORS configuration for browser clients
- SSE (Server-Sent Events) for real-time fault notifications
- Health check endpoint

**Fault Manager (ros2_medkit_fault_manager)**

- Centralized fault storage and management node
- ROS 2 services: ``report_fault``, ``get_faults``, ``clear_fault``
- AUTOSAR DEM-style debounce lifecycle (PREFAILED → CONFIRMED → HEALED → CLEARED)
- Fault aggregation from multiple sources
- Severity escalation
- In-memory storage with thread-safe implementation

**Fault Reporter (ros2_medkit_fault_reporter)**

- Client library for reporting faults from ROS 2 nodes
- Local filtering with configurable threshold and time window
- Fire-and-forget async service calls
- High-severity bypass for immediate fault reporting

**Diagnostic Bridge (ros2_medkit_diagnostic_bridge)**

- Bridge node converting ``/diagnostics`` messages to fault manager faults
- Configurable severity mapping from diagnostic status levels
- Support for diagnostic arrays with multiple status entries

**Serialization (ros2_medkit_serialization)**

- Runtime JSON ↔ ROS 2 message serialization
- Dynamic message introspection without compile-time type knowledge
- Support for all ROS 2 built-in types, arrays, nested messages
- Type caching for performance

**Messages (ros2_medkit_msgs)**

- ``Fault.msg``: Fault status message with severity, timestamps, sources
- ``FaultEvent.msg``: Fault event for subscriptions
- ``ReportFault.srv``: Service for reporting faults
- ``GetFaults.srv``: Service for querying faults with filters
- ``ClearFault.srv``: Service for clearing faults

**Documentation**

- Sphinx documentation with Doxygen integration
- Getting Started tutorial
- REST API reference
- Configuration reference (server, discovery, manifest)
- Authentication and HTTPS tutorials
- Docker deployment guide
- Companion project tutorials (web-ui, mcp-server)

**Tooling**

- Postman collection for API testing
- VS Code tasks for build/test/launch
- Development container configuration
- GitHub Actions CI/CD pipeline

Companion Projects
~~~~~~~~~~~~~~~~~~

- `sovd_web_ui <https://github.com/selfpatch/sovd_web_ui>`_: Web interface for entity browsing
- `ros2_medkit_mcp <https://github.com/selfpatch/ros2_medkit_mcp>`_: MCP server for LLM integration

SOVD Compliance
~~~~~~~~~~~~~~~

This release implements a subset of the SOVD (Service-Oriented Vehicle Diagnostics)
specification adapted for ROS 2:

- Core discovery endpoints (areas, components)
- Extended discovery (apps, functions) via manifest mode
- Data access (read, write)
- Operations (services, actions with executions)
- Configurations (parameters)
- Faults (query, clear)

Not yet implemented:

- Bulk data transfer
- Software updates
- Locks
- Triggers
- Communication logs
