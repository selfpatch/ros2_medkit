Changelog
=========

All notable changes to ros2_medkit are documented in this file.

The format is based on `Keep a Changelog <https://keepachangelog.com/en/1.1.0/>`_,
and this project adheres to `Semantic Versioning <https://semver.org/spec/v2.0.0.html>`_.

[Unreleased]
------------

Added
~~~~~

* Software update management with pluggable backend architecture:

  - 8 SOVD-compliant ``/updates`` endpoints (CRUD + prepare/execute/automated/status)
  - Plugin system via dlopen for runtime backend loading
  - Async lifecycle with progress tracking and status polling
  - Feature gating via ``updates.enabled`` parameter

* SOVD bulk-data endpoints for all entity types:

  - ``GET /{entity}/bulk-data`` - list available bulk-data categories
  - ``GET /{entity}/bulk-data/{category}`` - list bulk-data descriptors
  - ``GET /{entity}/bulk-data/{category}/{id}`` - download bulk-data file

* Inline ``environment_data`` in fault response with:

  - ``extended_data_records``: First/last occurrence timestamps
  - ``snapshots[]``: Array of freeze_frame and rosbag entries

* SOVD-compliant ``status`` object in fault response with aggregatedStatus,
  testFailed, confirmedDTC, pendingDTC fields
* UUID identifiers for rosbag bulk-data items
* ``x-medkit`` extensions with occurrence_count, severity_label

Changed
~~~~~~~

* Fault response structure now SOVD-compliant with ``item`` wrapper
* Rosbag downloads use SOVD bulk-data pattern instead of legacy endpoints
* Rosbag IDs changed from timestamps to UUIDs

Removed
~~~~~~~

* ``GET /faults/{code}/snapshots`` - use ``environment_data`` in fault response
* ``GET /faults/{code}/snapshots/bag`` - use bulk-data endpoint
* ``GET /{entity}/faults/{code}/snapshots`` - use ``environment_data``
* ``GET /{entity}/faults/{code}/snapshots/bag`` - use bulk-data endpoint

**Breaking Changes:**

* Fault response structure changed - clients must update to handle ``item`` wrapper
  and ``environment_data`` structure
* Legacy snapshot endpoints removed - migrate to inline snapshots and bulk-data
* Rosbag identifiers changed from timestamps to UUIDs

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
- ROS 2 services: ``report_fault``, ``list_faults``, ``clear_fault``
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
- ``ListFaults.srv``: Service for querying faults with filters
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
- Faults (query, clear) with environment_data
- Bulk data transfer (rosbags via bulk-data endpoints)

Not yet implemented:

- Locks
- Triggers
- Communication logs
