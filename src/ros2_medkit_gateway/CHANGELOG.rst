^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_gateway
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.0 (2026-02-27)
------------------

**Breaking Changes:**

* ``GET /version-info`` response key renamed from ``sovd_info`` to ``items`` for SOVD alignment (`#258 <https://github.com/selfpatch/ros2_medkit/pull/258>`_)
* ``GET /`` root endpoint restructured: ``endpoints`` is now a flat string array, added ``capabilities`` object, ``api_base`` field, and ``name``/``version`` top-level fields (`#258 <https://github.com/selfpatch/ros2_medkit/pull/258>`_)
* Default rosbag storage format changed from ``sqlite3`` to ``mcap`` (`#258 <https://github.com/selfpatch/ros2_medkit/pull/258>`_)

**Features:**

* Layered merge pipeline for hybrid discovery with per-layer, per-field-group merge policies (`#258 <https://github.com/selfpatch/ros2_medkit/pull/258>`_)
* Gap-fill configuration: control heuristic entity creation with ``allow_heuristic_*`` options and namespace filtering (`#258 <https://github.com/selfpatch/ros2_medkit/pull/258>`_)
* Plugin layer: ``IntrospectionProvider`` now wired into discovery pipeline via ``PluginLayer`` (`#258 <https://github.com/selfpatch/ros2_medkit/pull/258>`_)
* ``LogProvider`` plugin interface for custom log backends (`#258 <https://github.com/selfpatch/ros2_medkit/pull/258>`_)
* ``/health`` endpoint includes merge pipeline diagnostics (layers, conflicts, gap-fill stats) (`#258 <https://github.com/selfpatch/ros2_medkit/pull/258>`_)
* Entity detail responses now include ``logs``, ``bulk-data``, ``cyclic-subscriptions`` URIs (`#258 <https://github.com/selfpatch/ros2_medkit/pull/258>`_)
* Area and function log endpoints with namespace aggregation and host-based aggregation (`#258 <https://github.com/selfpatch/ros2_medkit/pull/258>`_)
* Entity capabilities fix: areas and functions now report correct resource collections (`#258 <https://github.com/selfpatch/ros2_medkit/pull/258>`_)
* SOVD compliance documentation with resource collection support matrix (`#258 <https://github.com/selfpatch/ros2_medkit/pull/258>`_)
* Linux introspection plugins: procfs, systemd, and container plugins for process-level diagnostics via ``x-medkit-*`` vendor extension endpoints (`#263 <https://github.com/selfpatch/ros2_medkit/pull/263>`_)
* Gateway plugin framework with dynamic C++ plugin loading (`#237 <https://github.com/selfpatch/ros2_medkit/pull/237>`_)
* Software updates plugin with 8 SOVD-compliant endpoints (`#237 <https://github.com/selfpatch/ros2_medkit/pull/237>`_, `#231 <https://github.com/selfpatch/ros2_medkit/pull/231>`_)
* SSE-based periodic data subscriptions for real-time streaming without polling (`#223 <https://github.com/selfpatch/ros2_medkit/pull/223>`_)
* Global ``DELETE /api/v1/faults`` endpoint (`#228 <https://github.com/selfpatch/ros2_medkit/pull/228>`_)
* Return HEALED/PREPASSED faults via status filter (`#218 <https://github.com/selfpatch/ros2_medkit/pull/218>`_)
* Bulk data upload and delete endpoints (`#216 <https://github.com/selfpatch/ros2_medkit/pull/216>`_)
* Token-bucket rate limiting middleware, configurable per-endpoint (`#220 <https://github.com/selfpatch/ros2_medkit/pull/220>`_)
* Reduce lock contention in ConfigurationManager (`#194 <https://github.com/selfpatch/ros2_medkit/pull/194>`_)
* Cache component topic map to avoid per-request graph rebuild (`#212 <https://github.com/selfpatch/ros2_medkit/pull/212>`_)
* Require cpp-httplib >= 0.14 in pkg-config check (`#230 <https://github.com/selfpatch/ros2_medkit/pull/230>`_)
* Add missing ``ament_index_cpp`` dependency to ``package.xml`` (`#191 <https://github.com/selfpatch/ros2_medkit/pull/191>`_)
* Unit tests for HealthHandlers, DataHandlers, and AuthHandlers (`#232 <https://github.com/selfpatch/ros2_medkit/pull/232>`_, `#234 <https://github.com/selfpatch/ros2_medkit/pull/234>`_, `#233 <https://github.com/selfpatch/ros2_medkit/pull/233>`_)
* Standardize include guards to ``#pragma once`` (`#192 <https://github.com/selfpatch/ros2_medkit/pull/192>`_)
* Use ``foreach`` loop for CMake coverage flags (`#193 <https://github.com/selfpatch/ros2_medkit/pull/193>`_)
* Migrate ``ament_target_dependencies`` to compat shim for Rolling (`#242 <https://github.com/selfpatch/ros2_medkit/pull/242>`_)
* Multi-distro CI support for ROS 2 Humble, Jazzy, and Rolling (`#219 <https://github.com/selfpatch/ros2_medkit/pull/219>`_, `#242 <https://github.com/selfpatch/ros2_medkit/pull/242>`_)
* Contributors: @bburda, @eclipse0922, @mfaferek93

0.2.0 (2026-02-07)
------------------
* Initial rosdistro release
* HTTP REST gateway for ros2_medkit diagnostics system
* SOVD-compatible entity discovery with four entity types:

  * Areas, Components, Apps, Functions
  * HATEOAS links and capabilities in all responses
  * Relationship endpoints (subareas, subcomponents, related-apps, hosts)

* Three discovery modes:

  * Runtime-only: automatic ROS 2 graph introspection
  * Manifest-only: YAML manifest with validation (11 rules)
  * Hybrid: manifest as source of truth + runtime linking

* REST API endpoints:

  * Fault management: GET/POST/DELETE /api/v1/faults
  * Data access: topic sampling via GenericSubscription
  * Operations: service calls and action goals via GenericClient
  * Configuration: parameter get/set via ROS 2 parameter API
  * Snapshots: GET /api/v1/faults/{code}/snapshots
  * Rosbag: GET /api/v1/faults/{code}/snapshots/bag

* Server-Sent Events (SSE) at /api/v1/faults/stream:

  * Multi-client support with thread-safe event queue
  * Keepalive, Last-Event-ID reconnection, configurable max_clients

* JWT-based authentication with configurable policies
* HTTPS/TLS support via OpenSSL and cpp-httplib
* Native C++ ROS 2 serialization via ros2_medkit_serialization (no CLI dependencies)
* Contributors: Bartosz Burda, Michal Faferek
