^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_gateway
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Unreleased
----------

**Features:**

* Plugin API version bumped to v7. Adds ``PluginContext::notify_entities_changed(EntityChangeScope)`` lifecycle hook for plugins that mutate the entity surface at runtime; default no-op preserves v6 source and binary compatibility (`#376 <https://github.com/selfpatch/ros2_medkit/issues/376>`_)
* New ``discovery.manifest.fragments_dir`` parameter: gateway scans the directory for ``*.yaml`` / ``*.yml`` fragment files on every manifest load / reload and merges apps, components, and functions on top of the base manifest. Fragments are forbidden from declaring top-level ``areas``, ``metadata`` (any field), ``discovery``, ``scripts``, ``capabilities``, or ``lock_overrides`` - those stay in the base manifest and each attempt is reported as a ``FRAGMENT_FORBIDDEN_FIELD`` validation error that fails the load / reload. Files merged in alphabetical order for deterministic duplicate-id errors (`#376 <https://github.com/selfpatch/ros2_medkit/issues/376>`_)
* ``ManifestParser::parse_fragment_file`` convenience entrypoint that injects a synthetic ``manifest_version`` header when the fragment omits one
* See ``design/plugin_entity_notifications.rst`` for the lifecycle and merge-rule walkthrough

0.4.0 (2026-03-20)
------------------

**Breaking Changes:**

* ``GET /version-info`` response key renamed from ``sovd_info`` to ``items`` for SOVD alignment (`#258 <https://github.com/selfpatch/ros2_medkit/pull/258>`_)
* ``GET /`` root endpoint restructured: ``endpoints`` is now a flat string array, added ``capabilities`` object, ``api_base`` field, and ``name``/``version`` top-level fields (`#258 <https://github.com/selfpatch/ros2_medkit/pull/258>`_)
* Default rosbag storage format changed from ``sqlite3`` to ``mcap`` (`#258 <https://github.com/selfpatch/ros2_medkit/pull/258>`_)
* Plugin API version bumped to v4 - added ``ScriptProvider``, locking API, and extended ``PluginContext`` with entity snapshot, fault listing, and sampler registration
* ``GraphProviderPlugin`` extracted to separate ``ros2_medkit_graph_provider`` package

**Features:**

*Discovery & Merge Pipeline:*

* Layered merge pipeline for hybrid discovery with per-layer, per-field-group merge policies (`#258 <https://github.com/selfpatch/ros2_medkit/pull/258>`_)
* Gap-fill configuration: control heuristic entity creation with ``allow_heuristic_*`` options and namespace filtering (`#258 <https://github.com/selfpatch/ros2_medkit/pull/258>`_)
* Plugin layer: ``IntrospectionProvider`` now wired into discovery pipeline via ``PluginLayer`` (`#258 <https://github.com/selfpatch/ros2_medkit/pull/258>`_)
* ``/health`` endpoint includes merge pipeline diagnostics (layers, conflicts, gap-fill stats) (`#258 <https://github.com/selfpatch/ros2_medkit/pull/258>`_)
* Entity detail responses now include ``logs``, ``bulk-data``, ``cyclic-subscriptions`` URIs (`#258 <https://github.com/selfpatch/ros2_medkit/pull/258>`_)
* Entity capabilities fix: areas and functions now report correct resource collections (`#258 <https://github.com/selfpatch/ros2_medkit/pull/258>`_)
* ``discovery.manifest.enabled`` / ``discovery.runtime.enabled`` parameters for hybrid mode
* ``NewEntities.functions`` - plugins can now produce Function entities
* ``GET /apps/{id}/is-located-on`` endpoint for reverse host lookup (app to component)
* Beacon discovery plugin system - push-based entity enrichment via ROS 2 topic
* ``x-medkit-topic-beacon`` and ``x-medkit-param-beacon`` vendor extension REST endpoints
* Linux introspection plugins: procfs, systemd, and container plugins via ``x-medkit-*`` vendor endpoints (`#263 <https://github.com/selfpatch/ros2_medkit/pull/263>`_)

*Locking:*

* SOVD-compliant resource locking: acquire, release, extend with session tracking and expiration
* Lock enforcement on all mutating handlers (PUT, POST, DELETE)
* Per-entity lock configuration via manifest YAML with ``required_scopes``
* Lock API exposed to plugins via ``PluginContext``
* Automatic cyclic subscription cleanup on lock expiry
* ``LOCKS`` capability in entity descriptions

*Scripts:*

* SOVD script execution endpoints: CRUD for scripts and executions with subprocess execution
* ``ScriptProvider`` plugin interface for custom script backends
* ``DefaultScriptProvider`` with manifest + filesystem CRUD, argument passing, and timeout
* Manifest-defined scripts: ``ManifestParser`` populates ``ScriptsConfig.entries`` from manifest YAML
* ``allow_uploads`` config toggle for hardened deployments
* RBAC integration for script operations

*Logging:*

* ``LogProvider`` plugin interface for custom log backends (`#258 <https://github.com/selfpatch/ros2_medkit/pull/258>`_)
* ``LogManager`` with ``/rosout`` ring buffer and plugin delegation
* ``/logs`` and ``/logs/configuration`` endpoints
* ``LOGS`` capability in discovery responses
* Configurable log buffer size via parameters
* Area and function log endpoints with namespace aggregation (`#258 <https://github.com/selfpatch/ros2_medkit/pull/258>`_)

*Triggers:*

* Condition-based triggers with CRUD endpoints, SSE event streaming, and hierarchy matching
* ``TriggerManager`` with ``ConditionEvaluator`` interface and 4 built-in evaluators (OnChange, OnChangeTo, EnterRange, LeaveRange)
* ``ResourceChangeNotifier`` for async dispatch from FaultManager, UpdateManager, and OperationManager
* ``TriggerTopicSubscriber`` for data trigger ROS 2 topic subscriptions
* Persistent trigger storage via SQLite with restore-on-restart support
* ``TriggerTransportProvider`` plugin interface for custom trigger delivery

*OpenAPI & Documentation:*

* ``RouteRegistry`` as single source of truth for routes and OpenAPI metadata
* ``OpenApiSpecBuilder`` for full OpenAPI 3.1.0 document assembly with ``SchemaBuilder`` and ``PathBuilder``
* Compile-time Swagger UI embedding (``ENABLE_SWAGGER_UI``)
* Named component schemas with ``$ref``, clean ``operationId`` values, endpoint descriptions, ``GenericError`` schema refs, ``info.contact``, Spectral-clean output, multipart upload schemas, static spec caching
* SOVD compliance documentation with resource collection support matrix (`#258 <https://github.com/selfpatch/ros2_medkit/pull/258>`_)

*Other:*

* Multi-collection cyclic subscription support (data, faults, logs, configurations, update-status)
* Generation-based caching for capability responses via ``CapabilityGenerator``
* ``PluginContext::get_child_apps()`` for Component-level aggregation
* Sub-resource RBAC patterns for all collections
* Auto-populate gateway version from ``package.xml`` via CMake
* Namespaced fault manager integration - ``FaultManagerPaths`` resolves service/topic names for custom namespaces
* Grouped ``fault_manager.*`` parameter namespace for cleaner configuration

**Build:**

* Extracted shared cmake modules into ``ros2_medkit_cmake`` package (`#294 <https://github.com/selfpatch/ros2_medkit/pull/294>`_)
* Auto-detect ccache for faster incremental rebuilds
* Precompiled headers for gateway package
* Centralized clang-tidy configuration (opt-in locally, mandatory in CI)

**Tests:**

* Unit tests for DiscoveryHandlers, OperationHandlers, ScriptHandlers, LockHandlers, LockManager, ScriptManager, DefaultScriptProvider
* Comprehensive integration tests for locking, scripts, graph provider plugin, beacon plugins, OpenAPI/docs, logging, namespaced fault manager
* Contributors: @bburda

0.3.0 (2026-02-27)
------------------

**Features:**

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
