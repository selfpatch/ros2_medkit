^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_gateway
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Unreleased
----------

**Breaking Changes:**

* Typed router refactor. ``HandlerContext`` no longer carries
  ``send_json`` / ``send_error`` / ``send_plugin_error`` / ``send_dto`` /
  ``parse_body``: handlers return ``http::Result<TResponse>`` and the
  framework owns response writing through ``RouteRegistry``. The raw
  ``void(httplib::Request, httplib::Response)`` ``RouteRegistry`` lambda
  overloads are removed - call sites must use the typed
  ``reg.get<T>`` / ``reg.post<TBody, T>`` / ``reg.del<T>`` overloads, the
  multi-shape ``reg.post_alternates<TBody, TAlt...>`` /
  ``reg.del_alternates<TAlt...>``, or one of the named escape hatches
  (``reg.sse`` / ``reg.binary_download`` / ``reg.multipart_upload<T>`` /
  ``reg.static_asset`` / ``reg.docs_endpoint`` / ``reg.docs_subtree``).
  ``static_assert(dto::has_dto_shape_v<T>)`` gates every typed overload, so
  non-DTO return types fail at compile time. The plugin ABI is unaffected:
  ``PluginResponse`` keeps its ``send_json`` / ``send_error`` surface and
  now routes through the same internal ``http::detail::write_json_body``
  primitive as the framework, so plugin wire format is unchanged
  (`#403 <https://github.com/selfpatch/ros2_medkit/issues/403>`_)
* Provider ABI typed. ``FaultProvider``, ``DataProvider``,
  ``OperationProvider``, and ``UpdateProvider::get_update`` return typed
  DTO envelopes (``FaultListResult`` / ``FaultDetailResult`` /
  ``FaultClearResult`` / the matching ``Data*Result`` and
  ``Operation*Result`` shapes / ``UpdateStatusResult``) instead of raw
  ``tl::expected<nlohmann::json, ErrorInfo>``. The wire bytes are
  byte-identical because each envelope wraps an opaque ``content`` object
  emitted verbatim by ``JsonWriter``; commercial and out-of-tree plugins
  must wrap their existing JSON in the matching envelope type
  (mechanical: ``Result.content = std::move(json_payload)``). The plugin
  ABI itself (``PluginRoute`` shape, ``PluginResponse`` ctor, plugin api
  version) is locked by ``test_plugin_abi_conformance`` and is unchanged
  (`#403 <https://github.com/selfpatch/ros2_medkit/issues/403>`_)
* ``SchemaWriter`` emits optional DTO fields as
  ``anyOf: [<inner>, {type: "null"}]`` (the OpenAPI 3.1 idiom) instead of
  ``nullable: true``. Generated clients see ``T | null`` for every optional
  field rather than ``T | undefined``. Wire format is unchanged - the
  gateway still omits absent optional fields, and ``JsonReader`` continues
  to accept absent fields; the schema change only opts the published spec
  into round-tripping a literal ``null`` value cleanly for clients that
  prefer to send one. The ``rtmaps_medkit`` variant is explicitly NOT
  covered by this PR - its handlers continue to run on the pre-typed
  HandlerContext surface and will be migrated separately
  (`#403 <https://github.com/selfpatch/ros2_medkit/issues/403>`_)
* Synchronous operation-execution service-call failures
  (``POST /api/v1/{entity-path}/operations/{id}/executions`` when the underlying
  ROS 2 service call fails) now return the standard SOVD ``GenericError`` envelope
  (``{"error_code": "vendor-error", "vendor_code":
  "x-medkit-ros2-service-unavailable", "message": "Service call failed", ...}``,
  HTTP status 500 unchanged) instead of the previous bespoke nested
  ``{"error": {"code", "message", "details"}}`` object. This aligns the one
  remaining non-standard error path with every other gateway error; clients that
  parsed ``error.code`` / ``error.details`` for this specific failure must read
  ``vendor_code`` / ``parameters`` instead
  (`#403 <https://github.com/selfpatch/ros2_medkit/issues/403>`_)
* ``ros2_medkit_msgs/srv/ClearFault`` request gains a ``bool skip_correlation_auto_clear`` field (see the per-entity fault scope entry below for the in-tree motivation). Adding a request field changes the service type hash, so out-of-tree callers that invoke the service directly (for example ``ros2 service call /fault_manager/clear_fault ros2_medkit_msgs/srv/ClearFault ...`` as documented in the ``ros2_medkit_fault_manager`` README) must rebuild against the new ``ros2_medkit_msgs`` to keep talking to ``fault_manager``. The in-tree gateway client and server are updated together (`#395 <https://github.com/selfpatch/ros2_medkit/issues/395>`_)
* Per-entity fault routes are now correctly scoped to the entity's hosted apps. ``GET /api/v1/{entity-path}/faults/{fault_code}``, ``DELETE /api/v1/{entity-path}/faults/{fault_code}``, ``GET /api/v1/{entity-path}/faults``, and ``DELETE /api/v1/{entity-path}/faults`` previously fell back to a prefix match against the entity's ``namespace_path``; when that was empty (host-derived / synthetic components, manifest components without a ``namespace`` field, Areas, Functions, and Apps with a wildcard ``ros_binding.namespace_pattern``) the scope filter was silently disabled and the routes exposed - and on ``DELETE``, cleared - faults reported by apps that belonged to entirely different entities. All four handlers now resolve the addressed entity to its hosted-app FQN set (via the new ``HandlerContext::resolve_entity_source_fqns`` helper) and apply a strict all-sources scope check: a fault counts as in scope only when **every** entry in its ``reporting_sources`` is owned by the entity (exact FQN match, or strict path-child via ``<fqn>/...``). Per-fault routes return ``404 Resource Not Found`` for any fault that fails the check; collection routes return an empty ``items`` array. The underlying ``GetFault.srv`` contract is unchanged; ``ClearFault.srv`` gains a new ``skip_correlation_auto_clear`` request flag so per-entity DELETE can opt out of cascade-clearing correlated symptom fault codes that may live in other entities. Per-entity collection responses no longer include the global ``muted_count`` / ``cluster_count`` / ``muted_faults`` / ``clusters`` correlation metadata; those remain on the global ``GET /api/v1/faults`` route. Behavior changes visible to clients: (a) faults reported by apps outside the addressed entity are no longer returned or cleared via that entity's route, (b) **mixed-source** faults that include at least one out-of-entity reporter are likewise rejected with ``404`` on per-fault routes and excluded from per-entity collection responses (use the global ``GET /api/v1/faults`` to see them), (c) per-entity DELETE no longer cascade-clears correlated symptoms outside the entity (`#395 <https://github.com/selfpatch/ros2_medkit/issues/395>`_)
* ``GET /api/v1/updates/{id}/status`` no longer returns ``404`` for a registered-but-idle package; ``POST /api/v1/updates`` now seeds a ``pending`` status, so the endpoint returns ``200 {"status": "pending"}`` immediately after registration. ``404`` is reserved for packages that are not registered. Clients that used ``404`` as a signal for "registered but nothing started yet" must adapt (`#378 <https://github.com/selfpatch/ros2_medkit/issues/378>`_)

**Features:**

* Typed ``fan_out_collection<T>`` aggregating helper replaces raw-JSON ``merge_peer_items`` on the typed collection routes (data, operations, config, logs). Peer items are decoded via ``dto::JsonReader<T>``; items that fail validation are removed from the merged ``items`` array, recorded in ``x-medkit.peer_dropped_items`` with the JsonReader error plus a best-effort ``source_id``, and logged at ``WARN``. Items that parse successfully are re-serialized through the local ``dto::JsonWriter<T>``, so any peer-supplied fields outside the local DTO schema are dropped from the merged response (the previous raw passthrough preserved unknown peer fields verbatim). Previously, malformed peer items silently disappeared into the merged response; fleet operators can now detect inter-gateway schema drift directly on the wire (`#403 <https://github.com/selfpatch/ros2_medkit/issues/403>`_)
* ``Collection<T, XMedkitT>`` is now a 2-parameter template. Domain list endpoints (faults, config, data, logs) reference their richer per-domain collection x-medkit struct (``FaultListXMedkit``, ``ConfigListXMedkit``, ``DataListXMedkit``, ``LogListXMedkit``) directly in the published schema instead of the generic ``XMedkitCollection``, so generated clients see aggregation counts, peer provenance, and ``peer_dropped_items`` from the schema (`#403 <https://github.com/selfpatch/ros2_medkit/issues/403>`_)
* New ``opaque_object("key", &T::field)`` DTO field descriptor in ``dto/contract.hpp``. Binds a ``nlohmann::json`` member as a typed "any JSON object" field: ``JsonWriter`` emits it verbatim, ``JsonReader`` rejects scalars / arrays / null, ``SchemaWriter`` emits ``{type: object, additionalProperties: true, x-medkit-opaque: true}``. Used for fields whose runtime shape is decided by an upstream component the gateway cannot introspect (live ROS message payloads, plugin-defined fault envelopes, action results) (`#403 <https://github.com/selfpatch/ros2_medkit/issues/403>`_)
* ``GET /api/v1/faults/stream`` event payloads now carry an optional ``x-medkit`` SOVD payload-extension object with ``entity_type`` and ``entity_id`` fields. When the gateway can resolve the fault's first reporting source back to a SOVD entity (via the manifest-mode linking index, or a runtime-mode last-segment match against an existing App), consumers can hit ``/{entity_type}/{entity_id}/bulk-data/rosbags/{fault_code}`` directly instead of HEAD-probing every entity. Resolution is snapshotted at event arrival, so a discovery refresh between enqueue and stream-out cannot retroactively change the entity reported to consumers. The ``x-medkit`` object is omitted entirely when no entity can be resolved, so existing SSE consumers ignore the addition (`#380 <https://github.com/selfpatch/ros2_medkit/issues/380>`_)
* Plugin API version bumped to v7. Adds ``PluginContext::notify_entities_changed(EntityChangeScope)`` lifecycle hook for plugins that mutate the entity surface at runtime; default no-op keeps v6 source code compiling unchanged against v7 headers. Binary compatibility is not provided: the plugin loader uses a strict equality check on ``plugin_api_version()``, so out-of-tree plugins must be recompiled (`#376 <https://github.com/selfpatch/ros2_medkit/issues/376>`_)
* New ``discovery.manifest.fragments_dir`` parameter: gateway scans the directory for ``*.yaml`` / ``*.yml`` fragment files on every manifest load / reload and merges apps, components, and functions on top of the base manifest. Fragments are forbidden from declaring top-level ``areas``, ``metadata``, ``discovery``, ``scripts``, ``capabilities``, or ``lock_overrides`` - those stay in the base manifest. Presence of any forbidden key (including empty-valued ones like ``areas: []``) is reported as a ``FRAGMENT_FORBIDDEN_FIELD`` validation error that fails the load / reload. Unknown top-level keys (typos such as ``app:`` vs ``apps:``) are ignored with a warning log. Files merged in alphabetical order for deterministic duplicate-id errors (`#376 <https://github.com/selfpatch/ros2_medkit/issues/376>`_)
* Fragment files are size-capped at 1 MiB (``ManifestParser::kMaxFragmentBytes``) before being read into memory, and any symlink resolving outside the canonical ``fragments_dir`` is skipped with a warning, so misconfigurations or symlink-based escapes cannot hand arbitrary bytes to the YAML parser (`#376 <https://github.com/selfpatch/ros2_medkit/issues/376>`_)
* All-or-nothing fragment semantics: a single malformed or forbidden fragment fails the entire load / reload and keeps the previously-loaded manifest active (`#376 <https://github.com/selfpatch/ros2_medkit/issues/376>`_)
* ``ManifestParser::parse_fragment_file`` convenience entrypoint that injects a synthetic ``manifest_version`` header when the fragment omits one
* See ``design/plugin_entity_notifications.rst`` for the lifecycle, merge-rule, and plugin-side write-contract walkthrough

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
