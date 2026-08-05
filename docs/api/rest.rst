REST API Reference
==================

The ros2_medkit gateway exposes a RESTful API for interacting with ROS 2 systems.
All endpoints are prefixed with ``/api/v1``.

.. note::

   Entity endpoints (``/components``, ``/apps``) share the same handler implementations.
   The examples use ``/components`` but the same patterns apply to ``/apps``.

.. contents:: Table of Contents
   :local:
   :depth: 2

Client Request Headers
----------------------

Two headers a client may send are read across many endpoints rather than
belonging to one. Both are optional, and both are declared per-operation in the
generated OpenAPI document, so a generated client sees them on exactly the
operations that read them.

``X-Client-Id``
   Identifies the calling client for :doc:`resource locking <locking>`. Read by
   every lock-participating write - those operations also carry
   ``x-medkit-lock-guarded: true`` and declare a ``409``, and
   :ref:`locking-blocked-operations` lists them. While a lock protects an
   entity's collection, only the client holding it may write; every other
   caller, including one that sends no ``X-Client-Id``, is answered ``409``.
   The lock endpoints themselves also read it: ``POST``/``PUT``/``DELETE``
   ``/locks`` require it, and the two ``GET`` routes use it only to fill in the
   ``owned`` field.

   ``DELETE /api/v1/faults`` is the one route that reads it without ever
   answering ``409``; it silently skips faults on entities locked by another
   client and still answers ``204``. Nothing on the response reports the skip -
   ``X-Medkit-Local-Only`` is about aggregated peers, not locks - so re-read the
   entity's faults to see what survived.

``X-Medkit-No-Fan-Out``
   Answer from this gateway alone: do not query aggregated peers and do not
   merge their items. Read by the **per-entity** resource-collection list
   endpoints (data, operations, configurations, faults, logs) and by ``GET
   /api/v1/version-info`` - exactly the operations that declare it in the
   OpenAPI document.

   The gateway sets it on its own outbound peer requests, which stops
   bidirectional aggregation from recursing **on the routes that check it**.
   The global ``GET /api/v1/faults`` does not check it, so it neither declares
   the header nor honours it (see :ref:`the fan-out design note
   <aggregation-fan-out>`).

   **Presence-only.** The value is never read, so ``X-Medkit-No-Fan-Out:
   false`` suppresses fan-out exactly like any other value. The OpenAPI schema
   is a string rather than a boolean for that reason. Omit the header to get
   the aggregated answer.

Server Capabilities
-------------------

``GET /api/v1/``
   Get server capabilities and entry points.

   **Example Response:**

   .. code-block:: json

      {
        "name": "ROS 2 Medkit Gateway",
        "version": "0.6.0",
        "api_base": "/api/v1",
        "endpoints": [
          "GET /api/v1/health",
          "GET /api/v1/areas",
          "GET /api/v1/components",
          "GET /api/v1/apps",
          "GET /api/v1/functions",
          "GET /api/v1/faults",
          "..."
        ],
        "capabilities": {
          "discovery": true,
          "data_access": true,
          "operations": true,
          "async_actions": true,
          "configurations": true,
          "faults": true,
          "logs": true,
          "bulk_data": true,
          "cyclic_subscriptions": true,
          "triggers": true,
          "updates": false,
          "authentication": false,
          "tls": false,
          "aggregation": false
        }
      }

   The ``capabilities.aggregation`` flag is ``true`` when the aggregation
   subsystem is enabled on this gateway (i.e. ``aggregation.enabled=true``
   in config, which wires up an ``AggregationManager``). It does NOT
   require peers to be present - a gateway with aggregation enabled but
   zero peers still reports ``true`` and still emits the
   aggregation-only response fields (``peers``, which may be an empty
   array, and ``warnings`` on ``/health``; ``x-medkit.contributors`` on
   entities, which will contain only ``"local"`` until a peer
   contributes). Clients can feature-detect those fields using this
   flag instead of probing for field presence.

``GET /api/v1/version-info``
   Get gateway version and status information.

   **Example Response:**

   .. code-block:: json

      {
        "items": [
          {
            "version": "1.0.0",
            "base_uri": "/api/v1",
            "vendor_info": {
              "version": "0.6.0",
              "name": "ros2_medkit"
            }
          }
        ]
      }

``GET /api/v1/health``
   Health check endpoint. Returns HTTP 200 if gateway is operational.

   When aggregation is enabled (``capabilities.aggregation == true`` in
   the root response), the body includes additional x-medkit extension
   fields:

   - ``peers`` - array of peer status objects (URL, name, reachability,
     last-seen timestamp) for every configured or discovered peer.
   - ``warnings`` - array of structured operator-actionable aggregation
     warnings (always present when aggregation is active; empty when
     there are no active anomalies). Each warning carries ``code``,
     ``message``, ``entity_ids``, and ``peer_names``. See
     :doc:`warning_codes` for the stable list of codes.
   - ``warning_schema_version`` - integer contract version for the
     ``warnings`` array. Clients key on this instead of string-matching
     codes. See :doc:`warning_codes` ``Schema versioning``.

   The body also always includes two subscription-pool vendor-extension
   sections, populated from atomic reads so ``/health`` never blocks even
   when the sampling pool is under load:

   - ``x-medkit-subscription-executor`` - state of the single-writer
     worker that owns the pool's subscription node. Fields:
     ``worker_alive``, ``degraded``, ``queue_depth``,
     ``queue_max_depth_observed``, ``queue_dropped``, ``tasks_completed``,
     ``tasks_failed``, ``last_task_latency_us``, ``max_task_latency_us``,
     ``current_task_age_ms``, ``watchdog_trips``, ``graph_events_received``.
     External monitors (k8s liveness, Docker HEALTHCHECK, systemd watchdog)
     should page on ``degraded == true``.
   - ``x-medkit-data-provider`` - pool-level counters: ``pool_size``,
     ``pool_cap``, ``pool_hits``, ``pool_misses``, ``evictions_total``,
     ``type_change_events``, ``graph_events_received``,
     ``concurrent_cold_waits``.

   See :doc:`/design/ros2_medkit_gateway/ros2_subscription_architecture`
   for the underlying pool design that produces these counters.

   .. note::

      Security: ``/health`` is currently reachable without
      authentication by default (``auth.enabled`` defaults to
      ``false``), and even with auth enabled the endpoint is readable
      by the ``viewer``, ``operator``, and ``configurator`` roles. The
      ``peers`` array enumerates every configured peer's name and URL,
      which reveals deployment topology. This is by design for
      operator observability in trusted LANs, but on shared-infra or
      multi-tenant installs you should front the endpoint with an
      authenticating reverse proxy or restrict the peer-name field to
      admin-gated callers at the ingress.

   **Example Response (aggregation enabled, one leaf collision):**

   .. code-block:: json

      {
        "status": "healthy",
        "timestamp": 1776185189048036615,
        "discovery": {
          "mode": "hybrid",
          "strategy": "hybrid_discovery"
        },
        "peers": [
          {"name": "peer_b", "url": "http://peer-b:8080", "healthy": true},
          {"name": "peer_c", "url": "http://peer-c:8080", "healthy": true}
        ],
        "warning_schema_version": 1,
        "warnings": [
          {
            "code": "leaf_id_collision",
            "message": "Component 'ecu-x' is announced by multiple peers (peer_b, peer_c); routing falls back to last-writer-wins which is non-deterministic. Resolve by renaming the Component on one side or by modelling it as a hierarchical parent (declare a child Component with parentComponentId='ecu-x' on the owning peer).",
            "entity_ids": ["ecu-x"],
            "peer_names": ["peer_b", "peer_c"]
          }
        ]
      }

Discovery Endpoints
-------------------

Areas
~~~~~

``GET /api/v1/areas``
   List all areas (logical/physical groupings).

   **Example Response:**

   .. code-block:: json

      {
        "items": [
          {
            "id": "powertrain",
            "name": "Powertrain",
            "href": "/api/v1/areas/powertrain"
          }
        ]
      }

``GET /api/v1/areas/{area_id}``
   Get area capabilities and metadata.

``GET /api/v1/areas/{area_id}/contains``
   List components contained in this area.

``GET /api/v1/areas/{area_id}/components``
   List components in a specific area.

   .. note::

      **ros2_medkit extension:** Areas support resource collections beyond the SOVD spec,
      which only defines them for apps and components. Areas provide ``/data``,
      ``/data-categories``, ``/data-groups``, ``/operations``, ``/configurations``,
      ``/faults``, ``/logs`` (namespace prefix aggregation), read-only ``/bulk-data``,
      and ``/triggers``. See :ref:`sovd-compliance` for details.

Components
~~~~~~~~~~

``GET /api/v1/components``
   List all components with their operations and capabilities.

   **Example Response:**

   .. code-block:: json

      {
        "items": [
          {
            "id": "temp_sensor",
            "name": "temp_sensor",
            "href": "/api/v1/components/temp_sensor"
          }
        ]
      }

``GET /api/v1/components/{component_id}``
   Get component capabilities including available resource collections.

   .. note::

      **ros2_medkit extension:** When a component carries an asset-identity
      nameplate - from the manifest ``identity:`` block (see
      :doc:`/config/manifest-schema`) or a protocol device-info read (e.g. the
      OPC UA BuildInfo/DI nameplate) - both the list and detail responses
      include it under ``x-medkit.identity``. Only populated fields are
      emitted (camelCase), ``extra`` holds vendor-specific keys, and
      ``_provenance`` records which source set each field (keys use the
      snake_case field names; ``extra`` entries are prefixed with
      ``extra.``).

   .. code-block:: json

      {
        "id": "plc_runtime",
        "name": "PLC Runtime",
        "x-medkit": {
          "source": "opcua",
          "identity": {
            "manufacturer": "SelfPatch Devices",
            "model": "SPX-1000",
            "serialNumber": "SN-0001-TEST",
            "hardwareRevision": "HW-A2",
            "softwareVersion": "SW-3.4.5",
            "networkEndpoint": "opc.tcp://plc.local:4840",
            "extra": {
              "buildNumber": "build-4567"
            },
            "_provenance": {
              "manufacturer": "opcua",
              "serial_number": "opcua",
              "extra.buildNumber": "opcua"
            }
          }
        }
      }

``GET /api/v1/components/{component_id}/hosts``
   List apps hosted on this component (SOVD 7.6.2.4).

``GET /api/v1/components/{component_id}/depends-on``
   List component dependencies.

Apps
~~~~

``GET /api/v1/apps``
   List all apps discovered by the gateway.

   The set of apps is populated either from the static manifest (manifest or hybrid mode)
   or via heuristic runtime discovery of ROS 2 nodes (see :doc:`/tutorials/heuristic-apps`).
   This endpoint may return an empty list if no apps are discovered or if app discovery is
   disabled in the gateway configuration.

``GET /api/v1/apps/{app_id}``
   Get capabilities for a single discovered app.

``GET /api/v1/apps/{app_id}/is-located-on``
   Return the parent component that hosts this app.

   The response follows the standard ``items`` wrapper and returns:

   - ``0`` items when the app has no associated host component
   - ``1`` item when the host component is resolved
   - ``1`` item with ``x-medkit.missing=true`` when the app references a host
     component that cannot currently be resolved

   **Example Response:**

   .. code-block:: json

      {
        "items": [
          {
            "id": "temp-sensor-hw",
            "name": "Temperature Sensor",
            "href": "/api/v1/components/temp-sensor-hw"
          }
        ],
        "x-medkit": {
          "total_count": 1
        },
        "_links": {
          "self": "/api/v1/apps/engine-temp-sensor/is-located-on",
          "app": "/api/v1/apps/engine-temp-sensor"
        }
      }

   Unknown apps return ``404 App not found`` with ``parameters.app_id``.

``GET /api/v1/apps/{app_id}/belongs-to``
   Return the area that contains this app via its parent component.

   Per SOVD (ISO 17978-3 §7.6), the corresponding
   ``belongs-to`` URI reference in ``GET /apps/{app_id}`` is only emitted when
   the app has a parent component (i.e. is not standalone). Standalone apps do
   not expose this subresource in HATEOAS and the endpoint will return an empty
   ``items`` collection if called directly.

   The response follows the standard ``items`` wrapper and returns:

   - ``0`` items when the app has no associated host component (standalone app)
   - ``0`` items when the parent component has no assigned area
   - ``1`` item when the area is resolved
   - ``1`` item with ``x-medkit.missing=true`` when the parent component references
     an area that cannot currently be resolved
   - ``1`` item with ``x-medkit.missing=true`` and ``x-medkit.unresolved_component``
     set to the dangling component id when the app references a parent component
     that cannot currently be resolved (manifest broken / component removed)

   **Example Response:**

   .. code-block:: json

      {
        "items": [
          {
            "id": "engine",
            "name": "Engine",
            "href": "/api/v1/areas/engine"
          }
        ],
        "x-medkit": {
          "total_count": 1
        },
        "_links": {
          "self": "/api/v1/apps/engine-temp-sensor/belongs-to",
          "app": "/api/v1/apps/engine-temp-sensor"
        }
      }

   Unknown apps return ``404 App not found`` with ``parameters.app_id``.

Functions
~~~~~~~~~

``GET /api/v1/functions``
   List all functions (requires manifest mode or hybrid mode).

``GET /api/v1/functions/{function_id}``
   Get function capabilities.

``GET /api/v1/functions/{function_id}/hosts``
   List apps that host this function.

``GET /api/v1/functions/{function_id}/x-medkit-graph``
   Get a function-scoped topology snapshot with per-topic metrics and pipeline status.
   Served by the ``ros2_medkit_graph_provider`` plugin. **Requires a real
   ``/diagnostics`` producer** publishing ``DiagnosticStatus`` messages keyed on the
   exact fully-qualified topic name - without one, every edge stays ``"pending"``
   forever. See :doc:`/tutorials/graph-provider` for the full prerequisites and a
   worked walkthrough, and :doc:`/config/graph-provider` for the threshold reference.

``GET /api/v1/x-medkit-watchdog``
   Get the graph watchdog's reliability status: which entities it has observed, which are
   armed (past the bringup-quiesce warmup and therefore eligible to raise), and the cached
   lifecycle state of any managed lifecycle nodes. Served by the
   ``ros2_medkit_graph_watchdog`` plugin. Read-only; it raises nothing and changes nothing.

   The payload carries ``schema_version``, the configured ``warmup_cycles``, a global state,
   and a per-entity map. An entity that is present but not yet armed is normal during
   bringup: the watchdog holds every raise until an entity has been continuously present for
   ``warmup_cycles`` ticks, so a joining node does not read as a failure.

   Use it to answer "why has the watchdog not reported anything yet" without reading logs.

   **Example Response:**

   .. code-block:: json

      {
        "x-medkit-graph": {
          "schema_version": "2.0.0",
          "graph_id": "perception_graph-graph",
          "timestamp": "2026-03-08T12:00:00.000Z",
          "scope": {
            "type": "function",
            "entity_id": "perception_graph"
          },
          "pipeline_status": "degraded",
          "bottleneck_edge": "edge-1",
          "topics": [
            {
              "topic_id": "topic-1",
              "name": "/camera/front/image_raw"
            },
            {
              "topic_id": "topic-2",
              "name": "/camera/front/camera_info"
            }
          ],
          "nodes": [
            {
              "entity_id": "camera_front",
              "node_status": "reachable"
            },
            {
              "entity_id": "detector",
              "node_status": "reachable"
            }
          ],
          "edges": [
            {
              "edge_id": "edge-1",
              "source": "camera_front",
              "target": "detector",
              "topic_id": "topic-1",
              "transport_type": "unknown",
              "metrics": {
                "source": "/greenwave_monitor",
                "publisher_count": 2,
                "rate_ambiguous": true,
                "frequency_hz": 12.5,
                "latency_ms": 4.2,
                "drop_rate_percent": 0.0,
                "metrics_status": "active"
              }
            },
            {
              "edge_id": "edge-2",
              "source": "camera_front",
              "target": "detector",
              "topic_id": "topic-2",
              "transport_type": "unknown",
              "metrics": {
                "publisher_count": 1,
                "frequency_hz": null,
                "latency_ms": null,
                "drop_rate_percent": 0.0,
                "metrics_status": "pending"
              }
            }
          ]
        }
      }

   **Field Notes:**

   - ``schema_version``: semver contract on the document's shape and field semantics.
     A minor bump is additive/backward-compatible (new optional field, new enum value a
     tolerant client can ignore); a major bump means an existing field's shape or
     meaning changed and old parsing logic may break.
   - ``pipeline_status``: overall graph state, one of ``healthy``, ``degraded``, ``broken``.
     A graph where every edge is still ``pending`` reads as ``healthy`` - a pipeline
     never observed is not evidence of a broken one. A scoped node that is
     ``unreachable`` makes the status at least ``degraded``: a dead node carries no
     topics and so contributes no edge, but the Function is not healthy while it is down.
   - ``node_status``: per-node reachability, one of ``reachable``, ``unreachable``
   - ``last_seen``: present only on a node whose ``node_status`` is ``unreachable``;
     an ISO 8601 millisecond-precision timestamp of the last introspection cycle
     that saw the app online, when known (omitted for an app that has never been
     seen online). A node whose ``node_status`` is ``unreachable`` always has an
     empty ``topics.publishes``/``topics.subscribes`` list, so it never appears as
     ``source`` or ``target`` of any edge in the same document:

     .. code-block:: json

        {
          "entity_id": "old_lidar_node",
          "node_status": "unreachable",
          "last_seen": "2026-03-08T11:59:42.017Z"
        }

   - ``topic_id`` / ``edge_id``: **positional, not stable.** Assigned by enumeration
     order on every build (``topic-1``, ``topic-2``, ... / ``edge-1``, ``edge-2``, ...)
     and renumbered whenever the topic/edge set changes. Do not persist them or use
     them as a cross-request reference.
   - ``metrics_status``: per-edge telemetry state, one of:

     - ``pending`` - no ``/diagnostics`` sample has ever been merged for this topic
       (permanent until real data arrives)
     - ``active`` - a sample was merged within the freshness window (tracks freshness,
       not field completeness)
     - ``error`` - a sample was merged in the past, but the newest one has been older
       than the freshness window continuously for longer than ``stale_grace_sec``
       (default ``2.0`` s - a single late sample does not flip this immediately; see
       :doc:`/config/graph-provider`)
   - ``error_reason``: present only when ``metrics_status`` is ``error``; the only
     reachable value is ``metrics_stale``
   - ``metrics.source``: the resolved fully-qualified node name that published the
     ``/diagnostics`` message this edge's metrics were last updated from. **Omitted**
     on ``pending`` edges and on any edge whose most recent sample could not be
     attributed to a specific publisher - never a fabricated name or a hardcoded
     vendor literal. With a single ``/diagnostics`` publisher the sample resolves
     on every RMW. Telling several simultaneous ``/diagnostics`` publishers apart
     per sample needs an RMW whose message publisher GID matches the graph endpoint
     GID (``rmw_fastrtps_cpp``); on an RMW without that (e.g. ``rmw_cyclonedds_cpp``)
     a sample from one of several publishers is left unattributed (omitted) rather
     than guessed.
   - ``metrics.publisher_count``: live publisher count on this edge's DATA topic,
     from the ROS graph (independent of ``/diagnostics``). Emitted whenever that
     graph query resolved - even at ``1``, and even while ``metrics_status`` is
     still ``pending`` - and omitted only when the query never ran or came back
     empty (never a fabricated ``0``).
   - ``metrics.rate_ambiguous``: present (``true``) only when ``publisher_count``
     is greater than ``1``. ``frequency_hz`` is a topic-level arrival rate summed
     across every publisher on the topic, so a duplicate or leftover publisher can
     inflate it and mask a genuinely slow pipeline as healthy - this is the
     operator-facing signal that the rate number is suspect. See
     :doc:`/config/graph-provider`'s ``multi_publisher_rate`` setting for the
     policy controlling whether ``frequency_hz`` is still shown (and allowed to
     drive the degraded verdict) once this is true.
   - ``transport_type``: reserved and currently unpopulated. Always the literal
     ``"unknown"``.

   .. note::

      **ros2_medkit extension:** Functions support resource collections beyond the SOVD spec.
      ``/data`` and ``/operations`` aggregate from hosted apps (per SOVD). Additionally,
      ``/configurations``, ``/faults``, ``/logs`` aggregate from hosts, read-only
      ``/bulk-data`` is available, ``/cyclic-subscriptions`` and ``/triggers`` are
      supported, and the vendor resource ``/x-medkit-graph`` exposes a function-scoped
      graph snapshot. See :ref:`sovd-compliance` for details.

Data Endpoints
--------------

Read and publish data from ROS 2 topics.

``GET /api/v1/components/{id}/data``
   Read all topic data from an entity.

   **Example Response:**

   .. code-block:: json

      {
        "items": [
          {
            "name": "temperature",
            "data_id": "powertrain%2Fengine%2Ftemperature",
            "type": "std_msgs/msg/Float64",
            "value": {"data": 85.5},
            "timestamp": "2025-01-15T10:30:00Z"
          }
        ],
        "x-medkit": {
          "entity_id": "temp_sensor",
          "total_count": 1
        }
      }

``GET /api/v1/components/{id}/data/{topic_path}``
   Read specific topic data. Topic path is URL-encoded (``/`` → ``%2F``).

   **Example:**

   .. code-block:: bash

      curl http://localhost:8080/api/v1/components/temp_sensor/data/powertrain%2Fengine%2Ftemperature

``PUT /api/v1/components/{id}/data/{topic_path}``
   Publish to a topic.

   - **Content-Type:** application/json
   - **200:** Message published successfully
   - **400:** Invalid message format
   - **401:** Unauthorized (when auth enabled)

   **Example:**

   .. code-block:: bash

      curl -X PUT http://localhost:8080/api/v1/components/brake_actuator/data/chassis%2Fbrakes%2Fcommand \
        -H "Content-Type: application/json" \
        -d '{"data": 50.0}'

Operations Endpoints
--------------------

Execute ROS 2 services and actions.

List Operations
~~~~~~~~~~~~~~~

``GET /api/v1/components/{id}/operations``
   List all operations (services and actions) for an entity.

   **Example Response:**

   .. code-block:: json

      {
        "items": [
          {
            "id": "calibrate",
            "name": "calibrate",
            "type": "service",
            "service_type": "std_srvs/srv/Trigger",
            "schema": {
              "request": {},
              "response": {"success": "bool", "message": "string"}
            }
          },
          {
            "id": "long_calibration",
            "name": "long_calibration",
            "type": "action",
            "action_type": "example_interfaces/action/Fibonacci",
            "schema": {
              "goal": {"order": "int32"},
              "result": {"sequence": "int32[]"},
              "feedback": {"partial_sequence": "int32[]"}
            }
          }
        ],
        "x-medkit": {
          "entity_id": "calibration",
          "total_count": 2
        }
      }

``GET /api/v1/components/{id}/operations/{operation_id}``
   Get operation details and schema.

Execute Operations
~~~~~~~~~~~~~~~~~~

``POST /api/v1/components/{id}/operations/{operation_id}/executions``
   Execute an operation (service call or action goal).

   - **Content-Type:** application/json
   - **200:** Service call completed (sync)
   - **202:** Action goal accepted (async)
   - **400:** Invalid input
   - **404:** Operation not found

   **Service Example (synchronous):**

   .. code-block:: bash

      curl -X POST http://localhost:8080/api/v1/components/calibration/operations/calibrate/executions \
        -H "Content-Type: application/json" \
        -d '{}'

   **Action Example (asynchronous):**

   .. code-block:: bash

      curl -X POST http://localhost:8080/api/v1/components/calibration/operations/long_calibration/executions \
        -H "Content-Type: application/json" \
        -d '{"order": 10}'

   **Action Response (202 Accepted):**

   .. code-block:: json

      {
        "id": "abc123-def456",
        "status": "running"
      }

   The ``202`` carries a ``Location`` header naming the new execution. It is
   the request path plus the execution id, so it stays inside the collection
   the caller addressed: a POST to ``/api/v1/functions/powertrain/...`` is
   answered with a ``/api/v1/functions/powertrain/...`` execution URI, never a
   ``/components/`` one.

``GET /api/v1/components/{id}/operations/{operation_id}/executions``
   List all executions for an operation. Available on every entity type that
   lists the operation - areas, components, apps and functions.

   An operation id that is not an action has no executions and lists empty:
   only ROS 2 actions produce a tracked execution.

``GET /api/v1/components/{id}/operations/{operation_id}/executions/{execution_id}``
   Get execution status and result.

   Executions belong to the entity they were started on. Reading, updating or
   cancelling one through a different entity's URI answers ``404`` even when
   the execution id exists, and the listing above shows an entity only the
   executions started through it. The same action reached through two entities
   (an app and the function that aggregates it) therefore keeps two separate
   execution collections.

   **Example Response (completed action):**

   .. code-block:: json

      {
        "execution_id": "abc123-def456",
        "status": "succeeded",
        "result": {"sequence": [0, 1, 1, 2, 3, 5, 8, 13, 21, 34]},
        "feedback": [
          {"partial_sequence": [0, 1]},
          {"partial_sequence": [0, 1, 1, 2, 3]}
        ]
      }

``DELETE /api/v1/components/{id}/operations/{operation_id}/executions/{execution_id}``
   Cancel a running execution.

   - **204:** Execution cancelled
   - **404:** Execution not found

Lifecycle Endpoints
-------------------

Read the lifecycle status of an entity and request lifecycle transitions.
Available for apps and components only.

Lifecycle control is delegated to a ``LifecycleProvider`` registered by a
substrate-owning plugin (ROS 2 lifecycle nodes, process/container/systemd, host
reboot). When no provider is registered for the entity, the status read returns
a value derived from runtime state and the transitions return ``501``.

Read Status
~~~~~~~~~~~

``GET /api/v1/apps/{app_id}/status``

``GET /api/v1/components/{component_id}/status``
   Return the current lifecycle status of the entity.

   The response ``status`` is ``ready`` or ``notReady``. Each supported
   transition is advertised as a URI field (``start``, ``restart``,
   ``force-restart``, ``shutdown``, ``force-shutdown``), present only when the
   entity's provider supports it. Without a provider, status is derived from
   runtime state (an App is ``ready`` while its node is online; the host
   Component is ``ready`` while reachable, any other Component is ``ready`` while
   at least one hosted App is online) and no transition URIs are returned.

   - **200:** Lifecycle status
   - **404:** Entity not found

   .. code-block:: json

      {
        "status": "ready",
        "restart": "/api/v1/apps/temp_sensor/status/restart"
      }

Request Transition
~~~~~~~~~~~~~~~~~~~

``PUT /api/v1/apps/{app_id}/status/{transition}``

``PUT /api/v1/components/{component_id}/status/{transition}``
   Request a lifecycle transition. ``{transition}`` is one of ``start``,
   ``restart``, ``force-restart``, ``shutdown``, ``force-shutdown``. The call is
   asynchronous: it returns on acceptance and the client polls the status read.

   RBAC: ``start`` / ``restart`` / ``force-restart`` require the ``operator``
   role; the destructive ``shutdown`` / ``force-shutdown`` require
   ``configurator``.

   - **202:** Transition accepted (the ``Location`` header points to the status URI)
   - **403:** Two different refusals share this status. The auth middleware
     rejects a caller without the role above, ahead of the handler, in the
     RFC 6749 shape (``{"error": "insufficient_scope", ...}``). The lifecycle
     provider rejects the transition itself in the SOVD shape
     (``{"error_code": "insufficient-access-rights", ...}``). Read
     ``error`` vs ``error_code`` to tell them apart.
   - **404:** Entity not found
   - **409:** A precondition was not fulfilled (``precondition-not-fulfilled``)
   - **501:** No lifecycle provider is registered for the entity (``not-implemented``)

   .. code-block:: bash

      curl -X PUT http://localhost:8080/api/v1/apps/temp_sensor/status/restart

Configurations Endpoints
------------------------

Manage ROS 2 node parameters.

``GET /api/v1/components/{id}/configurations``
   List all parameters for an entity.

   **Example Response:**

   .. code-block:: json

      {
        "items": [
          {
            "id": "publish_rate",
            "name": "publish_rate",
            "type": "double"
          },
          {
            "id": "sensor_id",
            "name": "sensor_id",
            "type": "string"
          }
        ],
        "x-medkit": {
          "entity_id": "temp_sensor",
          "total_count": 2
        }
      }

``GET /api/v1/components/{id}/configurations/{param_name}``
   Get a specific parameter value.

``PUT /api/v1/components/{id}/configurations/{param_name}``
   Set a parameter value.

   - **Content-Type:** application/json
   - **200:** Parameter updated
   - **400:** Invalid value - the node rejected it, or it cannot be converted to
     the parameter's type at all (e.g. a mixed-type array). Both are the
     caller's body, so both are ``400``, never ``500``.
   - **404:** Parameter not found

   **Example:**

   .. code-block:: bash

      curl -X PUT http://localhost:8080/api/v1/components/temp_sensor/configurations/publish_rate \
        -H "Content-Type: application/json" \
        -d '{"data": 20.0}'

``DELETE /api/v1/components/{id}/configurations/{param_name}``
   Reset parameter to default value.

``DELETE /api/v1/components/{id}/configurations``
   Reset all parameters to default values.

Resource Locking
----------------

SOVD resource locking for preventing concurrent modification of entity state.
See :doc:`locking` for the full API reference.

Faults Endpoints
----------------

Query and manage faults.

.. note::

   Faults are reported by ROS 2 nodes via the FaultReporter library, not via REST API.
   The gateway queries faults from the ros2_medkit_fault_manager node.

.. note::

   **Per-entity fault scope (``/{entity-path}/faults`` routes).** The gateway keys
   faults by ``fault_code`` only, and a fault's ``reporting_sources`` set is the
   union of every app that has reported that code. Per-entity routes apply a
   strict all-sources scope check: a fault is in scope for an entity iff **every**
   entry in ``reporting_sources`` is an app owned by that entity (exact FQN
   match, or strict path-child).

   This means a ``fault_code`` reported by apps in two different entities
   (for example ``SENSOR_TIMEOUT`` reported by both the lidar and the
   temperature sensor app) is **not** visible or clearable through either
   entity's per-entity routes - per-fault routes return ``404``, collection
   responses omit it, and per-entity ``DELETE`` skips it. To see, list, or
   clear such shared faults use the global ``GET /api/v1/faults`` /
   ``DELETE /api/v1/faults`` routes.

``GET /api/v1/faults``
   List all faults across the system.

``GET /api/v1/components/{id}/faults``
   List all faults for an entity.

   Both endpoints accept an optional ``?status=`` query parameter:

   +-----------------+--------------------------------------------------+
   | Value           | Returns                                          |
   +=================+==================================================+
   | *(default)*     | ``PREFAILED`` + ``CONFIRMED`` (active faults)    |
   +-----------------+--------------------------------------------------+
   | ``pending``     | ``PREFAILED`` only                               |
   +-----------------+--------------------------------------------------+
   | ``confirmed``   | ``CONFIRMED`` only                               |
   +-----------------+--------------------------------------------------+
   | ``cleared``     | ``CLEARED`` + ``HEALED`` + ``PREPASSED``         |
   |                 | (SOVD "cleared" semantics)                       |
   +-----------------+--------------------------------------------------+
   | ``healed``      | ``HEALED`` + ``PREPASSED`` only                  |
   +-----------------+--------------------------------------------------+
   | ``all``         | All statuses                                     |
   +-----------------+--------------------------------------------------+

   **Example Response:**

   .. code-block:: json

      {
        "items": [
          {
            "fault_code": "LIDAR_RANGE_INVALID",
            "severity": "ERROR",
            "message": "Invalid range configuration: min_range > max_range",
            "timestamp": "2025-01-15T10:30:00Z",
            "source": "lidar_driver"
          }
        ],
        "x-medkit": {
          "entity_id": "lidar_sensor",
          "total_count": 1
        }
      }

``GET /api/v1/components/{id}/faults/{fault_code}``
   Get details of a specific fault including environment data.

   **Example Response (200 OK):**

   .. code-block:: json

      {
        "item": {
          "code": "MOTOR_OVERHEAT",
          "fault_name": "Motor temperature exceeded threshold",
          "severity": 2,
          "status": {
            "aggregatedStatus": "active",
            "testFailed": "1",
            "confirmedDTC": "1",
            "pendingDTC": "0"
          }
        },
        "environment_data": {
          "extended_data_records": {
            "first_occurrence": "2026-02-04T10:30:00.000Z",
            "last_occurrence": "2026-02-04T10:35:00.000Z"
          },
          "snapshots": [
            {
              "type": "freeze_frame",
              "name": "motor_temperature",
              "data": 105.5,
              "x-medkit": {
                "topic": "/motor/temperature",
                "message_type": "sensor_msgs/msg/Temperature",
                "full_data": {"temperature": 105.5, "variance": 0.1},
                "captured_at": "2026-02-04T10:30:00.123Z"
              }
            },
            {
              "type": "rosbag",
              "name": "fault_recording",
              "bulk_data_uri": "/apps/motor_controller/bulk-data/rosbags/550e8400-e29b-41d4-a716-446655440000",
              "size_bytes": 1234567,
              "duration_sec": 6.0,
              "format": "mcap"
            }
          ]
        },
        "x-medkit": {
          "occurrence_count": 3,
          "reporting_sources": ["/powertrain/motor_controller"],
          "severity_label": "ERROR"
        }
      }

   **Status Object:**

   The ``status`` object follows SOVD fault status specification:

   - ``aggregatedStatus``: Overall status (``active``, ``passive``, ``cleared``)
   - ``testFailed``: Test failed indicator (``0`` or ``1``)
   - ``confirmedDTC``: Confirmed DTC indicator (``0`` or ``1``)
   - ``pendingDTC``: Pending DTC indicator (``0`` or ``1``)

   **Snapshot Types:**

   - ``freeze_frame``: Data captured at fault confirmation. Entity frames for
     faults that were already confirmed when the gateway started are captured
     at gateway start instead and carry ``"capture_origin": "startup"`` in
     their ``x-medkit`` block. For a plugin-backed entity that reports its
     link down, the values are the plugin's last known ones and may predate
     the confirmation by the length of the outage; such entries carry
     ``connected`` (the payload's link flag, ``false`` for the loss-of-comms
     case) and ``source_timestamp`` (the payload's own timestamp, verbatim)
     in ``x-medkit``, both only when the plugin's payload reports them.
     ``captured_at`` always dates the capture, not the values.
   - ``rosbag``: Recording file available via bulk-data endpoint

   **Response codes:**

   - **200:** Fault details
   - **404:** Fault not found, or reported by an app outside this entity's scope
   - **503:** Fault manager unavailable

``DELETE /api/v1/components/{id}/faults/{fault_code}``
   Clear a fault.

   - **204:** Fault cleared
   - **404:** Fault not found, or reported by an app outside this entity's scope

``DELETE /api/v1/components/{id}/faults``
   Clear all faults for an entity.

   Accepts the optional ``?status=`` query parameter (same values as ``GET /faults``).
   Without it, clears pending and confirmed faults.

   - **204:** Faults cleared (or none to clear)
   - **400:** Invalid status parameter
   - **503:** Fault manager unavailable

``DELETE /api/v1/faults``
   Clear all faults across the system *(ros2_medkit extension, not SOVD)*.

   Accepts the optional ``?status=`` query parameter (same values as ``GET /faults``).
   Without it, clears pending and confirmed faults.

   - **204:** Faults cleared (or none to clear)
   - **400:** Invalid status parameter
   - **503:** Fault manager unavailable

Logs Endpoints
--------------

Query and configure the /rosout ring buffer for an entity. Supported entity types:
**areas** (aggregated from hosted apps, merged with a namespace prefix query), **components**
(aggregated from hosted apps, merged with a namespace prefix query), **apps** (exact FQN match;
external apps by bare entity id), and **functions** (aggregated from hosted apps).

.. note::

   By default, log entries are sourced from the ``/rosout`` ROS 2 topic. ros2_medkit retains
   the 200 most recent entries per node in an in-memory ring buffer (configurable via
   ``logs.buffer_size`` in ``gateway_params.yaml``). A ``LogProvider`` plugin can replace the
   storage backend or take full ownership of the log pipeline (see plugin development docs).

``GET /api/v1/components/{id}/logs``
   Query log entries aggregated from the component's hosted apps. Child apps resolve via
   the entity cache using the same rule as fault scoping: an external app (plugin-provided,
   no ROS binding) is queried by its bare entity id, every other app by its exact FQN, and
   an external component also returns entries stored under its own id. When the component
   declares a non-empty namespace, a namespace prefix query runs in addition and the results
   are merged and deduplicated, so ROS nodes under the namespace keep contributing even when
   the component also hosts external apps. The response always carries
   ``x-medkit.aggregation_level=component`` and ``aggregated=true``; the ``app_count`` and
   ``aggregation_sources`` fields are populated only when at least one hosted source
   resolves.

``GET /api/v1/apps/{id}/logs``
   Query log entries for the specific app. A ROS-bound app is queried by its exact FQN; an
   external app by its bare entity id (the id ``add_log_entry`` and fault reporting use).

**Query parameters:**

.. list-table::
   :header-rows: 1
   :widths: 20 80

   * - Parameter
     - Description
   * - ``severity``
     - Minimum severity filter (``debug`` | ``info`` | ``warning`` | ``error`` | ``fatal``).
       The stricter of this parameter and the entity's configured ``severity_filter`` is applied.
       Without this parameter, the entity's configured ``severity_filter`` (default: ``debug``)
       determines the minimum level. Empty or absent = use entity config only.
   * - ``context``
     - Substring filter applied to the log entry's logger name (``context.node`` in the response).
       Maximum length: 256 characters. Empty or absent = no filter.

**Response 200:**

.. code-block:: json

   {
     "items": [
       {
         "id": "log_42",
         "timestamp": "2026-01-15T10:30:00.123456789Z",
         "severity": "warning",
         "message": "Calibration drift detected",
         "context": {
           "node": "powertrain/engine/temp_sensor",
           "function": "read_sensor",
           "file": "temp_sensor.cpp",
           "line": 99
         }
       }
     ]
   }

The ``context.function``, ``context.file``, and ``context.line`` fields are omitted when empty/zero.

**Severity values** map directly to the ROS 2 log levels:

.. list-table::
   :header-rows: 1
   :widths: 15 15 70

   * - Value
     - ROS 2 level
     - Meaning
   * - ``debug``
     - DEBUG (10)
     - Fine-grained diagnostic information
   * - ``info``
     - INFO (20)
     - Normal operational messages
   * - ``warning``
     - WARN (30)
     - Non-fatal anomalies
   * - ``error``
     - ERROR (40)
     - Errors that may require attention
   * - ``fatal``
     - FATAL (50)
     - Critical failures

``GET /api/v1/components/{id}/logs/configuration`` / ``GET /api/v1/apps/{id}/logs/configuration``
   Return the current log configuration for the entity.

   **Response 200:**

   .. code-block:: json

      {
        "severity_filter": "debug",
        "max_entries": 100
      }

``PUT /api/v1/components/{id}/logs/configuration`` / ``PUT /api/v1/apps/{id}/logs/configuration``
   Update the log configuration for the entity. All body fields are optional.

   **Request body:**

   .. code-block:: json

      {
        "severity_filter": "warning",
        "max_entries": 500
      }

   ``severity_filter`` - minimum severity to return in query results (``debug`` | ``info`` | ``warning`` |
   ``error`` | ``fatal``). Entries below this level are excluded from queries. Default: ``debug``.

   ``max_entries`` - maximum number of entries returned per query. Must be between 1 and 10,000
   (inclusive). Default: ``100``.

   **Response 204:** No content.

   - **400:** Invalid ``severity_filter`` or ``max_entries`` value

Bulk Data Endpoints
-------------------

Access, upload, and delete large binary data (rosbags, calibration files, firmware, etc.)
associated with entities. Read endpoints (GET) support all entity types. Write endpoints
(POST, DELETE) are supported for components and apps only.

List Categories
~~~~~~~~~~~~~~~

``GET /api/v1/{entity-path}/bulk-data``

List available bulk-data categories for an entity. Returns the union of rosbag categories
(from the fault manager) and configured categories (from ``bulk_data.categories``).

**Supported entity paths:**

- ``/apps/{app-id}``
- ``/components/{component-id}``
- ``/areas/{area-id}``
- ``/functions/{function-id}``
- ``/areas/{area-id}/subareas/{subarea-id}``
- ``/components/{component-id}/subcomponents/{subcomponent-id}``

**Example:**

.. code-block:: bash

   curl http://localhost:8080/api/v1/apps/motor_controller/bulk-data

**Response (200 OK):**

.. code-block:: json

   {
     "items": ["rosbags", "calibration", "firmware"]
   }

List Bulk Data Items
~~~~~~~~~~~~~~~~~~~~

``GET /api/v1/{entity-path}/bulk-data/{category}``

List all bulk-data items in a category for the entity.

**Example:**

.. code-block:: bash

   curl http://localhost:8080/api/v1/apps/motor_controller/bulk-data/rosbags

**Response (200 OK):**

.. code-block:: json

   {
     "items": [
       {
         "id": "550e8400-e29b-41d4-a716-446655440000",
         "name": "MOTOR_OVERHEAT recording 2026-02-04T10:30:00Z",
         "mimetype": "application/x-mcap",
         "size": 1234567,
         "creation_date": "2026-02-04T10:30:00.000Z",
         "x-medkit": {
           "fault_code": "MOTOR_OVERHEAT",
           "duration_sec": 6.0,
           "format": "mcap",
           "recording_id": "fault_MOTOR_OVERHEAT_1738664999000"
         }
       }
     ]
   }

For ``rosbags``, faults confirmed in one burst share a single recording: each
fault gets its own descriptor with the full bag size, and
``x-medkit.recording_id`` (the bag directory name) is the same for every
descriptor served from that recording, so clients can group them.

Download Bulk Data
~~~~~~~~~~~~~~~~~~

``GET /api/v1/{entity-path}/bulk-data/{category}/{id}``

Download a specific bulk-data file.

**Response Headers:**

- ``Content-Type``: the media type of the stored artifact - see below
- ``Content-Disposition``: ``attachment; filename="FAULT_CODE.mcap"``
- ``Accept-Ranges``: ``bytes`` - the download is served by a range-aware
  provider, so a client may fetch part of the file
- ``Access-Control-Expose-Headers``: ``Content-Disposition``

**Media types.** The OpenAPI document declares
``application/x-mcap``, ``application/x-sqlite3`` and
``application/octet-stream`` for this response, followed by ``*/*``. That is
not hedging: for the ``rosbags`` category the type is derived from the
recorded storage format and is one of the three named types, but every other
category serves back the media type recorded when the file was uploaded, which
is chosen by the uploading client. Uploading a ``text/csv`` makes the download
serve ``text/csv``. The named types are declared because they *are*
derivable; ``*/*`` is declared because the rest genuinely is not.

There is no response schema, for either status. The body is raw file content,
and OpenAPI 3.1 has no way to say "bytes" - ``format: binary`` was an OpenAPI
3.0 idiom that 3.1 dropped when it aligned with JSON Schema 2020-12. A
schema-free media type entry is the accurate description.

**Range requests.** A request carrying a ``Range`` header is answered with
**206 Partial Content** and a ``Content-Range: bytes <start>-<end>/<total>``
header instead of ``200``; the body is the requested slice. Several ranges in
one request are answered as a single ``multipart/byteranges`` body, which is
declared on the 206 only - the 200 can never carry it.

**Example:**

.. code-block:: bash

   curl -O -J http://localhost:8080/api/v1/apps/motor_controller/bulk-data/rosbags/550e8400-e29b-41d4-a716-446655440000

   # One byte range
   curl -H 'Range: bytes=0-1023' \
     http://localhost:8080/api/v1/apps/motor_controller/bulk-data/rosbags/550e8400-e29b-41d4-a716-446655440000

**Response Codes:**

- **200 OK**: File content
- **206 Partial Content**: The byte range requested via ``Range``, with ``Content-Range``
- **404 Not Found**: Entity, category, or bulk-data ID not found
- **416 Range Not Satisfiable**: The ``Range`` header could not be parsed. Not
  specific to this endpoint - see :ref:`rest-range-rejection`.

Upload Bulk Data
~~~~~~~~~~~~~~~~

``POST /api/v1/{entity-path}/bulk-data/{category}``

Upload a new bulk-data file to the specified category. Files are sent as
``multipart/form-data``. The ``rosbags`` category is read-only and cannot be
used for uploads.

**Supported entity types:** components, apps only. Areas and functions return 405.

**Form Fields:**

.. list-table::
   :header-rows: 1
   :widths: 20 15 65

   * - Field
     - Required
     - Description
   * - ``file``
     - Yes
     - The file to upload (binary data with filename and content type).
   * - ``description``
     - No
     - Human-readable description of the file.
   * - ``metadata``
     - No
     - JSON string with arbitrary key-value metadata.

**Example:**

.. code-block:: bash

   curl -X POST http://localhost:8080/api/v1/components/motor_controller/bulk-data/calibration \
     -F "file=@calibration_data.bin;type=application/octet-stream" \
     -F "description=Motor calibration parameters v2.1" \
     -F 'metadata={"version": "2.1", "author": "engineer_01"}'

**Response (201 Created):**

.. code-block:: json

   {
     "id": "calibration_1739612345000000000_ab12cd34",
     "name": "calibration_data.bin",
     "mimetype": "application/octet-stream",
     "size": 4096,
     "creation_date": "2026-03-15T14:30:00.000Z",
     "description": "Motor calibration parameters v2.1",
     "x-medkit": {
       "version": "2.1",
       "author": "engineer_01"
     }
   }

**Response Headers:**

- ``Location``: ``/api/v1/components/motor_controller/bulk-data/calibration/calibration_1739612345000000000_ab12cd34``

**Error Responses:**

- **400 Bad Request**: Missing ``file`` field, unknown category, or ``rosbags`` category
- **405 Method Not Allowed**: Upload attempted on areas or functions
- **413 Payload Too Large**: File exceeds ``bulk_data.max_upload_size``

Delete Bulk Data
~~~~~~~~~~~~~~~~

``DELETE /api/v1/{entity-path}/bulk-data/{category}/{id}``

Delete a specific bulk-data item. The ``rosbags`` category is managed by the
fault manager and cannot be deleted via this endpoint.

**Supported entity types:** components, apps only. Areas and functions return 405.

**Example:**

.. code-block:: bash

   curl -X DELETE http://localhost:8080/api/v1/components/motor_controller/bulk-data/calibration/calibration_1739612345000000000_ab12cd34

**Response Codes:**

- **204 No Content**: Item deleted successfully
- **400 Bad Request**: ``rosbags`` category (managed by fault manager)
- **404 Not Found**: Entity, category, or bulk-data ID not found
- **405 Method Not Allowed**: Delete attempted on areas or functions

Software Updates
----------------

Manage software update packages with an async prepare/execute lifecycle.
The updates feature requires a plugin implementing ``UpdateProvider`` to be loaded
via the plugin framework (see :doc:`/config/server`).
Without such a plugin, all endpoints return ``501 Not Implemented``.

``GET /api/v1/updates``
   List all registered update packages.

   **Query Parameters:**

   - ``origin`` (optional): Filter by origin (``remote`` or ``proximity``)
   - ``target-version`` (optional): Filter by target version

   **Example Response (200 OK):**

   .. code-block:: json

      {
        "items": ["firmware-v2.1", "calibration-update-3"]
      }

``POST /api/v1/updates``
   Register a new update package.

   **Request Body:**

   .. code-block:: json

      {
        "id": "firmware-v2.1",
        "update_name": "Firmware Update v2.1",
        "automated": true,
        "origins": ["remote"],
        "duration": 600,
        "size": 52428800,
        "updated_components": ["ecu_main"],
        "affected_components": ["ecu_main", "ecu_secondary"]
      }

   **Response (201 Created):**

   .. code-block:: json

      {
        "id": "firmware-v2.1"
      }

   **Response Headers:**

   - ``Location``: ``/api/v1/updates/firmware-v2.1``

``GET /api/v1/updates/{id}``
   Get full metadata for a specific update package.

   **Response (200 OK):**

   Returns the JSON metadata as registered.

   - **404 Not Found:** Package does not exist

``DELETE /api/v1/updates/{id}``
   Delete an update package.

   - **204 No Content:** Package deleted
   - **404 Not Found:** Package does not exist
   - **409 Conflict:** Operation in progress for this package

``PUT /api/v1/updates/{id}/prepare``
   Trigger preparation of an update (download, verify, check dependencies).
   Runs asynchronously - poll the status endpoint for progress.

   - **202 Accepted:** Preparation started
   - **404 Not Found:** Package does not exist
   - **409 Conflict:** Operation already in progress

   **Response Headers:**

   - ``Location``: ``/api/v1/updates/{id}/status``

``PUT /api/v1/updates/{id}/execute``
   Trigger execution of a prepared update (install). Only succeeds after
   prepare has completed.

   - **202 Accepted:** Execution started
   - **400 Bad Request:** Package not prepared
   - **404 Not Found:** Package does not exist
   - **409 Conflict:** Operation already in progress

   **Response Headers:**

   - ``Location``: ``/api/v1/updates/{id}/status``

``PUT /api/v1/updates/{id}/automated``
   Trigger automated update (prepare + execute in one step). Only works
   for packages that support automated mode.

   - **202 Accepted:** Automated update started
   - **400 Bad Request:** Package does not support automated mode
   - **404 Not Found:** Package does not exist
   - **409 Conflict:** Operation already in progress

   **Response Headers:**

   - ``Location``: ``/api/v1/updates/{id}/status``

``GET /api/v1/updates/{id}/status``
   Get the current status and progress of an update operation.

   **Example Response (200 OK):**

   .. code-block:: json

      {
        "status": "inProgress",
        "progress": 65,
        "sub_progress": [
          {"name": "download", "progress": 100},
          {"name": "verify", "progress": 30}
        ],
        "x-medkit": {
          "phase": "preparing"
        }
      }

   **Status values:** ``pending``, ``inProgress``, ``completed``, ``failed``

   A successful ``POST /api/v1/updates`` seeds a ``pending`` status for the package,
   so this endpoint returns ``200`` with ``{"status": "pending", "x-medkit": {"phase": "none"}}``
   immediately after registration, before any ``prepare`` or ``execute`` call.

   **Vendor extension ``x-medkit.phase``** (non-standard, SOVD-compatible):
   ``none``, ``preparing``, ``prepared``, ``executing``, ``executed``,
   ``failed``, ``deleting``. Differentiates "prepare completed" (``status``
   ``completed`` + ``x-medkit.phase`` ``prepared``) from "execute completed"
   (``status`` ``completed`` + ``x-medkit.phase`` ``executed``). Clients that
   only consume the standard ``status`` field continue to work unchanged.

   When ``status`` is ``failed``, an ``error`` object is included:

   .. code-block:: json

      {
        "status": "failed",
        "error": {
          "error_code": "internal-error",
          "message": "Download failed: connection timeout"
        }
      }

   - **404 Not Found:** Package is not registered

Cyclic Subscriptions
--------------------

Cyclic subscriptions provide periodic push-based delivery of any SOVD resource collection
via Server-Sent Events (SSE). A client creates a subscription specifying the resource URI
(data, faults, configurations, logs, or ``x-`` vendor extensions) and a delivery interval.
The server then pushes the latest value at the requested frequency.

Subscriptions are temporary - they do not survive server restart.

**Supported collections:**

- ``data`` - Topic data (requires a resource path, e.g. ``/data/temperature``)
- ``faults`` - Fault list (resource path optional, e.g. ``/faults`` or ``/faults/fault_001``)
- ``configurations`` - Parameter values (resource path optional)
- ``logs`` - Application log entries from ``/rosout``
- ``x-*`` - Vendor extensions (e.g. ``x-medkit-graph``)

**Interval values:**

- ``fast`` - 50ms sampling period
- ``normal`` - 200ms sampling period (default)
- ``slow`` - 500ms sampling period

``POST /api/v1/{entity_type}/{entity_id}/cyclic-subscriptions``
   Create a new cyclic subscription.

   Response: **201 Created** with a ``Location`` header pointing to the new
   subscription.

   **Applies to:** ``/apps``, ``/components``, ``/functions``

   **Request Body:**

   .. code-block:: json

      {
        "resource": "/api/v1/apps/temp_sensor/data/temperature",
        "protocol": "sse",
        "interval": "normal",
        "duration": 300
      }

   **Fields:**

   - ``resource`` (string, required): Full SOVD resource URI to observe
     (e.g. ``/api/v1/apps/{id}/data/{topic}``, ``/api/v1/apps/{id}/faults``,
     ``/api/v1/functions/{id}/x-medkit-graph``)
   - ``protocol`` (string, optional): Transport protocol. Only ``"sse"`` supported. Default: ``"sse"``
   - ``interval`` (string, required): One of ``fast``, ``normal``, ``slow``
   - ``duration`` (integer, required): Subscription lifetime in seconds.
     Must be > 0 and <= ``sse.max_duration_sec`` (default: 3600)

   **Error responses:**

   - **400** ``invalid-parameter`` - Invalid interval, duration <= 0, or duration exceeds max
   - **400** ``x-medkit-invalid-resource-uri`` - Malformed resource URI or path traversal
   - **400** ``x-medkit-entity-mismatch`` - Resource URI references different entity than route
   - **400** ``x-medkit-collection-not-supported`` - Entity doesn't support the collection
   - **400** ``x-medkit-collection-not-available`` - No data provider registered for collection
   - **400** ``x-medkit-unsupported-protocol`` - Requested protocol not available
   - **503** ``service-unavailable`` - Max subscription capacity reached

   **Response 201 Created:**

   .. code-block:: json

      {
        "id": "sub_001",
        "observed_resource": "/api/v1/apps/temp_sensor/data/temperature",
        "event_source": "/api/v1/apps/temp_sensor/cyclic-subscriptions/sub_001/events",
        "protocol": "sse",
        "interval": "normal"
      }

``GET /api/v1/{entity_type}/{entity_id}/cyclic-subscriptions``
   List all active cyclic subscriptions for an entity. Returns ``{"items": [...]}``.

``GET /api/v1/{entity_type}/{entity_id}/cyclic-subscriptions/{id}``
   Get details of a single subscription.

``PUT /api/v1/{entity_type}/{entity_id}/cyclic-subscriptions/{id}``
   Update ``interval`` and/or ``duration`` of an existing subscription.
   Only provided fields are updated. Updating ``duration`` resets the
   expiry timer from the current time (not from the original creation time).

   **Request Body:**

   .. code-block:: json

      {
        "interval": "fast",
        "duration": 600
      }

``DELETE /api/v1/{entity_type}/{entity_id}/cyclic-subscriptions/{id}``
   Cancel and remove a subscription. Returns 204 No Content.

``GET /api/v1/{entity_type}/{entity_id}/cyclic-subscriptions/{id}/events``
   SSE event stream. Connect to receive periodic data updates.

   **Response Headers:**

   .. code-block:: text

      Content-Type: text/event-stream
      Cache-Control: no-cache
      Connection: keep-alive

   **Event Format (EventEnvelope):**

   .. code-block:: text

      data: {"timestamp":"2026-02-14T10:30:00.250Z","payload":{"id":"/temperature","data":{"data":23.5}}}

   The stream auto-closes when the duration expires, the client disconnects,
   or the subscription is deleted.

**Multi-collection examples:**

Subscribe to faults on a component:

.. code-block:: json

   {
     "resource": "/api/v1/components/ecu1/faults",
     "interval": "slow",
     "duration": 600
   }

Subscribe to a specific configuration parameter:

.. code-block:: json

   {
     "resource": "/api/v1/apps/temp_sensor/configurations/calibration_offset",
     "interval": "normal",
     "duration": 120
   }

Scripts
-------

Upload, manage, and execute diagnostic scripts on entities.
*(ISO 17978-3, 7.15)*

Scripts are available on **Components** and **Apps** entity types.
The feature must be enabled by setting ``scripts.scripts_dir`` in the gateway configuration.

Script Error Statuses
~~~~~~~~~~~~~~~~~~~~~

Beyond the usual 400 / 404 / 500, and 501 on every script endpoint when no
scripts backend is configured, each endpoint answers only what its own backend
call can produce. With the built-in backend:

.. list-table::
   :header-rows: 1
   :widths: 45 55

   * - Endpoint
     - Extra statuses
   * - ``POST .../scripts`` (upload)
     - **413** ``script-file-too-large`` - file over the configured size limit
   * - ``DELETE .../scripts/{script_id}``
     - **409** ``script-managed`` (manifest-owned, not editable) or
       ``script-running``
   * - ``POST .../scripts/{script_id}/executions``
     - **429** ``script-concurrency-limit``
   * - ``PUT .../executions/{execution_id}``
     - **409** ``script-not-running``
   * - ``DELETE .../executions/{execution_id}``
     - **409** ``script-running``

The listing and read endpoints (``GET .../scripts``,
``GET .../scripts/{script_id}``, ``GET .../executions/{execution_id}``) add
nothing to the blanket set.

The 429 is the **script manager's** concurrency limit, not the HTTP rate
limiter's: it is answered whether or not ``rate_limiting.enabled`` is set, and
carries no ``Retry-After`` or ``X-RateLimit-*`` headers. See
:ref:`rate-limiting` for the other 429. When the limiter is on, both can answer
429 on the execution-start route and the document can only describe one: the
route's own declaration wins, so that operation's 429 is documented as the
script manager's, without the limiter's headers. The body shape is the same
either way.

``script-already-exists`` is defined for backends that maintain their own
registry (a plugin with a SQLite store, say); the built-in backend generates
ids and never returns it.

Upload Script
~~~~~~~~~~~~~

``POST /api/v1/{entity_type}/{entity_id}/scripts``
   Upload a diagnostic script via ``multipart/form-data``.

   - **file** (required): The script file (Python, bash, or sh)
   - **metadata** (optional): JSON with name, description, parameters_schema

   Response: **201 Created** with ``Location`` header pointing to the new script.

   .. note::

      Uploads can be disabled by setting ``scripts.allow_uploads: false`` in the
      gateway configuration. When disabled, POST returns 400. Pre-deployed
      manifest scripts remain available for execution.

List Scripts
~~~~~~~~~~~~

``GET /api/v1/{entity_type}/{entity_id}/scripts``
   List all scripts for an entity. Returns ``{"items": [...]}``.

Get Script
~~~~~~~~~~

``GET /api/v1/{entity_type}/{entity_id}/scripts/{script_id}``
   Get metadata for a specific script.

Delete Script
~~~~~~~~~~~~~

``DELETE /api/v1/{entity_type}/{entity_id}/scripts/{script_id}``
   Delete an uploaded script. Returns **204 No Content**.
   Returns **409** if the script is manifest-managed or currently executing.

Start Execution
~~~~~~~~~~~~~~~

``POST /api/v1/{entity_type}/{entity_id}/scripts/{script_id}/executions``
   Start a new execution of a script.

   **Request Body:**

   .. code-block:: json

      {
        "execution_type": "now",
        "parameters": {"threshold": 0.1}
      }

   .. list-table::
      :header-rows: 1
      :widths: 25 15 10 50

      * - Attribute
        - Type
        - Conv
        - Description
      * - ``execution_type``
        - string
        - M
        - When to run. The shipped backend accepts only ``now``; see the note below
      * - ``parameters``
        - object
        - O
        - Input parameters for the script
      * - ``proximity_response``
        - string
        - O
        - Co-location proof token

   .. note::

      The built-in script backend supports only ``now``. Other execution types
      (``on_restart``, ``now_and_on_restart``, ``once_on_restart``) require a
      plugin-provided ScriptProvider and will return 400 ``invalid-parameter``
      if not supported.

   Response: **202 Accepted** with ``Location`` header pointing to the execution status.

Get Execution Status
~~~~~~~~~~~~~~~~~~~~

``GET /api/v1/{entity_type}/{entity_id}/scripts/{script_id}/executions/{execution_id}``
   Poll the status of a script execution.

   Status values: ``prepared``, ``running``, ``completed``, ``failed``, ``terminated``

Terminate Execution
~~~~~~~~~~~~~~~~~~~

``PUT /api/v1/{entity_type}/{entity_id}/scripts/{script_id}/executions/{execution_id}``
   Send a termination action to a running execution.

   **Request Body:**

   .. code-block:: json

      {"action": "stop"}

   Action values: ``stop`` (SIGTERM), ``forced_termination`` (SIGKILL).

Delete Execution
~~~~~~~~~~~~~~~~

``DELETE /api/v1/{entity_type}/{entity_id}/scripts/{script_id}/executions/{execution_id}``
   Remove a completed/terminated execution resource. Returns **204 No Content**.
   Returns **409** if the execution is still running.

Triggers
--------

Triggers provide condition-based push notifications for resource changes via
Server-Sent Events (SSE). Unlike cyclic subscriptions - which poll a resource
at a fixed interval and push every sample - triggers evaluate a condition
against each change and only fire when the condition is met.

**Key differences from Cyclic Subscriptions:**

- Cyclic subscriptions push data at a fixed interval (``fast``/``normal``/``slow``)
  regardless of whether the value changed
- Triggers are event-driven: they only fire when a specific condition is satisfied
  (e.g., value changed, entered a range, reached a threshold)
- Triggers support persistence across gateway restarts (``persistent: true``)
- Triggers can be one-shot (fire once, then auto-terminate) or multishot (continuous)

**Supported entity types:** ``/areas``, ``/components``, ``/apps``, ``/functions``

.. note::

   **ros2_medkit extension:** SOVD defines triggers for apps and components only.
   ros2_medkit extends trigger support to areas and functions, allowing
   hierarchy-scoped monitoring. Area-level triggers catch changes from all
   descendant entities within the area.

**Observable resource collections:**

- ``data`` - Topic data changes (driven by ``TriggerTopicSubscriber``)
- ``faults`` - Fault state transitions (created, updated, cleared)
- ``operations`` - Operation execution completions
- ``updates`` - Software update status changes
- ``logs`` - Log entries matching configured severity (x-medkit extension)

Create Trigger
~~~~~~~~~~~~~~

``POST /api/v1/{entity_type}/{entity_id}/triggers``
   Create a new condition-based trigger.

   Response: **201 Created** with a ``Location`` header pointing to the new
   trigger.

   **Request Body:**

   .. code-block:: json

      {
        "resource": "/api/v1/apps/temp_sensor/data/powertrain%2Fengine%2Ftemperature",
        "trigger_condition": {
          "condition_type": "LeaveRange",
          "lower_bound": 20.0,
          "upper_bound": 80.0
        },
        "path": "/data",
        "protocol": "sse",
        "multishot": true,
        "persistent": false,
        "lifetime": 300,
        "log_settings": {
          "severity": "warning",
          "marker": "Temperature threshold exceeded"
        }
      }

   **Fields:**

   .. list-table::
      :header-rows: 1
      :widths: 20 10 70

      * - Field
        - Required
        - Description
      * - ``resource``
        - Yes
        - Full SOVD resource URI to observe (e.g. ``/api/v1/apps/{id}/data/{topic}``,
          ``/api/v1/apps/{id}/faults``, ``/api/v1/areas/{id}/faults``).
          Must reference the same entity as the route.
      * - ``trigger_condition``
        - Yes
        - Object with ``condition_type`` and condition-specific parameters.
          See `Trigger Conditions`_ below.
      * - ``path``
        - No
        - JSON Pointer within the resource payload to evaluate. When set, the
          condition is evaluated against the value at this path instead of the
          full payload.
      * - ``protocol``
        - No
        - Transport protocol. Only ``"sse"`` is supported. Default: ``"sse"``.
      * - ``multishot``
        - No
        - If ``true``, the trigger fires repeatedly. If ``false``, the trigger
          auto-terminates after the first event. Default: ``false``.
      * - ``persistent``
        - No
        - If ``true``, the trigger survives gateway restarts (when
          ``on_restart_behavior`` is ``"restore"``). Default: ``false``.
      * - ``lifetime``
        - No
        - Time-to-live in seconds. The trigger auto-terminates after this
          duration. Must be a positive integer. Omit for no expiry.
      * - ``log_settings``
        - No
        - Temporary log entry injected when the trigger fires.
          Accepts ``severity`` (log level: ``debug``, ``info``, ``warning``,
          ``error``, ``fatal``; default: ``info``) and ``marker`` (descriptive
          message text; default: ``"Trigger fired"``). The log entry includes
          trigger metadata (trigger ID, condition type, resource URI).

   **Response 201 Created:**

   .. code-block:: json

      {
        "id": "trig_001",
        "status": "active",
        "observed_resource": "/api/v1/apps/temp_sensor/data/powertrain%2Fengine%2Ftemperature",
        "event_source": "/api/v1/apps/temp_sensor/triggers/trig_001/events",
        "protocol": "sse",
        "trigger_condition": {
          "condition_type": "LeaveRange",
          "lower_bound": 20.0,
          "upper_bound": 80.0
        },
        "multishot": true,
        "persistent": false,
        "lifetime": 300
      }

   **Error Responses:**

   - **400** ``invalid-parameter`` - Missing or invalid ``resource``, ``trigger_condition``,
     ``condition_type``, ``lifetime``, or condition-specific parameters
   - **400** ``x-medkit-invalid-resource-uri`` - Malformed resource URI or path traversal
   - **400** ``x-medkit-entity-mismatch`` - Resource URI references a different entity than
     the route
   - **503** ``service-unavailable`` - Maximum trigger capacity reached
     (configurable via ``triggers.max_triggers``)

List Triggers
~~~~~~~~~~~~~

``GET /api/v1/{entity_type}/{entity_id}/triggers``
   List all triggers for an entity.

   **Response 200:**

   .. code-block:: json

      {
        "items": [
          {
            "id": "trig_001",
            "status": "active",
            "observed_resource": "/api/v1/apps/temp_sensor/faults",
            "event_source": "/api/v1/apps/temp_sensor/triggers/trig_001/events",
            "protocol": "sse",
            "trigger_condition": {"condition_type": "OnChange"},
            "multishot": true,
            "persistent": false,
            "lifetime": 300
          }
        ]
      }

Get Trigger
~~~~~~~~~~~

``GET /api/v1/{entity_type}/{entity_id}/triggers/{trigger_id}``
   Get details of a single trigger.

   **Response 200:** Same schema as creation response.

   - **404** ``resource-not-found`` - Trigger not found or belongs to a different entity

Update Trigger
~~~~~~~~~~~~~~

``PUT /api/v1/{entity_type}/{entity_id}/triggers/{trigger_id}``
   Update the lifetime of an existing trigger. Updating ``lifetime`` resets the
   expiry timer from the current time.

   **Request Body:**

   .. code-block:: json

      {
        "lifetime": 600
      }

   **Response 200:** Updated trigger object (same schema as creation response).

   - **400** ``invalid-parameter`` - Missing or invalid ``lifetime``
   - **404** ``resource-not-found`` - Trigger not found

Delete Trigger
~~~~~~~~~~~~~~

``DELETE /api/v1/{entity_type}/{entity_id}/triggers/{trigger_id}``
   Remove a trigger. Any active SSE connection for this trigger is closed.

   - **204** No Content - Trigger deleted
   - **404** ``resource-not-found`` - Trigger not found

Trigger Events (SSE Stream)
~~~~~~~~~~~~~~~~~~~~~~~~~~~

``GET /api/v1/{entity_type}/{entity_id}/triggers/{trigger_id}/events``
   SSE event stream for a trigger. Connect to receive events when the trigger
   condition is met. The stream sends keepalive comments every 15 seconds.

   **Response Headers:**

   .. code-block:: text

      Content-Type: text/event-stream
      Cache-Control: no-cache

   **Frame format:**

   Each event is one SSE frame carrying an ``id:`` field and a ``data:`` field
   holding the JSON ``TriggerEventFrame``:

   .. code-block:: text

      id: 1
      data: {"timestamp":"2026-03-19T10:30:00.250Z","payload":{"data":{"data":85.5}}}

   The id counts events on this connection from 1, and unlike the fault stream
   this route does not read ``Last-Event-ID`` - so the id is a position within
   one connection, not a replay cursor, and reconnecting restarts it at 1.

   A brief disconnect does not by itself lose events. Each trigger holds a
   queue of up to 100 pending events, filled as conditions fire whether or not
   a client is attached, and drained on the next connection - so a multishot
   trigger that is reconnected to promptly delivers what it buffered.

   The queue is in memory and belongs to the trigger's lifetime, not to the
   connection, so anything that ends or resets the trigger takes the queue with
   it. Known cases: overflow past 100 discards the oldest; a single-shot
   trigger terminates on firing, after which its stream answers ``404``; the
   ``lifetime`` expiring discards the trigger's whole state; deleting the
   trigger, or restarting the gateway, does the same. Restart loses the queue
   even for a ``persistent`` trigger - persistence stores the trigger and its
   last observed value, never its pending events. Treat the buffer as a
   convenience across a reconnect, not as a delivery guarantee; if you need
   one, poll the underlying resource rather than relying on the stream.

   While no event is pending, the stream sends a comment line rather than a
   frame, every 15 seconds:

   .. code-block:: text

      :keepalive

   **TriggerEventFrame fields:**

   - ``timestamp`` (string) - ISO 8601 timestamp of when the event was generated
   - ``payload`` (object) - The observed resource's value at the moment the
     condition fired - the whole value, not the ``path`` sub-document the
     condition was evaluated against

   There is no ``error`` member. A trigger frame exists only because a
   condition fired, so there is no failed-evaluation case to report; a
   resource that cannot be read simply produces no frame. The *cyclic
   subscription* stream is the one that reports a failed sample inline - see
   ``SubscriptionEventFrame`` - and the two are easy to confuse because they
   are otherwise the same shape.

   The stream closes when:

   - The trigger's ``lifetime`` expires
   - The trigger is deleted
   - A one-shot trigger fires (``multishot: false``)
   - The client disconnects
   - The gateway shuts down
   - Maximum SSE client limit is reached (503 on connect)

   **Example:**

   .. code-block:: bash

      curl -N http://localhost:8080/api/v1/apps/temp_sensor/triggers/trig_001/events

   - **404** ``resource-not-found`` - Trigger not found or expired
   - **503** ``service-unavailable`` - Maximum SSE client limit reached

Trigger Conditions
~~~~~~~~~~~~~~~~~~

The ``trigger_condition`` object in the creation request specifies when the
trigger fires. Four standard condition types are supported:

.. list-table::
   :header-rows: 1
   :widths: 18 32 50

   * - Condition Type
     - Parameters
     - Behavior
   * - ``OnChange``
     - (none)
     - Fires whenever the current value differs from the previous value.
       First evaluation always fires.
   * - ``OnChangeTo``
     - ``target_value`` (any JSON value, required)
     - Fires when the current value equals the target AND differs from the
       previous value. First evaluation checks target only.
   * - ``EnterRange``
     - ``lower_bound`` (number, required), ``upper_bound`` (number, required)
     - Fires when a numeric value transitions from outside the inclusive range
       [lower_bound, upper_bound] to inside it. Requires a previous value
       (first evaluation does not fire).
   * - ``LeaveRange``
     - ``lower_bound`` (number, required), ``upper_bound`` (number, required)
     - Fires when a numeric value transitions from inside the inclusive range
       [lower_bound, upper_bound] to outside it. Requires a previous value
       (first evaluation does not fire).

Plugins can register custom condition evaluators with ``x-`` prefixed names
(e.g., ``x-threshold-count``) via the ``ConditionRegistry``.

Configuration
~~~~~~~~~~~~~

Configure triggers in ``gateway_params.yaml``:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       triggers:
         # Enable/disable the trigger subsystem (default: true)
         # When false, trigger endpoints return 501
         enabled: true

         # Maximum concurrent triggers across all entities (default: 1000)
         # Returns HTTP 503 when this limit is reached
         max_triggers: 1000

         # Behavior on gateway restart for persistent triggers
         # "reset": Clear all triggers on restart (default)
         # "restore": Reload persistent triggers from storage
         on_restart_behavior: "reset"

         # Trigger persistence storage
         storage:
           # Path to SQLite database for persistent triggers
           # Empty string = in-memory only (default)
           # Example: "/var/lib/ros2_medkit/triggers.db"
           path: ""

Persistence
~~~~~~~~~~~

Triggers created with ``"persistent": true`` are stored in a SQLite database.
On gateway restart, their behavior depends on the ``on_restart_behavior``
configuration:

- **reset** (default): All triggers are cleared on restart, regardless of
  the ``persistent`` flag. This is the safest option for development.
- **restore**: Persistent triggers are reloaded from the database. Their
  ``previous_value`` state is preserved, allowing range-based conditions
  (EnterRange, LeaveRange) to evaluate correctly without losing context.

Non-persistent triggers are always cleared on restart.

Fault Triggers (threshold rules)
--------------------------------

Fault triggers are runtime threshold rules on an app's discovered data points:
the engine polls each rule's value and reports a fault while the value stays
crossed (level-triggered), then auto-clears it when the value recovers. They
are a sibling of the SOVD notification ``/triggers`` collection above and never
overload it; rules persist to a JSON store and survive a restart.

The routes are part of the generated OpenAPI spec (``/api/v1/docs``, tag
``FaultTriggers``), so generated clients and Swagger UI discover them the same
way as every other endpoint.

The engine runs only when ``fault_triggers.enabled`` is true *and* at least one
plugin is loaded. Without it the routes stay mounted and answer ``501``
(``not-implemented``) - the same shape the ``/updates`` and ``/triggers`` gates
use, so a client can tell "this build has no threshold engine" apart from "no
such app or rule".

``GET /api/v1/apps/{app_id}/fault-triggers``
   List the app's rules. The owning app is the one in the path; it is not
   repeated in the item, and neither is the engine's internal cross latch.

   .. code-block:: json

      {
        "items": [
          {
            "id": "ftr_1",
            "data_name": "level",
            "operator": ">=",
            "threshold": 80.0,
            "fault_code": "TANK_OVERFILL",
            "severity": "CRITICAL",
            "active": true
          }
        ]
      }

``POST /api/v1/apps/{app_id}/fault-triggers``
   Create a rule. Required: ``data_name``, ``operator`` (``>``, ``<``, ``>=``,
   ``<=``, ``==``), ``threshold`` (number), ``fault_code``, ``severity``
   (``INFO``/``WARNING``/``ERROR``/``CRITICAL``). Optional: ``active``
   (default ``true``). Returns ``201`` with the created rule and a ``Location``
   header pointing to it.

   Validation: ``400`` (``invalid-parameter``) for missing/invalid fields or a
   ``data_name`` the app does not expose (when enumerable); ``404``
   (``entity-not-found``) when the app itself was never discovered; ``409``
   (``precondition-not-fulfilled``) when the ``fault_code`` is already used by
   another rule - fault codes are global to the fault store, so two rules
   sharing one would fight over the same fault.

``DELETE /api/v1/apps/{app_id}/fault-triggers/{trigger_id}``
   Remove a rule (``204``). A fault currently asserted by the rule is cleared;
   the correlation cascade is skipped so the clear stays scoped to the rule's
   own fault.

.. _rate-limiting:

Rate Limiting
-------------

The gateway supports token-bucket-based rate limiting to protect endpoints from abuse. Rate limiting is disabled by default and can be enabled via configuration parameters.

Configuration
~~~~~~~~~~~~~

You can configure global and per-client RPM (requests per minute) limits:

- ``rate_limiting.enabled``: ``true`` to enable.
- ``rate_limiting.global_requests_per_minute``: Overarching limit across all clients.
- ``rate_limiting.client_requests_per_minute``: Limit per individual client IP.

Endpoint limits can also be overridden with patterns:

- ``rate_limiting.endpoint_limits``: List of ``"pattern:rpm"`` strings. For example, ``["/api/v1/*/operations/*:10"]`` limits execution calls without affecting other data endpoints.

Response Headers
~~~~~~~~~~~~~~~~

When rate limiting is enabled, the gateway includes the following HTTP response headers on every check:

- ``X-RateLimit-Limit``: The effective RPM limit applied.
- ``X-RateLimit-Remaining``: Number of requests remaining in the current minute window.
- ``X-RateLimit-Reset``: Unix epoch time (in seconds) when the limit bucket resets.

Rejection (429 Too Many Requests)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If a request exceeds the available tokens, it is rejected with an HTTP 429 status code and a ``Retry-After`` header indicating the number of seconds to wait before retrying.

**Example Response:**

.. code-block:: json

   {
     "error_code": "rate-limit-exceeded",
     "message": "Too many requests. Please retry after 10 seconds.",
     "parameters": {
       "retry_after": 10,
       "limit": 60,
       "reset": 1739612355
     }
   }

.. _rest-authentication:

Authentication Endpoints
------------------------

JWT-based authentication with Role-Based Access Control (RBAC).

The ``/auth/*`` endpoints, and the authentication middleware guarding every
other route, answer errors in the RFC 6749 section 5.2 shape rather than the
SOVD ``GenericError`` used everywhere else:

.. code-block:: json

   {
     "error": "invalid_grant",
     "error_description": "Refresh token is expired or unknown"
   }

``/auth/authorize`` and ``/auth/token`` accept the request body as either
``application/json`` or ``application/x-www-form-urlencoded``, the encoding
RFC 6749 clients default to. ``/auth/revoke`` accepts JSON only, and per
RFC 7009 section 2.2 answers ``200`` whether or not the submitted token was
valid - so it never returns ``401``.

.. seealso::

   :doc:`/tutorials/authentication` for configuration details.

``POST /api/v1/auth/authorize``
   Authenticate with client credentials.

   **Request:**

   .. code-block:: json

      {
        "grant_type": "client_credentials",
        "client_id": "admin",
        "client_secret": "admin_secret_key"
      }

   **Response:**

   .. code-block:: json

      {
        "access_token": "eyJhbGciOiJIUzI1NiIs...",
        "token_type": "Bearer",
        "expires_in": 3600,
        "refresh_token": "dGhpcyBpcyBhIHJlZnJlc2g...",
        "scope": "admin"
      }

``POST /api/v1/auth/token``
   Refresh access token.

   **Request:**

   .. code-block:: json

      {
        "grant_type": "refresh_token",
        "refresh_token": "dGhpcyBpcyBhIHJlZnJlc2g..."
      }

``POST /api/v1/auth/revoke``
   Revoke a token.

   **Request:**

   .. code-block:: json

      {"token": "dGhpcyBpcyBhIHJlZnJlc2g..."}

Vendor Extension Endpoints (Plugins)
-------------------------------------

Plugin-registered endpoints use the ``x-medkit-`` prefix following the SOVD vendor extension
mechanism. These endpoints are only available when the corresponding plugin is loaded
(see :doc:`/tutorials/linux-introspection`).

.. warning::

   The procfs plugin exposes process command lines (``/proc/{pid}/cmdline``) via HTTP.
   Command lines may contain sensitive data (API keys, passwords passed as arguments).
   Enable authentication when using the procfs plugin in production environments.

.. note::

   Vendor extension endpoints are registered dynamically by plugins. They do not appear in
   the ``GET /`` root endpoint list. Use entity capability responses (``GET /apps/{id}``,
   ``GET /components/{id}``) to discover available extensions via the ``capabilities`` field.

Linux Process Introspection (x-medkit-procfs)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Requires: ``procfs_introspection`` plugin.

``GET /api/v1/apps/{id}/x-medkit-procfs``
   Get process-level information for a single app.

   **Response 200:**

   .. code-block:: json

      {
        "pid": 1234,
        "ppid": 1,
        "state": "S",
        "exe": "/usr/bin/talker",
        "cmdline": "/usr/bin/talker --ros-args __node:=talker __ns:=/demo",
        "rss_bytes": 524288,
        "vm_size_bytes": 2097152,
        "threads": 4,
        "cpu_user_ticks": 1520,
        "cpu_system_ticks": 340,
        "cpu_user_seconds": 15.2,
        "cpu_system_seconds": 3.4,
        "uptime_seconds": 123.45
      }

   - **404:** Process not found (node not running or PID cache miss)
   - **503:** Failed to read process information

``GET /api/v1/components/{id}/x-medkit-procfs``
   Aggregate process info for all apps in the component. Processes are
   deduplicated by PID (multiple nodes in the same process appear once).

   **Response 200:**

   .. code-block:: json

      {
        "processes": [
          {
            "pid": 1234,
            "node_ids": ["talker", "listener"],
            "...": "same fields as app endpoint"
          }
        ]
      }

Systemd Unit Introspection (x-medkit-systemd)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Requires: ``systemd_introspection`` plugin and ``libsystemd``.

``GET /api/v1/apps/{id}/x-medkit-systemd``
   Get systemd unit information for the app's process.

   **Response 200:**

   .. code-block:: json

      {
        "unit": "ros2-talker.service",
        "unit_type": "service",
        "active_state": "active",
        "sub_state": "running",
        "restart_count": 2,
        "watchdog_usec": 5000000
      }

   ``restart_count`` and ``watchdog_usec`` are only meaningful for service units.
   For other unit types (timer, mount, etc.) they are always 0.

   - **404:** Process not found or not managed by a systemd unit
   - **503:** Failed to query systemd properties

``GET /api/v1/components/{id}/x-medkit-systemd``
   Aggregate systemd unit info for all apps in the component. Units are
   deduplicated by unit name.

   **Response 200:**

   .. code-block:: json

      {
        "units": [
          {
            "unit": "ros2-talker.service",
            "node_ids": ["talker", "listener"],
            "...": "same fields as app endpoint"
          }
        ]
      }

Container Introspection (x-medkit-container)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Requires: ``container_introspection`` plugin. Only supports cgroup v2
(Ubuntu 22.04+, Fedora 31+).

``GET /api/v1/apps/{id}/x-medkit-container``
   Get container information for the app's process.

   **Response 200:**

   .. code-block:: json

      {
        "container_id": "a1b2c3d4e5f6...",
        "runtime": "docker",
        "memory_limit_bytes": 1073741824,
        "cpu_quota_us": 100000,
        "cpu_period_us": 100000
      }

   Fields ``memory_limit_bytes``, ``cpu_quota_us``, and ``cpu_period_us`` are only present
   when the container has resource limits configured.

   - **404:** Process not found or not running in a container
   - **503:** Failed to read cgroup information

``GET /api/v1/components/{id}/x-medkit-container``
   Aggregate container info for all apps in the component. Containers are
   deduplicated by container ID.

   **Response 200:**

   .. code-block:: json

      {
        "containers": [
          {
            "container_id": "a1b2c3d4e5f6...",
            "node_ids": ["talker", "listener"],
            "...": "same fields as app endpoint"
          }
        ]
      }

x-medkit-topic-beacon
~~~~~~~~~~~~~~~~~~~~~

Provided by the ``ros2_medkit_topic_beacon`` plugin (not available when the
plugin is not loaded). Returns beacon metadata for an entity populated from
``MedkitDiscoveryHint`` messages published by the entity's node via a ROS 2
topic (push-based).

``GET /api/v1/apps/{id}/x-medkit-topic-beacon``

``GET /api/v1/components/{id}/x-medkit-topic-beacon``

**Example:**

.. code-block:: bash

   curl http://localhost:8080/api/v1/apps/engine_temp_sensor/x-medkit-topic-beacon

**Response (200 OK):**

.. code-block:: json

   {
     "entity_id": "engine_temp_sensor",
     "status": "active",
     "age_sec": 1.234,
     "stable_id": "",
     "display_name": "Engine Temperature Sensor",
     "transport_type": "shared_memory",
     "negotiated_format": "",
     "process_id": 12345,
     "process_name": "sensor_node",
     "hostname": "robot-1",
     "component_id": "powertrain",
     "function_ids": ["monitoring"],
     "depends_on": [],
     "metadata": {"custom_key": "custom_value"}
   }

**Status values:**

- ``active`` - Hint is within the configured ``beacon_ttl_sec``
- ``stale`` - Hint is past TTL but within ``beacon_expiry_sec``

**Notes:**

- ``age_sec`` is the elapsed time in seconds since the last hint was received.
- When no beacon data exists for an entity, the endpoint returns 404 with
  error code ``x-medkit-beacon-not-found`` (not an ``"unknown"`` status).

**Response Codes:**

- **200 OK** - Beacon data found and returned
- **404 Not Found** (code: ``ERR_ENTITY_NOT_FOUND``) - Entity does not exist
- **404 Not Found** (code: ``x-medkit-beacon-not-found``) - Entity exists but no beacon data received

x-medkit-param-beacon
~~~~~~~~~~~~~~~~~~~~~

Provided by the ``ros2_medkit_param_beacon`` plugin (not available when the
plugin is not loaded). Returns beacon metadata for an entity populated by
polling ROS 2 node parameters matching a configured prefix (pull-based).

``GET /api/v1/apps/{id}/x-medkit-param-beacon``

``GET /api/v1/components/{id}/x-medkit-param-beacon``

**Example:**

.. code-block:: bash

   curl http://localhost:8080/api/v1/apps/engine_temp_sensor/x-medkit-param-beacon

**Response (200 OK):**

The response schema is identical to ``x-medkit-topic-beacon``. See above for
the full field listing.

**Response Codes:**

- **200 OK** - Beacon data found and returned
- **404 Not Found** (code: ``ERR_ENTITY_NOT_FOUND``) - Entity does not exist
- **404 Not Found** (code: ``x-medkit-beacon-not-found``) - Entity exists but no beacon data received

Error Responses
---------------

Every error carries the SOVD ``GenericError`` body - a flat object, not a
nested ``error`` envelope:

.. code-block:: json

   {
     "error_code": "entity-not-found",
     "message": "Entity not found",
     "parameters": {
       "entity_id": "unknown_component"
     }
   }

``error_code`` and ``message`` are always present. ``parameters`` is
cause-specific and omitted when there is nothing to add.

A vendor-specific failure carries a **fourth** key. The gateway rewrites
``error_code`` to the sentinel ``vendor-error`` and moves the real
``x-medkit-*`` code into ``vendor_code``, so a generic SOVD client sees a code
it knows while the precise one stays available:

.. code-block:: json

   {
     "error_code": "vendor-error",
     "vendor_code": "x-medkit-gateway-shutdown",
     "message": "Gateway is shutting down"
   }

Match on ``error_code``, and on ``vendor_code`` when ``error_code`` is
``vendor-error``. Do not match on ``message`` - it is prose and changes.

The ``/auth/*`` endpoints are the one exception: they answer RFC 6749
section 5.2 ``{"error": "...", "error_description": "..."}`` instead, as does
the authentication middleware on the 401 and 403 it returns ahead of any
route. See :ref:`rest-authentication`.

Common Error Codes
~~~~~~~~~~~~~~~~~~

These are the values that appear in ``error_code`` on the wire.

.. list-table::
   :header-rows: 1
   :widths: 30 15 55

   * - Error Code
     - HTTP Status
     - Description
   * - ``entity-not-found``
     - 404
     - The requested entity does not exist
   * - ``resource-not-found``
     - 404
     - The requested resource (topic, service, parameter) does not exist
   * - ``operation-not-found``
     - 404
     - The named operation does not exist on this entity
   * - ``invalid-request``
     - 400
     - Malformed request body (not valid JSON, or not an object). The same code
       also appears below with a 409, on a lock acquire collision - read the
       status, not the code alone, to tell the two apart.
   * - ``invalid-parameter``
     - 400
     - A field or query parameter failed validation. ``parameters.parameter``
       names the offending one.
   * - ``collection-not-supported``
     - 400
     - This entity type does not serve the requested resource collection
   * - ``precondition-not-fulfilled``
     - 409
     - The request conflicts with current state (e.g. a duplicate
       ``fault_code`` on a fault-trigger rule)
   * - ``lock-broken``
     - 409
     - A guarded write was refused because another client holds a lock on the
       entity. ``parameters.lock_id`` names it.
   * - ``invalid-request``
     - 409
     - The request conflicts with the current state of the resource. This is
       **not** lock-specific - operations use it to refuse re-executing a
       running operation, for instance - so do not assume lock semantics or a
       lock-shaped ``parameters``. ``message`` identifies the conflict and
       ``parameters`` varies with it. For the lock cases and what each one
       carries, see :ref:`the lock refusal table <rest-lock-refusals>`. The
       same code also appears with 400 for a malformed body - see the row
       above.
   * - ``insufficient-access-rights``
     - 403
     - A lifecycle provider refused the transition (``AccessDenied``). Not a
       lock error - the lock routes never emit this code.
   * - ``forbidden``
     - 403
     - The caller is not the owner of the lock it tried to release or extend
       (``LockManager``'s ``lock-not-owner``).
   * - ``payload-too-large``
     - 413
     - Upload exceeds the configured size limit
   * - ``not-implemented``
     - 501
     - The feature is not enabled, or no backend is configured for it
   * - ``service-unavailable``
     - 503
     - A backing service (fault store, subscription manager) refused
   * - ``rate-limit-exceeded``
     - 429
     - Client exceeded its request quota
   * - ``internal-error``
     - 500
     - Unhandled failure. ``parameters.details`` carries the cause.
   * - ``vendor-error``
     - varies
     - A vendor-specific failure; read ``vendor_code`` for the real code
   * - ``x-medkit-plugin-error``
     - 400-599
     - Plugin provider returned an error. Status varies by plugin. Message truncated to 512 chars.
   * - ``x-medkit-gateway-shutdown``
     - 503
     - Gateway is in the process of shutting down. **Do not retry against the same
       gateway instance** - the process is going away. Clients should fail over to
       another gateway or surface the outage to the operator.
   * - ``x-medkit-subscribe-failed``
     - 500
     - Could not create the underlying ROS 2 subscription (rcl error during slot
       creation). Transient: retry once after a short backoff. Persistent failure
       usually indicates a publisher type mismatch or a missing IDL package.
   * - ``x-medkit-resource-sample-failed``
     - n/a
     - A cyclic subscription's sampler could not read the resource on this
       tick. Delivered inside the SSE frame's ``error`` object, never as an
       HTTP status; the stream stays open and the next tick is retried.
   * - ``x-medkit-cold-wait-cap-exceeded``
     - 503
     - Too many concurrent /data callers are waiting on cold (publisher-but-no-data)
       topics. Retry with exponential backoff. ``params.cold_wait_cap`` carries the
       configured cap. Tune via ``data_provider.cold_wait_cap`` and
       ``data_provider.max_parallel_samples`` if this fires under normal load.
   * - ``x-medkit-ros2-topic-unavailable``
     - 404
     - The data resource names a topic the ROS 2 graph does not currently have
   * - ``x-medkit-ros2-service-unavailable``
     - 500
     - A ROS 2 service call backing an operation failed
   * - ``x-medkit-ros2-action-rejected``
     - 400
     - A ROS 2 action server rejected the goal
   * - ``x-medkit-ros2-action-unavailable``
     - 500
     - A ROS 2 action execution failed
   * - ``x-medkit-ros2-parameter-read-only``
     - 403
     - The configuration parameter is declared read-only on the node
   * - ``x-medkit-update-not-found``
     - 404
     - No update package with that id
   * - ``x-medkit-update-already-exists``
     - 400
     - An update package with that id is already registered
   * - ``x-medkit-update-in-progress``
     - 409
     - Another update is executing, or this one is being deleted
   * - ``x-medkit-update-not-prepared``
     - 400
     - ``execute`` was called before ``prepare`` completed
   * - ``x-medkit-update-not-automated``
     - 400
     - ``automated`` was requested on a package whose ``automated`` is false
   * - ``x-medkit-script-already-exists``
     - 409
     - A script with that id already exists
   * - ``x-medkit-managed-script``
     - 409
     - The script is manifest-managed and cannot be modified over REST
   * - ``x-medkit-script-running``
     - 409
     - The script has a running execution and cannot be deleted
   * - ``x-medkit-script-not-running``
     - 409
     - A control action was sent to an execution that is not running
   * - ``x-medkit-concurrency-limit``
     - 429
     - The script backend's concurrent-execution limit was reached
   * - ``x-medkit-script-too-large``
     - 413
     - The uploaded script exceeds the configured size limit
   * - ``x-medkit-ros2-node-unavailable``
     - 503
     - The node backing a configuration read or write did not answer in time
   * - ``x-medkit-invalid-resource-uri``
     - 400
     - A trigger or subscription ``resource`` URI does not parse
   * - ``x-medkit-entity-mismatch``
     - 400
     - A trigger or subscription ``resource`` URI names a different entity than
       the route it was posted to
   * - ``x-medkit-collection-not-supported``
     - 400
     - The ``resource`` URI names a collection triggers and subscriptions
       cannot observe
   * - ``x-medkit-collection-not-available``
     - 400
     - The ``resource`` URI names a collection this entity does not serve
   * - ``x-medkit-unsupported-protocol``
     - 400
     - The requested subscription ``protocol`` has no registered transport

The table is kept complete by a check rather than by review:
``scripts/check_error_codes_documented.py`` (ctest
``gateway_error_codes_documented``) fails if any ``ERR_*`` declared in
``error_codes.hpp`` and named anywhere in the gateway's sources, its headers,
or an in-tree plugin is missing from **this table** - a mention elsewhere in
this guide does not count. So the claim it backs is exactly: every error code
this repository can put on the wire appears above.

What that check does not reach, and this sentence therefore does not claim: a
third-party plugin may raise codes declared nowhere in this repository, and the
statuses and descriptions in the third column are read from the emitters by
hand. One code is excluded by name in the script, with its reason -
``x-medkit-internal-forwarded``, a framework sentinel the error writer returns
on before rendering anything.

Several codes are reached by more than one internal cause. Locking is where
that matters most, because its outcomes are spread across five rows above;
this is every refusal the lock routes can produce:

.. _rest-lock-refusals:

.. list-table:: Lock refusals, by internal cause
   :header-rows: 1
   :widths: 26 12 24 38

   * - Internal cause
     - Status
     - ``error_code``
     - Notes
   * - ``lock-conflict``
     - 409
     - ``invalid-request``
     - Entity already locked. Carries ``existing_lock_id``; retryable with
       ``break_lock``
   * - ``lock-not-breakable``
     - 409
     - ``invalid-request``
     - What a ``break_lock`` retry returns when the held lock forbids it. Also
       carries ``existing_lock_id``
   * - ``lock-not-owner``
     - 403
     - ``forbidden``
     - Releasing or extending a lock held by another client
   * - ``lock-not-found``
     - 404
     - ``resource-not-found``
     - No lock on the entity, or it has expired
   * - ``invalid-expiration``
     - 400
     - ``invalid-parameter``
     - Expiration or extension exceeding the configured maximum. The
       non-positive branch never gets this far - the handler rejects it first
       with its own ``invalid-parameter``
   * - ``lock-required``
     - 409
     - ``invalid-request``
     - The entity's manifest requires a lock for this collection and the caller
       holds none. Carries ``details``, ``entity_id`` and ``collection``, and -
       unlike the two rows above - **no** ``existing_lock_id``, because no lock
       exists to name
   * - (guarded write)
     - 409
     - ``lock-broken``
     - A write refused because another client's lock covers the collection.
       Always carries ``entity_id`` and ``collection``; carries ``lock_id`` only
       when the blocking lock could be identified. Comes from the request
       handler rather than from ``LockManager``

Three refusals the manager can construct are shadowed by an earlier handler
check and so do not reach a client in that form: ``lock-disabled`` (the lock
routes answer ``501`` ``not-implemented`` before ``LockManager`` is consulted),
unknown scope, and non-positive expiration (``LockHandlers`` validates both
against the same vocabulary first and emits its own ``400``
``invalid-parameter``, with ``parameters.invalid_scope`` naming the offending
scope).

On ``PUT`` / ``DELETE .../locks/{lock_id}`` the ``404`` a client normally meets
is the handler's - it checks the lock exists and belongs to the entity before
delegating, and its body carries ``lock_id`` and ``entity_id``. The manager's
own parameter-free ``404`` is still reachable in the narrow window where the
lock expires between those two lookups.

.. _rest-range-rejection:

Range Rejection (416)
~~~~~~~~~~~~~~~~~~~~~

Every operation in the OpenAPI document declares **416 Range Not Satisfiable**,
including operations that have nothing to do with file downloads. This is not
over-declaration. The HTTP layer parses the ``Range`` header before routing the
request, so a syntactically invalid ``Range`` is rejected before any handler
runs - on any path, including paths that do not exist:

.. code-block:: bash

   $ curl -i -H 'Range: furlongs=1-2' http://localhost:8080/api/v1/health
   HTTP/1.1 416 Range Not Satisfiable

The body is the usual ``GenericError`` shape, which is why the document
declares it as such rather than as a body-less response: the HTTP layer itself
writes 416 with an empty body, and the gateway's global error handler then
fills any body-less error response with a ``GenericError``.

Only the six bulk-data download routes declare a ``Range`` *request* parameter,
because they are the only routes where sending one is useful. 416 is
nevertheless reachable everywhere.

416 is not the only status answered this way, and the ``error_code`` in the
body it produces is a placeholder - see the next section.

.. _rest-framework-error-bodies:

Framework-Produced Error Bodies
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Some errors are answered by the HTTP layer itself, before any gateway handler
runs and sometimes before routing. cpp-httplib produces these with an **empty
body**, and the gateway's global error handler then fills any body-less error
response with a ``GenericError`` so that clients always receive the same
envelope. Statuses reaching a client this way include:

- **400** - malformed request line or headers, or an unparseable
  ``multipart/form-data`` boundary
- **413** - a form-urlencoded payload over the built-in length cap
- **414** - request URI too long
- **416** - unparseable ``Range`` header (see :ref:`rest-range-rejection`)
- **500** - an exception escaping a handler

**The ``error_code`` on these bodies is a placeholder.** The global handler
writes ``resource-not-found`` regardless of the actual status, because it runs
after the fact and has no way to know why the HTTP layer rejected the request.
So a 413 and a 416 both arrive carrying ``"error_code":
"resource-not-found"``. (The 404 an unrouted request produces goes through the
same path, where that code happens to be right - which is why the mismatch is
easy to miss on the statuses above.)

**Read the HTTP status, not the ``error_code``, whenever the status was not
produced by a handler.** The codes listed under `Common Error Codes`_ are
accurate only for errors the gateway itself raises. The ``parameters.status``
field on these bodies repeats the real status, which is the reliable field.

Plugin Entity Delegation
~~~~~~~~~~~~~~~~~~~~~~~~

Entities created by gateway plugins (via ``IntrospectionProvider``) have their
data, operations, and faults requests transparently routed to the owning plugin's
``DataProvider``, ``OperationProvider``, or ``FaultProvider``. The response format
is determined by the plugin. If the plugin returns an error, the response uses the
``x-medkit-plugin-error`` vendor code with an ``entity_id`` parameter identifying
the affected entity. See :doc:`/tutorials/plugin-system` for details on per-entity
provider routing.

URL Encoding
------------

Topic and parameter paths containing ``/`` must be URL-encoded:

.. list-table::
   :header-rows: 1
   :widths: 50 50

   * - Original Path
     - URL Encoded
   * - ``/powertrain/engine/temperature``
     - ``powertrain%2Fengine%2Ftemperature``
   * - ``/chassis/brakes/command``
     - ``chassis%2Fbrakes%2Fcommand``

.. _sovd-compliance:

SOVD Compliance
---------------

The gateway implements a **pragmatic subset** of the SOVD (Service-Oriented Vehicle
Diagnostics) standard. We follow SOVD where it matters for interoperability -
endpoint contracts, data model, entity hierarchy - but extend it where ROS 2
use cases benefit.

**SOVD-Aligned Capabilities:**

- Discovery (``/areas``, ``/components``, ``/apps``, ``/functions``)
- Data access (``/data``) with topic sampling and JSON serialization
- Operations (``/operations``, ``/executions``) with async action support
- Configurations (``/configurations``)
- Faults (``/faults``) with ``environment_data`` and SOVD status object
- Logs (``/logs``) with severity filtering and per-entity configuration
- Bulk Data (``/bulk-data``) with custom categories and rosbag downloads
- Software Updates (``/updates``) with async prepare/execute lifecycle
- Cyclic Subscriptions (``/cyclic-subscriptions``) with SSE-based delivery
- Scripts (``/scripts``) with upload, execution, and lifecycle management
- Triggers (``/triggers``) with condition-based push notifications

**Pragmatic Extensions:**

The SOVD spec defines resource collections only for apps and components. ros2_medkit
extends this to areas and functions where aggregation makes practical sense.

The matrix below transcribes ``EntityCapabilities::for_type``. That drives the
paths in an entity's ``/docs`` sub-document, and the collection check in
``validate_collection_access_typed``. It is **not** where the ``capabilities``
array of ``GET /{entity-type}/{id}`` comes from: that array is built from a
second, independent list, the ``CapabilityBuilder::Capability`` vector each
handler in ``discovery_handlers.cpp`` assembles. The two surfaces overlap but
are not the same set - the component array also carries ``status``,
``subcomponents``, ``hosts`` and ``depends-on``, and the area array
``subareas``, ``contains`` and ``components``, none of which are resource
collections and so none of which appear in the table.

The transcription is by hand. What is checked mechanically is the property the
table exists to describe:
``test_openapi_contract::test_every_advertised_collection_is_served`` takes the
first discovered entity of each type, follows every non-templated ``href`` in
its ``capabilities`` array **and** every path in its ``/docs`` sub-document
against a live gateway, and fails on a 404 - so it covers both surfaces,
including where they disagree. Its fixture discovers no areas, so the Areas
column below is covered by the ``EntityCapabilities`` unit tests instead, which
assert the per-type lists directly. ``501`` is a served answer, not a missing
one: see ``data-categories`` and ``data-groups`` below.

Collections named by the SOVD standard that the gateway does **not** serve
per entity - ``data-lists``, ``modes`` and ``communication-logs`` - are absent
from the table and from every capability list. ``updates`` is server-scoped
only (``/api/v1/updates``), never mounted under an entity.

.. list-table:: Resource Collection Support Matrix
   :header-rows: 1
   :widths: 20 16 16 16 16 16

   * - Resource
     - Areas
     - Components
     - Apps
     - Functions
     - SOVD Spec
   * - data
     - aggregated
     - yes
     - yes
     - aggregated
     - apps, components
   * - data-categories
     - 501
     - 501
     - 501
     - 501
     - apps, components
   * - data-groups
     - 501
     - 501
     - 501
     - 501
     - apps, components
   * - operations
     - aggregated
     - yes
     - yes
     - aggregated
     - apps, components
   * - configurations
     - aggregated
     - yes
     - yes
     - aggregated
     - apps, components
   * - faults
     - aggregated
     - yes
     - yes
     - aggregated
     - apps, components
   * - logs
     - prefix match
     - prefix match
     - exact match
     - from hosts
     - apps, components
   * - bulk-data
     - read-only
     - full CRUD
     - full CRUD
     - read-only
     - apps, components
   * - cyclic-subscriptions
     - \-
     - yes
     - yes
     - yes
     - apps, components
   * - scripts
     - \-
     - yes
     - yes
     - \-
     - apps, components
   * - locks
     - \-
     - yes
     - yes
     - \-
     - apps, components
   * - triggers
     - yes (x-medkit)
     - yes
     - yes
     - yes (x-medkit)
     - apps, components
   * - fault-triggers
     - \-
     - \-
     - yes (x-medkit)
     - \-
     - not in SOVD

Three rows depend on configuration, and the two advertising surfaces answer
differently, which is worth stating rather than leaving to be discovered:

- ``locks``: the routes are always registered for components and apps and answer
  ``501`` when there is no lock manager (``locking.enabled`` off). The
  ``capabilities`` entry and the ``locks`` URI field follow the lock manager; the
  ``/docs`` sub-document lists ``/locks`` unconditionally, because
  ``for_type(COMPONENT)`` does.
- ``scripts``: the same shape. ``ScriptManager`` is constructed unconditionally,
  so all eight script routes are always registered for components and apps, and
  they answer ``501`` until a backend exists - either a plugin
  ``ScriptProvider`` or a non-empty ``scripts.scripts_dir``. The
  ``capabilities`` entry follows the backend; the sub-document lists
  ``/scripts`` unconditionally.
- ``fault-triggers``: always registered and always advertised, for apps only;
  with no fault-trigger engine running the routes answer ``501``.

Other extensions beyond SOVD:

- Vendor extension fields using ``x-medkit`` prefix (per SOVD extension mechanism)
- ``DELETE /faults`` - Clear all faults globally
- ``GET /faults/stream`` - SSE real-time fault notifications. Each event payload carries an
  optional ``x-medkit`` SOVD payload-extension object with ``entity_type`` and ``entity_id``
  fields when the gateway can resolve the fault's first reporting source back to an entity,
  so consumers can hit ``/{entity_type}/{entity_id}/bulk-data/rosbags/{fault_code}`` directly
  without enumerating entities. Resolution is snapshotted at event arrival; the entire
  ``x-medkit`` object is omitted when no entity can be resolved.
- ``/health`` - Health check with discovery pipeline diagnostics
- ``/version-info`` - Gateway version information
- ``/docs`` - OpenAPI capability description
- SSE fault streaming - Real-time fault notifications
- ``x-medkit`` extension fields in responses

**Cross-Gateway Resource Aggregation:**

When aggregation is enabled, per-entity resource collection endpoints perform
real-time fan-out to peer gateways. The affected endpoints are: data,
operations, faults, configurations, logs, and the global ``GET /api/v1/faults``
endpoint. The gateway sends the same request to all healthy peers, merges their
``items`` arrays into the local response, and returns the combined result.

If some peer requests fail during fan-out (peer unreachable or non-2xx
response), the response includes vendor metadata indicating partial results:

.. code-block:: json

   {
     "items": [],
     "x-medkit": {
       "partial": true,
       "failed_peers": ["secondary_gateway"]
     }
   }

When all peers respond successfully, these fields are omitted. See the
:doc:`aggregation configuration guide </config/aggregation>` for setup details.

``GET /{entity}/configurations`` applies the same honesty to its *local*
backing nodes. This only arises for entities backed by more than one ROS 2
node; a single-node entity has no partial state, since its one node either
answers or the whole request fails. When several backing nodes contribute and
some answer while others do not, the response stays ``200`` but flags itself
``partial`` and names the failed nodes: nodes that were down or unresponsive
(``503``-class) in ``unavailable_nodes``, any other per-node failure in
``failed_nodes``. The parameters of the nodes that *did* answer are still
returned - a partial list never discards them:

.. code-block:: json

   {
     "items": [
       {"id": "use_sim_time", "name": "use_sim_time", "type": "parameter"}
     ],
     "x-medkit": {
       "partial": true,
       "unavailable_nodes": ["/down_node"]
     }
   }

This matches ``GET /{entity}/configurations/{param}``, which returns ``503``
``x-medkit-ros2-node-unavailable`` for the same backing-node outage.

When every backing node responds, ``unavailable_nodes`` and ``failed_nodes`` are
omitted. Note that ``partial`` may still appear on its own from the peer
fan-out described above, without any local unavailable node.

When *no* backing node answers, the route fails rather than returning an empty
partial list: it surfaces the highest-severity per-node failure, so a
node-unavailability outage returns ``503`` while a purely internal failure -
for example every backing node reporting shutdown - returns ``500``. This route
could not return ``500`` before; clients that previously saw ``503`` for a
manager shutdown now see ``500``.

Capability Description (OpenAPI Docs)
--------------------------------------

The gateway provides self-describing OpenAPI 3.1.0 capability descriptions at any level
of the API hierarchy. Append ``/docs`` to any valid path to receive a context-scoped
OpenAPI spec describing the available operations at that level.

``GET /api/v1/docs``
   Returns the full OpenAPI spec for the gateway root, including all server-level
   endpoints, entity collections, and global resources.

``GET /api/v1/{entity-collection}/docs``
   Returns a spec scoped to the entity collection (e.g., ``/apps/docs``,
   ``/components/docs``). Includes collection listing and detail endpoints.

``GET /api/v1/{entity-type}/{entity-id}/docs``
   Returns a spec for a specific entity, including all resource collection
   endpoints supported by that entity (data, operations, configurations, faults,
   logs, bulk-data, cyclic-subscriptions, triggers).

``GET /api/v1/{entity-type}/{entity-id}/{resource}/docs``
   Returns a spec for a specific resource collection, with detailed schemas
   for each resource item.

**Features:**

- Specs include SOVD extensions (``x-sovd-version``, ``x-sovd-data-category``)
- Each operation declares exactly one success status, derived from the handler's
  C++ return type. The few operations whose handler can genuinely answer with one
  of several success shapes (``POST .../operations/{operation_id}/executions``,
  ``DELETE .../faults/{fault_code}``,
  ``DELETE .../configurations``) carry ``x-medkit-alternates: true`` and list every
  alternative under its own status code. A generated client can therefore branch on
  status only where that marker is present.
- Entity-level specs reflect actual capabilities from the runtime entity cache
- Specs are cached per entity cache generation for performance
- Plugin-registered vendor routes appear in path-scoped specs when the requested
  path matches a plugin route prefix (not in the root spec)

**Configuration:**

- ``docs.enabled`` (bool, default: ``true``) - Set to ``false`` to disable
  the ``/docs`` endpoints. Returns 501 when disabled.

**Swagger UI (optional):**

When built with ``-DENABLE_SWAGGER_UI=ON``, the gateway serves an interactive
Swagger UI at ``/api/v1/swagger-ui`` with embedded assets (no CDN dependency).

**Error Responses:**

- **404:** No capability description available for the requested path
- **501:** Capability description is disabled (``docs.enabled=false``)

See Also
~~~~~~~~

- :doc:`/config/discovery-options` for merge pipeline configuration
- :doc:`/tutorials/authentication` - Configure authentication
- :doc:`/config/server` - Server configuration options
