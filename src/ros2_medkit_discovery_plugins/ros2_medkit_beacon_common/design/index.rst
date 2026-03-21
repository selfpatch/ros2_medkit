ros2_medkit_beacon_common
==========================

This section contains design documentation for the ros2_medkit_beacon_common library.

Overview
--------

The ``ros2_medkit_beacon_common`` package is a shared C++ library used by the
beacon discovery plugins (``ros2_medkit_topic_beacon`` and ``ros2_medkit_param_beacon``).
It provides the data model, hint storage, entity mapping, validation, and response
building that both beacon transports rely on. It is not a plugin itself - it is
linked as a static library by the two beacon plugin packages.

Architecture
------------

The following diagram shows the beacon common components and their relationships.

.. plantuml::
   :caption: Beacon Common Library Architecture

   @startuml ros2_medkit_beacon_common_architecture

   skinparam linetype ortho
   skinparam classAttributeIconSize 0

   title Beacon Common - Library Architecture

   package "ros2_medkit_gateway" {
       interface IntrospectionProvider {
           +introspect(input): IntrospectionResult
       }

       struct IntrospectionInput {
           +areas: vector~<Area~>
           +components: vector~<Component~>
           +apps: vector~<App~>
           +functions: vector~<Function~>
       }

       struct IntrospectionResult {
           +metadata: map~<string, json~>
           +new_entities: NewEntities
       }
   }

   package "ros2_medkit_beacon_common" {

       struct BeaconHint {
           + entity_id: string
           + stable_id: string
           + display_name: string
           + function_ids: vector~<string~>
           + depends_on: vector~<string~>
           + component_id: string
           + transport_type: string
           + negotiated_format: string
           + process_id: uint32
           + process_name: string
           + hostname: string
           + metadata: map~<string, string~>
           + received_at: time_point
       }

       class BeaconHintStore {
           - hints_: map~<string, StoredHint~>
           - config_: Config
           - mutex_: shared_mutex
           --
           + update(hint): bool
           + evict_and_snapshot(): vector~<StoredHint~>
           + get(entity_id): optional~<StoredHint~>
           + size(): size_t
       }

       class BeaconEntityMapper {
           - config_: Config
           --
           + map(hints, current): IntrospectionResult
           --
           - build_metadata(hint): json
           - apply_function_membership()
       }

       class BeaconValidator <<utility>> {
           + {static} validate_beacon_hint(hint, limits): ValidationResult
       }

       class BeaconResponseBuilder <<utility>> {
           + {static} build_beacon_response(id, stored): json
       }

       struct "BeaconHintStore::Config" as StoreConfig {
           + beacon_ttl_sec: double = 10.0
           + beacon_expiry_sec: double = 300.0
           + max_hints: size_t = 10000
       }

       struct ValidationLimits {
           + max_id_length: 256
           + max_string_length: 512
           + max_function_ids: 100
           + max_metadata_entries: 50
       }

       enum HintStatus {
           ACTIVE
           STALE
           EXPIRED
       }
   }

   BeaconHintStore o--> BeaconHint : stores
   BeaconHintStore --> HintStatus : computes
   BeaconHintStore --> StoreConfig : configured by
   BeaconEntityMapper --> BeaconHintStore : reads snapshots
   BeaconEntityMapper ..> IntrospectionResult : produces
   BeaconEntityMapper ..> IntrospectionInput : reads
   BeaconValidator --> BeaconHint : validates
   BeaconResponseBuilder --> BeaconHintStore : reads

   @enduml

Main Components
---------------

1. **BeaconHint** - Core data structure representing a discovery hint from a ROS 2 node

   - ``entity_id`` is the only required field (identifies the app)
   - Identity fields: ``stable_id``, ``display_name`` for human-readable labeling
   - Topology fields: ``function_ids``, ``depends_on``, ``component_id`` for entity relationships
   - Transport fields: ``transport_type``, ``negotiated_format`` for DDS introspection
   - Process fields: ``process_id``, ``process_name``, ``hostname`` for runtime context
   - ``metadata`` map for arbitrary key-value pairs
   - ``received_at`` timestamp in steady_clock domain for TTL computation

2. **BeaconHintStore** - Thread-safe TTL-based storage for beacon hints

   - ``update()`` inserts or refreshes a hint; returns false if capacity is full for new entity IDs
   - ``evict_and_snapshot()`` atomically removes expired hints and returns a consistent snapshot
   - Three-state lifecycle: ACTIVE (within TTL), STALE (past TTL but before expiry), EXPIRED (evicted)
   - Configurable: TTL (default 10s), expiry (default 300s), max capacity (default 10,000 hints)
   - Protected by ``std::shared_mutex`` for concurrent reads during ``get()``

3. **BeaconEntityMapper** - Maps stored hints to gateway IntrospectionResult

   - Takes a snapshot from ``BeaconHintStore`` and the current ``IntrospectionInput``
   - Builds per-entity metadata JSON from hint fields
   - Applies function membership: if a hint declares ``function_ids``, the mapper updates
     the corresponding Function entities' host lists
   - Optionally allows new entity creation (``allow_new_entities`` config flag)

4. **BeaconValidator** - Input sanitization for incoming beacon hints

   - Validates ``entity_id`` format (required, max length, allowed characters)
   - Truncates oversized string fields rather than rejecting the entire hint
   - Enforces limits on collection sizes (function_ids, depends_on, metadata entries)
   - Returns ``valid=false`` only when ``entity_id`` itself is invalid

5. **BeaconResponseBuilder** - Builds JSON responses for beacon metadata HTTP endpoints

   - Constructs the response payload served by ``x-medkit-beacon`` vendor extension endpoints
   - Includes hint data plus status (active/stale) and last-seen timestamp

Design Decisions
----------------

Shared Library, Not Plugin
~~~~~~~~~~~~~~~~~~~~~~~~~~

The beacon common code is a static library linked into the topic and parameter
beacon plugins, not a standalone plugin. This avoids an extra shared library load
at runtime while keeping the transport-specific logic (topic subscription vs.
parameter polling) in separate plugin ``.so`` files.

Three-State Hint Lifecycle
~~~~~~~~~~~~~~~~~~~~~~~~~~

Hints transition through ACTIVE, STALE, and EXPIRED states rather than using a
simple present/absent model. The STALE state allows the gateway to report that a
node was recently seen but is no longer actively sending beacons - useful for
distinguishing temporary network hiccups from actual node departures.

Capacity Limits
~~~~~~~~~~~~~~~

The store enforces a hard cap on the number of tracked hints (default 10,000) to
prevent memory exhaustion from misbehaving nodes or DDoS scenarios. When capacity
is reached, new entity IDs are rejected while existing hints can still be refreshed.

Validation Strategy
~~~~~~~~~~~~~~~~~~~

The validator is lenient by design: only an invalid ``entity_id`` causes rejection.
Other fields are sanitized (truncated, pruned) so that partially malformed beacons
still contribute useful discovery information rather than being silently dropped.
