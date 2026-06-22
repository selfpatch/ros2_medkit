Entity Cache Architecture
=========================

Background
----------

The gateway's ``ThreadSafeEntityCache`` holds the live SOVD entity tree
(Areas, Components, Apps, Functions) and is read by every HTTP handler
thread. Prior to issue #442 the cache stored entities in
``std::unordered_map`` containers that reallocated on insert and shifted
slot addresses on erase. Discovery refreshes - which run every 100 ms on
graph-event detection - caused structural allocations even when nothing had
changed, adding heap churn and making the cache unsuitable for
memory-constrained or embedded deployments.

This document describes the replacement architecture introduced in #442.

Goals
-----

1. Eliminate per-refresh structural allocations in the cache layer for a
   stable entity graph.
2. Make heap use predictable: capacity is fixed at startup, not
   open-ended.
3. Gate the generation counter on actual changes so polling it on a quiet
   graph is cheap.
4. Preserve the existing public API and SOVD REST contract - the changes
   are entirely below the handler layer.

Object Pool and Flat Index Architecture
---------------------------------------

The cache uses two building blocks.

**SlotStore<T>** - a stable-slot object pool. Each entity type (Area,
Component, App, Function) gets its own ``SlotStore<T>``. The store holds
entities in a flat ``std::vector`` that is pre-reserved to ``capacity``
slots at construction. Slots are never moved or shifted after allocation;
a freed slot is marked available and reused for the next insert. Slot IDs
are stable for the lifetime of an entity. The store exposes typed
references so callers do not hold raw pointers across mutations.

**FlatHashMap<K, V>** - a linear-probe open-address hash map pre-reserved
to the same capacity at construction. Used for all index maps:

- ``area_index_``, ``component_index_``, ``app_index_``,
  ``function_index_`` - map entity ID to slot ID (O(1) lookup)
- ``component_to_apps_``, ``area_to_components_``, etc. - relationship
  indexes (parent ID to child slot ID list)
- ``operation_index_`` - operation full_path to owning entity reference
- ``topic_type_cache_`` - topic name to message type

All these maps are reserved once at ``ThreadSafeEntityCache`` construction
and do not rehash as long as the live entity count stays within
``capacity``. The ``capacity`` value comes from the
``entity_cache.capacity`` ROS parameter (default: 256, valid range:
16-1000000; values outside this range are clamped with a warning at
startup).

Incremental Reconcile
---------------------

``update_all`` (and the per-type variants ``update_areas``,
``update_components``, ``update_apps``, ``update_functions``) implement an
in-place diff instead of a clear-and-rebuild:

1. **Remove** slots whose IDs are absent from the incoming list.
2. **Add** incoming IDs that are not yet in the index (allocate a new
   slot from the pool).
3. **Change** slots where an existing entity's payload differs from the
   incoming value (in-place overwrite, no slot movement).

Only mutations that fall into categories 1, 2, or 3 advance the
generation counter. A no-op call (incoming set is identical to the
current cache) leaves the generation unchanged. This allows consumers such
as the OpenAPI capability generator to skip expensive regeneration when
the graph has not changed.

Zero-Structural-Allocation Property
------------------------------------

Once ``capacity`` slots are reserved and the entity count stabilises
below that threshold, steady-state ``update_all`` calls perform zero
structural allocations in the cache layer: no ``std::vector`` resize, no
hash map rehash, no ``new``/``delete`` for index entries. In-place slot
overwrites update entity payloads without touching the pool structure.

.. note::

   **Honest boundary.** Entity payloads themselves - ``nlohmann::json``
   objects (e.g., ``host_metadata``, ``type_info``) and
   ``std::vector<std::string>`` fields (topic lists, service lists) -
   still allocate on the heap inside the discovery layer before the cache
   sees them. Making those payloads fixed-capacity (e.g., custom
   allocators or pre-sized arenas) is a separate future effort and is
   explicitly out of scope for this change. The zero-structural-allocation
   guarantee applies to the cache data structures, not to the payloads
   stored in them.

Discovery-Side Allocation Under Graph Churn
-------------------------------------------

The cache is only the last step of ``refresh_cache()``. Per-graph-event
allocation profiling showed the dominant churn under a churning graph was in
the discovery pipeline, not the cache rebuild, driven by two things:

1. **Eager schema building.** ``discover_apps`` rebuilt the JSON request/
   response (and goal/result/feedback) schemas for every service and action in
   the graph on every refresh, then deep-copied them through several
   intermediate containers. These schemas are immutable per type and are not
   read from the discovered entities at all - the ``/operations`` handler
   already resolves them on demand. Discovery now leaves ``type_info`` empty;
   the handler resolves it lazily and ``TypeIntrospection`` caches the
   assembled per-type ``type_info`` as a shared, immutable object
   (``get_service_type_info`` / ``get_action_type_info``), so repeated requests
   reuse it.

2. **Refresh frequency.** The rclcpp graph event fires many times per second
   under churn, running the full discovery pipeline each time. Graph events are
   now debounced (``discovery.refresh_debounce_ms``, default 1 s) so a burst of
   graph changes coalesces into a single refresh.

Overflow Behaviour
------------------

If ``update_all`` receives more entities than the reserved ``capacity``,
the pool and maps grow dynamically (the standard resize path). A one-shot
WARN log fires when this happens:

.. code-block:: text

   [WARN] entity_cache capacity 256 exceeded (grew); raise
   entity_cache.capacity for embedded determinism

The gateway continues operating normally. The WARN fires once per process
lifetime to avoid log spam.

Generation Counter
------------------

``ThreadSafeEntityCache::generation()`` returns a ``uint64_t`` that
increments on each mutation that actually changes the cache contents.
Callers use it as a change token:

.. code-block:: cpp

   auto gen = cache.generation();
   // ... later ...
   if (cache.generation() != gen) {
     rebuild_openapi_spec();
   }

Because the counter is gated on real changes, polling it on a stable
graph costs nothing beyond an atomic load.

Health Endpoint Stats
---------------------

``GET /api/v1/health`` exposes entity cache statistics under the
``x-medkit-entity-cache`` vendor-extension key:

.. code-block:: json

   {
     "status": "healthy",
     "x-medkit-entity-cache": {
       "capacity": 256,
       "areas": 2,
       "components": 1,
       "apps": 14,
       "functions": 3,
       "generation": 7,
       "grew": false
     }
   }

Configuration
-------------

The cache capacity is set via ``entity_cache.capacity`` in
``gateway_params.yaml`` (or any ROS parameter source). See
:doc:`/config/server` (Performance Tuning section) for the full parameter
reference.

See Also
--------

- :doc:`ros2_subscription_architecture` - Analogous embedded-hardened
  design for the topic subscription pool
- :doc:`/config/server` - ``entity_cache.capacity`` parameter reference
