Multi-Instance Aggregation
==========================

This document describes the design of multi-instance peer aggregation in
ros2_medkit_gateway. It covers the entity model changes (SOVD alignment),
entity merge logic, request routing, and mDNS-based auto-discovery.

.. contents:: Table of Contents
   :local:
   :depth: 3

Overview
--------

A single ros2_medkit_gateway instance discovers and serves the ROS 2 entities on
its local machine. In production systems, robots often run multiple processes
across several hosts, containers, or network segments. Multi-instance aggregation
allows a **primary gateway** to transparently merge entities from one or more
**peer gateways** into a single unified API. Clients see one entity tree and do
not need to know which gateway owns which entity.

.. plantuml::
   :caption: Multi-Instance Aggregation - High-Level Architecture

   @startuml aggregation_overview

   skinparam linetype ortho
   skinparam packageStyle rectangle

   actor "Client" as client

   package "Primary Gateway (host-A)" {
       class AggregationManager {
           + fetch_all_peer_entities()
           + fan_out_get()
           + forward_request()
           + check_all_health()
       }
       class EntityMerger {
           + merge_areas()
           + merge_functions()
           + merge_components()
           + merge_apps()
           + get_routing_table()
       }
       class MdnsDiscovery {
           + start()
           + stop()
       }
       class EntityCache
   }

   package "Peer Gateway B (host-B)" {
       class "REST API (B)" as restB
   }

   package "Peer Gateway C (host-C)" {
       class "REST API (C)" as restC
   }

   client --> AggregationManager : HTTP request
   AggregationManager --> EntityMerger : merge remote entities
   AggregationManager --> restB : PeerClient
   AggregationManager --> restC : PeerClient
   AggregationManager --> EntityCache : update with merged entities
   MdnsDiscovery ..> AggregationManager : add_discovered_peer()
   EntityMerger ..> EntityCache : merged entity set

   @enduml

Entity Model (SOVD Alignment)
-----------------------------

Prior to this feature, the gateway used only **Areas**, **Components**, and
**Apps** as entity types. The SOVD spec (ISO 17978) defines a richer hierarchy
where **Components** represent physical hardware (ECUs, hosts) and **Functions**
represent logical capabilities. This feature aligns the entity model:

- **Area** - Physical or logical domain (unchanged). Namespace grouping.
- **Component** - Physical host or ECU. In runtime-only mode, the gateway
  creates a single Component from the local hostname using ``HostInfoProvider``.
  All Apps discovered on that host are children of this Component.
- **App** - Individual ROS 2 node (unchanged).
- **Function** - Logical capability grouping. In runtime-only mode, each
  ROS 2 namespace becomes a Function entity, grouping the Apps that share
  that namespace. In manifest mode, Functions are explicitly declared.

This mapping aligns with the SOVD view where a Component is "what hosts the
software" and a Function is "what the software does".

.. plantuml::
   :caption: SOVD Entity Model Alignment

   @startuml entity_model

   skinparam linetype ortho
   skinparam classAttributeIconSize 0

   title SOVD Entity Hierarchy

   class Area {
       id: string
       namespace_path: string
       description: string
   }

   class Component {
       id: string
       name: string
       description: string
       area: string
       source: "runtime" | "manifest"
   }

   class App {
       id: string
       name: string
       namespace_path: string
       component: string
       source: string
   }

   class Function {
       id: string
       name: string
       hosts: vector<string>
       source: "manifest" | "runtime"
   }

   Area "1" *--> "*" Component : contains
   Component "1" *--> "*" App : hosts
   Function "1" o--> "*" App : groups

   note right of Component
       In runtime mode: single host
       Component from HostInfoProvider
       (hostname, OS, arch).
   end note

   note right of Function
       In runtime mode: each namespace
       becomes a Function entity.
       In manifest mode: explicitly declared.
   end note

   @enduml

HostInfoProvider
~~~~~~~~~~~~~~~~

``HostInfoProvider`` reads local system information (hostname, OS from
``/etc/os-release``, CPU architecture from ``uname``) and creates a single
``Component`` entity representing the physical host. The Component ID is the
sanitized hostname (lowercase, dots replaced with underscores, truncated to
256 characters).

This replaces the old "synthetic component per namespace" behavior in
runtime-only discovery mode. The single host Component gives the entity tree a
physically meaningful root, and namespace-based grouping is handled by Function
entities instead.

Resource Collections on Functions and Areas
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Functions and Areas support the same resource collections as Components and Apps:

- **data** - Aggregated topic data from all hosted entities
- **operations** - Aggregated services and actions from hosted entities
- **configurations** - Aggregated parameters from hosted entities
- **faults** - Aggregated faults from hosted entities
- **logs** - Aggregated log entries from hosted entities

Requests to ``/functions/{id}/data`` are fan-out queries that collect data from
all entities listed in the Function's ``hosts`` field. Similarly, Area resource
collection requests aggregate from all Components contained in that Area.

Entity Merge Logic
------------------

When entities arrive from a peer gateway, the ``EntityMerger`` applies
type-specific merge rules:

.. plantuml::
   :caption: Entity Merge Logic by Type

   @startuml entity_merge

   skinparam activity {
       BackgroundColor #f0f0f0
       BorderColor #999999
   }

   title Entity Merge Rules

   |Area / Function|
   start
   :Receive remote entity;
   if (Same ID exists locally?) then (yes)
       :Merge into existing entity;
       note right
           - Area: combine relations
           - Function: union of hosts lists
           - No routing entry (both sides own it)
       end note
   else (no)
       :Add as new entity;
       :Set source = "peer:<name>";
   endif
   stop

   |Component / App|
   start
   :Receive remote entity;
   if (Same ID exists locally?) then (yes)
       :Prefix remote ID with\n"peername__original_id";
       :Add routing entry for prefixed ID;
   else (no)
       :Keep original ID;
       :Add routing entry for original ID;
   endif
   :Set source = "peer:<name>";
   stop

   @enduml

**Merge rules summary:**

- **Areas**: Merge by ID. If both local and remote have the same Area ID (e.g.,
  ``root``), they are combined into one entity. Remote-only Areas are added with
  ``source: "peer:<name>"``. No routing table entry is created for merged Areas
  because the local gateway owns the merged entity.

- **Functions**: Merge by ID, combining the ``hosts`` lists from both sides.
  If both gateways expose a ``navigation`` Function, the merged entity lists
  hosts from both gateways. Same ownership semantics as Areas.

- **Components**: Prefix on collision. If a remote Component has the same ID as
  a local one, the remote entity's ID is prefixed with ``peername__`` (double
  underscore separator). For example, remote Component ``lidar_unit`` from peer
  ``robot_arm`` becomes ``robot_arm__lidar_unit``. A routing table entry maps the
  (possibly prefixed) ID to the peer name for request forwarding.

- **Apps**: Same prefix-on-collision behavior as Components.

The ``EntityMerger::SEPARATOR`` constant (``__``) is used as the prefix
separator. The routing table maps ``entity_id -> peer_name`` for all remote
entities that need request forwarding.

Request Routing
---------------

When a request arrives for an entity, ``HandlerContext`` checks the routing
table. If the entity is local, processing continues normally. If the entity
maps to a peer, the request is forwarded transparently:

.. plantuml::
   :caption: Request Routing - Local vs Remote

   @startuml request_routing

   actor Client
   participant "Primary Gateway" as primary
   participant HandlerContext as ctx
   participant AggregationManager as agg
   participant "Peer Gateway" as peer

   Client -> primary : GET /api/v1/components/robot_arm__lidar/data
   primary -> ctx : validate_entity_for_route()
   ctx -> ctx : Look up "robot_arm__lidar" in routing table
   ctx --> primary : entity found, peer = "robot_arm"

   primary -> agg : forward_request("robot_arm", req, res)
   agg -> peer : GET /api/v1/components/lidar/data
   note right
       Path rewritten: strips peer prefix
       from entity ID before forwarding
   end note
   peer --> agg : 200 OK + JSON data
   agg --> primary : Copy response to client
   primary --> Client : 200 OK + JSON data

   == Local Entity ==

   Client -> primary : GET /api/v1/apps/my_node/data
   primary -> ctx : validate_entity_for_route()
   ctx -> ctx : "my_node" not in routing table
   ctx --> primary : entity found, local

   primary -> primary : Process locally (DataAccessManager)
   primary --> Client : 200 OK + JSON data

   @enduml

**Global collection endpoints** (e.g., ``GET /api/v1/components``,
``GET /api/v1/faults``) use **fan-out**: the primary gateway sends the
same request to all healthy peers, collects the responses, and merges the
``items`` arrays into a single response. If some peers fail, the response
body includes ``x-medkit.partial: true`` and ``x-medkit.failed_peers``.

Peer Discovery
--------------

Peers can be configured statically in the YAML config or discovered
automatically via mDNS.

Static Peers
~~~~~~~~~~~~

Configure peers directly in ``gateway_params.yaml`` using parallel arrays:

.. code-block:: yaml

   aggregation:
     enabled: true
     peer_urls: ["http://192.168.1.10:8080", "http://192.168.1.11:8080"]
     peer_names: ["arm_controller", "base_platform"]

Static peers are always present in the peer list regardless of mDNS settings.

mDNS Auto-Discovery
~~~~~~~~~~~~~~~~~~~~

``MdnsDiscovery`` uses multicast DNS (via the ``mjansson/mdns`` header-only C
library) to announce and discover gateway instances on the local network.

- **Announce**: A background thread responds to mDNS queries for the configured
  service type (default: ``_medkit._tcp.local``). Other gateways on the network
  discover this instance automatically.

- **Browse**: A background thread periodically sends mDNS queries and processes
  responses. When a new peer is found, ``AggregationManager::add_discovered_peer()``
  is called. When a peer sends a goodbye, ``remove_discovered_peer()`` is called.

mDNS discovery works alongside static peers. A gateway can have both static
and dynamically discovered peers.

Health Monitoring
~~~~~~~~~~~~~~~~~

``AggregationManager`` periodically calls ``check_all_health()`` on a
configurable interval (default: 10 seconds). Each ``PeerClient`` GETs
``/api/v1/health`` on its peer. If the health check fails, the peer is
marked unhealthy and excluded from fan-out queries and entity fetching.

When a peer recovers (health check succeeds again), it is automatically
re-included.

Stream Proxy
~~~~~~~~~~~~

For streaming connections (e.g., SSE fault subscriptions), the ``StreamProxy``
interface provides transport-agnostic event proxying. The ``SSEStreamProxy``
implementation connects to a peer's SSE endpoint and relays events back to the
primary gateway's client. Each ``StreamEvent`` carries the ``peer_name`` so the
aggregator can attribute events to their source.

Deployment Topologies
---------------------

Star Topology
~~~~~~~~~~~~~

One primary gateway aggregates from multiple leaf gateways. Best for robots
with a central controller and peripheral subsystems:

.. code-block:: text

   Client
     |
   Primary (host-A)
    / | \
   B  C  D  (leaf gateways)

Each leaf gateway discovers its own ROS 2 subsystem. The primary merges all
entities and serves a unified view.

Chain Topology
~~~~~~~~~~~~~~

Gateways are chained: A aggregates from B, which aggregates from C. Each
level in the chain sees the merged view of everything downstream:

.. code-block:: text

   Client -> A -> B -> C

Gateway A sees entities from A + B + C. Gateway B sees entities from B + C.
Gateway C sees only its own entities.

This is useful for layered systems where subsystems have their own aggregation
level (e.g., a fleet gateway that aggregates per-robot gateways, which in turn
aggregate per-subsystem gateways).

Containers on Same Host
~~~~~~~~~~~~~~~~~~~~~~~

Multiple ROS 2 subsystems run in separate containers on the same host. Each
container runs its own gateway instance. A host-level gateway aggregates from
all containers via ``localhost`` or Docker network:

.. code-block:: text

   Client
     |
   Host Gateway (port 8080)
    /           \
   Container A   Container B
   (port 8081)   (port 8082)

mDNS discovery handles container-to-container communication automatically
when containers share a network. Static peers work for bridge-networked
containers where mDNS does not cross network boundaries.

Key Classes
-----------

``PeerClient``
    HTTP client for communicating with a single peer gateway. Supports health
    checking, entity fetching, transparent request forwarding (proxy), and
    JSON-parsed responses for fan-out merging. Thread-safe via atomic health
    flag and mutex-guarded lazy client creation.

``EntityMerger``
    Stateless merge engine that combines local and remote entity sets using
    type-specific rules (merge by ID for Area/Function, prefix on collision for
    Component/App). Produces a routing table mapping remote entity IDs to peer
    names.

``AggregationManager``
    Central coordinator that manages the set of ``PeerClient`` instances, runs
    health checks, maintains the routing table, and provides fan-out and
    forwarding APIs. Thread-safe via ``shared_mutex``.

``MdnsDiscovery``
    Background service for announcing and browsing mDNS services. Runs
    announce and browse threads. Invokes callbacks when peers are found or
    removed.

``StreamProxy`` / ``SSEStreamProxy``
    Transport-agnostic interface for proxying streaming connections to peers.
    ``SSEStreamProxy`` implements SSE-based event relaying with a background
    reader thread.

``HostInfoProvider``
    Reads local host system info (hostname, OS, architecture) and produces a
    single ``Component`` entity representing the physical host. Used in
    runtime-only discovery to replace synthetic per-namespace Components.
