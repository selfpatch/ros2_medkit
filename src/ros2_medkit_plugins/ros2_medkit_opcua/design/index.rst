==================
ros2_medkit_opcua
==================

Design notes for the OPC-UA gateway plugin.

Motivation
==========

Mixed ROS 2 + PLC diagnostics
-----------------------------

Industrial robot deployments are rarely pure ROS 2. A typical cell combines ROS 2 manipulators
with PLC-controlled fixtures, conveyors, safety relays, and process equipment. Operators want a
single diagnostic surface that covers both sides.

The OPC-UA plugin lets ``ros2_medkit_gateway`` expose OPC-UA PLC tags as first-class SOVD entities,
giving diagnostic tools and MCP agents one consistent API across the whole cell.

Architecture
============

The plugin is loaded by ``ros2_medkit_gateway`` at runtime via ``dlopen`` and implements two
interfaces:

- ``GatewayPlugin`` - lifecycle (``configure``, ``set_context``, ``get_routes``, ``shutdown``) and
  vendor REST endpoints.
- ``IntrospectionProvider`` - emits an ``IntrospectionResult`` describing the SOVD entities
  generated from the PLC node map, so the gateway's merge pipeline can place them in the entity tree.

::

    OPC-UA server (PLC, OpenPLC, Siemens S7, Beckhoff, ...)
             |  opc.tcp, port 4840
             v
    OpcuaClient (open62541pp wrapper)
             |
             v
    OpcuaPoller  --- threshold alarms ---> OpcuaPlugin::on_alarm_change
             |                                     |
             |  PollSnapshot                       | ros2_medkit_msgs/ReportFault
             v                                     v  (or ClearFault when alarm clears)
    OpcuaPlugin::publish_values          ros2_medkit_fault_manager
             |                                     |
             v                                     v
    std_msgs/Float32 topics            SOVD faults on PLC entities
    (one per numeric node)

Node map schema
===============

All mapping lives in YAML (``config/tank_demo_nodes.yaml`` is the reference example):

- ``area_id`` / ``component_id`` - SOVD tree placement for the PLC runtime
- ``nodes`` - list of entries, each with:

  - ``node_id`` - OPC-UA node identifier (e.g. ``"ns=2;i=1"``)
  - ``entity_id`` - SOVD app the value belongs to
  - ``data_name`` - short name used in REST URLs
  - ``display_name``, ``unit``, ``data_type``, ``writable`` - metadata
  - ``min_value`` / ``max_value`` - optional write range check
  - ``alarm`` - optional fault definition (``fault_code``, ``severity``, ``threshold``,
    ``above_threshold`` direction)

The plugin binary is completely PLC-agnostic; pointing it at a different node map file
reuses the same build for a different PLC.

Poll vs subscription mode
=========================

The plugin supports two data paths, selected by the ``prefer_subscriptions`` config flag:

**Polling (default)** - a background thread reads all mapped nodes at a fixed interval
(``poll_interval_ms``, default 1000 ms). Simple, predictable, no event loop needed, sufficient
for diagnostic use cases where latency below one second does not add value.

**Subscription** - registers OPC-UA monitored items and receives change notifications at
``subscription_interval_ms``. Lower CPU cost on large node sets, but requires a running OPC-UA
client event loop. Kept as an opt-in path because many PLC servers have limits on the number
of active subscriptions.

Both modes share the same ``PollSnapshot`` output structure, so downstream consumers
(``OpcuaPlugin::publish_values``, REST handlers, fault mapping) are agnostic to the source.

Alarm -> fault mapping
======================

Each node in the map may declare an ``alarm`` block. The poller compares the current value
against the configured threshold on every poll; transitions across the threshold produce
edge-triggered callbacks:

- **Active -> fault reported** via ``/fault_manager/report_fault`` with the configured
  severity and message
- **Cleared -> fault cleared** via ``/fault_manager/clear_fault`` by fault code

The plugin keeps per-fault state only long enough to detect edges; the fault manager owns
persistence and fault lifecycle.

Type-aware writes
=================

``POST /apps/{id}/x-plc-operations/{op}`` accepts a JSON body ``{"value": ...}``. The handler:

1. Looks up the node by ``data_name`` (with optional ``set_`` prefix in the op name).
2. Rejects read-only nodes with a 400 error.
3. Coerces the JSON value to the node's declared ``data_type`` (``bool``, ``int``, ``string``,
   otherwise ``double``) and returns a typed 400 on mismatch.
4. Applies the range check (``min_value`` / ``max_value``) if configured.
5. Calls ``OpcuaClient::write_value`` which uses the node's OPC-UA data type to build the
   correct binary payload.

This avoids the common failure mode of writing a ``float64`` to an OPC-UA ``REAL`` (float32)
node and getting silent truncation or a server-side rejection.

ROS 2 topic bridge
==================

``set_context`` creates one ``std_msgs/Float32`` publisher per numeric node, named from the
``ros2_topic`` field of the node map. After every poll, ``publish_values`` pushes the current
values onto these topics. Non-numeric nodes (strings) are skipped with a one-time warning per
node.

This lets ROS 2 consumers (rqt plots, nav2 behavior trees, diagnostic aggregators) consume
PLC state without having to speak OPC-UA directly.

Security
========

The current implementation uses anonymous OPC-UA authentication with ``SecurityPolicy=None``.
This is explicitly **not suitable for untrusted networks** - the plugin logs a warning on
startup. Proper username/password and certificate-based authentication are planned.

Supported PLC servers
=====================

The plugin speaks plain OPC-UA and supports all four node identifier types
defined by OPC 10000-6 section 5.3.1.10 (``i=`` numeric, ``s=`` string,
``g=`` GUID, ``b=`` opaque ByteString). This means the same binary works
against any compliant OPC-UA server without a code change - only the
``node_map.yaml`` changes per vendor. ``config/tank_demo_nodes.yaml`` carries
ready-to-paste examples for OpenPLC (this demo), Siemens S7-1500 TIA Portal,
Beckhoff TwinCAT 3, Allen-Bradley via Kepware, and KUKA KR C5.

What the plugin can NOT do today (regardless of vendor):

- Complex OPC-UA types: only scalar Variables (``int``, ``float``, ``bool``,
  ``string``) are mapped. Structures, arrays, enums, unions and
  ExtensionObjects are ignored.
- Vendor information models: profiles like Euromap 77 (injection molding),
  Siemens DI (device integration), and PA-DIM (process automation) are not
  interpreted natively. Tags exposed through those models can still be
  mapped manually via their node IDs, but the plugin does not understand
  the model semantics.
- Native OPC-UA Alarms & Conditions: the plugin detects alarms by applying
  thresholds to polled values. Servers that publish native AlarmCondition
  events (e.g. Siemens ``AlarmConditionType``) are not subscribed to.

Future work
===========

Tracked issues in the `ros2_medkit issue tracker
<https://github.com/selfpatch/ros2_medkit/issues>`_:

- `#367 <https://github.com/selfpatch/ros2_medkit/issues/367>`_
  Certificate-based OPC-UA authentication (Basic256Sha256)
- `#368 <https://github.com/selfpatch/ros2_medkit/issues/368>`_
  Optional auto-browse of the OPC-UA address space to seed the node map
  (deferred pending validated user demand; UaExpert and ``python -m
  asyncua.tools.uals`` are recommended for now)
- `#366 <https://github.com/selfpatch/ros2_medkit/issues/366>`_
  Vendor ``open62541pp`` inline so the package can be added to the
  rosdistro release list

Untracked (open the issue if you hit the pain):

- Hot-reload of the node map without restarting the plugin
- Complex OPC-UA type support (structures, arrays, enums)
- Native ``AlarmCondition`` event subscription as a complement to
  threshold polling
- Vendor information model bindings (Euromap 77, Siemens DI, PA-DIM)
