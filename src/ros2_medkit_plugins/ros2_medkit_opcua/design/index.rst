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
- Vendor information model bindings (Euromap 77, Siemens DI, PA-DIM)


Native ``AlarmConditionType`` event subscription (issue #386)
=============================================================

The plugin subscribes to native OPC-UA Part 9 ``AlarmConditionType`` events
emitted by vendor PLCs, in addition to the threshold-based polling path. Both
modes coexist in a single ``node_map.yaml`` (different YAML keys) and feed the
same ``fault_manager`` service.

Configuration
-------------

Two YAML forms describe alarms; an entry can use one but never both:

.. code-block:: yaml

   nodes:
     # Threshold-based (existing): polls the scalar value and raises a fault
     # when it crosses the configured threshold.
     - node_id: 'ns=2;i=2'
       entity_id: tank_process
       data_name: tank_temperature
       data_type: float
       alarm:
         fault_code: TANK_OVERHEAT
         severity: ERROR
         threshold: 80.0
         above_threshold: true

   event_alarms:
     # Native AlarmConditionType (new): subscribes to events emitted from
     # the source NodeId and bridges them through the state machine below.
     - alarm_source: 'ns=4;s=Alarms.Overpressure'
       entity_id: tank_process
       fault_code: PLC_OVERPRESSURE
       severity_override: ERROR        # optional - else derived from event Severity
       message: 'Tank overpressure'    # optional - else event Message field

State machine
-------------

Inputs from each event payload (positional ``EventFilter`` select clauses):

- ``EnabledState.Id`` - bool
- ``ShelvingState.CurrentState.Id`` - NodeId; non-Unshelved => suppressed
- ``ActiveState.Id`` - bool
- ``AckedState.Id`` - bool
- ``ConfirmedState.Id`` - bool
- ``BranchId`` - NodeId; non-null means historical branch (Part 9 §5.5.2.12)

Decision order, first match wins:

+-----+--------------------------------------+---------------------------------------+
| #   | Condition                            | Outcome                               |
+=====+======================================+=======================================+
| 1   | ``BranchId != null``                 | history-only (no SOVD update)         |
+-----+--------------------------------------+---------------------------------------+
| 2   | ``EnabledState == false``            | clear if was active, else no-op       |
+-----+--------------------------------------+---------------------------------------+
| 3   | ``ShelvingState/CurrentState/Id``    | clear if was active, else no-op.      |
|     | in {``TimedShelved`` (i=2930),       | A null/unset/unknown ``Id`` is        |
|     | ``OneShotShelved`` (i=2932)}         | treated as ``Unshelved`` (some        |
|     |                                      | servers leave the optional field      |
|     |                                      | uninitialized).                       |
+-----+--------------------------------------+---------------------------------------+
| 4   | ``ActiveState == true``              | ``CONFIRMED`` (idempotent)            |
+-----+--------------------------------------+---------------------------------------+
| 5a  | ``ActiveState == false`` and         | internal ``HEALED`` state; ``/faults``|
|     | not (``Acked`` and ``Confirmed``)    | still shows ``CONFIRMED`` (see note   |
|     |                                      | below)                                |
+-----+--------------------------------------+---------------------------------------+
| 5b  | ``ActiveState == false``,            | ``CLEARED``                           |
|     | ``Acked == true``,                   |                                       |
|     | ``Confirmed == true``                |                                       |
+-----+--------------------------------------+---------------------------------------+

``Retain`` is intentionally NOT used for state determination. Per Part 9
§5.5.2.10 it controls visibility during ``ConditionRefresh`` bursts only;
lifecycle is driven entirely by Active / Acked / Confirmed. The SOVD
``PREFAILED`` state has no native equivalent and is reserved for the
threshold-polling pre-trigger path.

.. note::

   **HEALED is internal-only.** Rule 5a transitions the bridge's internal
   ``SovdAlarmStatus`` to ``Healed`` and emits the ``ReportHealed`` action,
   but ``OpcuaPlugin::on_event_alarm`` deliberately treats that action as a
   no-op. The reason: ``ros2_medkit_msgs/srv/ReportFault`` has only
   ``EVENT_FAILED`` / ``EVENT_PASSED`` verbs, and routing ``EVENT_PASSED``
   on every latch would feed the fault_manager debounce engine - which has
   its own statistical ``HEALED`` semantics (``healing_threshold`` cycles
   of PASSED events) and may auto-clear the fault, defeating Part 9's
   mandatory ack/confirm contract. Until ``ros2_medkit_msgs/msg/Fault``
   gains a ``STATUS_LATCHED`` (or equivalent) value, ``/faults`` keeps
   ``status=CONFIRMED`` for a latched alarm; the next ``Cleared`` (rule 5b)
   removes the entry.

   This is a known UX gap: an operator looking at ``/faults`` cannot
   distinguish "alarm physically active" from "alarm physically cleared,
   awaiting confirm". The ack/confirm SOVD operations work end-to-end; the
   visibility limitation is tracked as a follow-up.

Severity mapping
----------------

OPC-UA severity is a 1-1000 scalar. The plugin maps it to selfpatch's SOVD
severity buckets:

- 1-200    -> ``INFO``
- 201-500  -> ``WARNING``
- 501-800  -> ``ERROR``
- 801-1000 -> ``CRITICAL``

This is the selfpatch convention, **not** IEC 62682 - that spec defines a
1-1000 priority scale but no normative band names. ``severity_override`` on
an ``event_alarms`` entry takes precedence when set.

ConditionRefresh
----------------

After creating event monitored items the plugin invokes ``ConditionRefresh``
(Server object ``i=2253``, method ``i=3875``) so the server pushes any
condition that fired before the subscription started. The same call fires
on every successful reconnect.

The bracketing ``RefreshStartEventType`` (i=2787) and ``RefreshEndEventType``
(i=2788) are recognized and used to set a diagnostic flag; live notifications
arriving during the burst are applied normally because the state machine is
driven by per-condition ``ConditionId`` and runs idempotently.

Acknowledge / Confirm round-trip
--------------------------------

Two SOVD operations appear on every entity that has at least one event-mode
alarm declared:

- ``POST /apps/{entity}/operations/acknowledge_fault/executions``
- ``POST /apps/{entity}/operations/confirm_fault/executions``

Body:

.. code-block:: json

   { "fault_code": "PLC_OVERPRESSURE", "comment": "operator on radio" }

The plugin resolves ``(entity_id, fault_code)`` to the live ``ConditionId``
maintained by the poller, then calls the inherited
``AcknowledgeableConditionType`` method (``Acknowledge`` ``i=9111`` or
``Confirm`` ``i=9113``) on that NodeId. The latest ``EventId`` ``ByteString``
captured from the most recent notification is passed as the first argument;
without it servers return ``BadEventIdUnknown`` (Part 9 §5.7.3).

Vendor matrix
-------------

+----------------------+---------------------+-------------------------------+
| Vendor / runtime     | AlarmConditionType  | Notes                         |
+======================+=====================+===============================+
| Siemens S7-1500      | yes (FW V2.9+)      | ProDiag, Program_Alarm,       |
|                      |                     | system diagnostics            |
+----------------------+---------------------+-------------------------------+
| Beckhoff TwinCAT 3   | yes (TF6100)        | ``Confirm`` propagates to     |
|                      |                     | PLC code; ``Ack`` does not    |
+----------------------+---------------------+-------------------------------+
| Rockwell ControlLogix| yes (via FactoryTalk| Tag-based alarms bridged by   |
|                      | Linx FW 16.20+)     | the gateway                   |
+----------------------+---------------------+-------------------------------+
| CodeSys 3.5+         | yes (alarm manager  | Custom severity mapping       |
|                      | provider library)   |                               |
+----------------------+---------------------+-------------------------------+
| OpenPLC v3           | no                  | Scalar variables only;        |
|                      |                     | use threshold mode            |
+----------------------+---------------------+-------------------------------+

Out of scope
------------

- ``ShelvingState`` write operations (``TimedShelve`` / ``OneShotShelve`` /
  ``Unshelve``). The plugin reads the state to suppress active alarms but
  does not yet expose operator UI to set it.
- OPC-UA branch reasoning beyond ``BranchId``-based suppression.
  Re-fires are tracked via ``fault_manager`` ``occurrence_count`` plus the
  ``/faults/stream`` SSE history.
- Auto-discovery of alarm sources via ``Server.GeneratedEvents`` browse
  (tracked in #368 alongside scalar auto-discovery).
- ``Quality`` (StatusCode) propagation to a SOVD ``status_quality`` field.
  Requires an additive field on the ``ReportFault.srv`` schema; tracked
  separately.
