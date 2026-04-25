^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_opcua
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Native OPC-UA Part 9 ``AlarmConditionType`` event subscription. The plugin now subscribes to vendor-defined alarms (Siemens S7-1500 ``Program_Alarm`` / ProDiag, Beckhoff TF6100, CodeSys 3.5+, Rockwell via FactoryTalk Linx) and bridges each event into the SOVD fault lifecycle. Configured via a new top-level ``event_alarms:`` block in the node map YAML; mutually exclusive per entry with the existing threshold-based ``alarm`` form. (issue #386)
* New SOVD operations on entities that host alarm sources: ``acknowledge_fault`` invokes the inherited ``Acknowledge`` method on the live ``ConditionId`` (i=9111, EventId tracked per Part 9 §5.7.3); ``confirm_fault`` invokes ``Confirm`` (i=9113). Both accept an optional ``comment`` rendered as ``LocalizedText`` on the server.
* ``OpcuaClient`` gains ``add_event_monitored_item`` / ``remove_event_monitored_item`` / ``call_method`` and a generation counter that filters callbacks fired from defunct subscriptions after a reconnect. Heap-owned ``EventCallbackContext`` resolves the open62541pp / raw-C lifetime hazard.
* Header-only ``AlarmStateMachine`` mapping ``EnabledState x ShelvingState x ActiveState x AckedState x ConfirmedState x BranchId`` to SOVD ``CONFIRMED / HEALED / CLEARED / Suppressed``. Full transition table documented in ``design/index.rst``.
* ``ConditionRefresh`` (Server method i=3875) is invoked on subscribe and on every reconnect, with ``RefreshStartEvent`` / ``RefreshEndEvent`` bracketing tracked for diagnostics.
* New ``test_alarm_server`` fixture (open62541-based, full namespace 0 + alarms enabled) emits AlarmConditionType events on stdin commands; integration test ``run_alarm_tests.sh`` runs in CI alongside the existing OpenPLC threshold suite.

0.4.0 (2026-04-11)
------------------
* Initial release
* ``OpcuaPlugin`` implementation of ``GatewayPlugin`` and ``IntrospectionProvider`` that bridges OPC-UA capable PLCs into the SOVD entity tree
* REST endpoints via the new ``get_routes()`` plugin API: ``x-plc-data``, ``x-plc-operations``, ``x-plc-status``
* Vendor capabilities registered per entity - only PLC-backed apps and the PLC runtime component advertise the ``x-plc-*`` endpoints
* Full OPC 10000-6 section 5.3.1.10 node identifier support (``i=`` numeric, ``s=`` string, ``g=`` GUID, ``b=`` opaque ByteString); example node maps for OpenPLC, Siemens S7-1500 TIA Portal, Beckhoff TwinCAT 3, Allen-Bradley via Kepware and KUKA KR C5
* ``NodeMap`` driven by YAML configuration - same binary serves any OPC-UA compliant server by changing the node map file
* Deterministic entity ordering in ``IntrospectionResult`` output (entries sorted by id)
* Threshold-based PLC alarm detection routed to SOVD faults via ``ros2_medkit_msgs`` services ``ReportFault`` / ``ClearFault``
* Optional bridging of numeric PLC values to ROS 2 ``std_msgs/Float32`` topics from ``set_context()``
* Type-aware writes with per-node range validation
* Robust connection-loss detection: all three OPC-UA client paths (``read_value``, ``read_values``, ``write_value``) mark the connection as dropped on terminal status codes so ``OpcuaPoller`` reconnect kicks in without stalling
* Polling mode (default) and OPC-UA subscription mode, backed by ``open62541pp`` v0.16.0
* Integration test suite against an OpenPLC IEC 61131-3 tank demo container
* Contributors: @mfaferek93
