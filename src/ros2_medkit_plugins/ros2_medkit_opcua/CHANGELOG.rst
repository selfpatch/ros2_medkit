^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_opcua
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
