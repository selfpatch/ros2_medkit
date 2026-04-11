^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_opcua
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Initial release
* ``OpcuaPlugin`` implementation of ``GatewayPlugin`` and ``IntrospectionProvider`` that bridges OPC-UA capable PLCs into the SOVD entity tree
* REST endpoints via the new ``get_routes()`` plugin API: ``x-plc-data``, ``x-plc-operations``, ``x-plc-status``
* ``NodeMap`` driven by YAML configuration - same binary serves any PLC by changing the node map file
* Threshold-based PLC alarm detection routed to SOVD faults via ``ros2_medkit_msgs`` services ``ReportFault`` / ``ClearFault``
* Optional bridging of numeric PLC values to ROS 2 ``std_msgs/Float32`` topics from ``set_context()``
* Type-aware writes with per-node range validation
* Polling mode (default) and OPC-UA subscription mode, backed by ``open62541pp`` v0.16.0
* Integration test suite against an OpenPLC IEC 61131-3 tank demo container
* Contributors: @mfaferek
