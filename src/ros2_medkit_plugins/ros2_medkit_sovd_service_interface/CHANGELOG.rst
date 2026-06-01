^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_sovd_service_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.0 (2026-06-01)
------------------
* Initial release - gateway plugin that exposes the medkit entity tree and fault data over standard ROS 2 services, so other ROS 2 nodes can query entities and faults without going through the HTTP API (`#330 <https://github.com/selfpatch/ros2_medkit/issues/330>`_)
* Creates ROS 2 services under a configurable prefix (default ``/medkit``): ``list_entities``, ``list_entity_faults``, and ``get_capabilities``; a ``get_entity_data`` service is also registered but currently returns "not implemented" (use the HTTP REST API for topic data) pending (`#351 <https://github.com/selfpatch/ros2_medkit/issues/351>`_)
* Runs as a gateway MODULE plugin with read-only ``PluginContext`` access to the entity cache and fault manager; implements no provider interfaces
* Shutdown guards and ``noexcept`` destructors reset ROS resources before member destruction to prevent teardown crashes
* Contributors: @bburda, @mfaferek93
