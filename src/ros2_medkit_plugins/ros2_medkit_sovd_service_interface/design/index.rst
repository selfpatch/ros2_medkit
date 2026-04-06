ros2_medkit_sovd_service_interface
====================================

Overview
--------

The ``ros2_medkit_sovd_service_interface`` package implements a gateway plugin that
exposes the medkit entity tree and fault data via standard ROS 2 services. This enables
other ROS 2 nodes to query gateway entities and faults without going through the HTTP API.

The plugin creates three ROS 2 services:

- ``~/list_entities`` - lists entities by type (areas, components, apps, functions)
- ``~/get_entity_data`` - returns data items for a specific entity
- ``~/get_capabilities`` - returns the capability set for a specific entity

Architecture
------------

The plugin runs as a gateway MODULE (loaded via dlopen) and uses ``PluginContext`` to
access the entity cache and fault manager. It does not implement any provider interfaces
(DataProvider, OperationProvider, etc.) - instead it reads data from the gateway's
existing managers and exposes it via ROS 2 service interfaces.

.. code-block:: text

   Gateway Process
   +------------------------------------------------------+
   | GatewayNode                                          |
   |   +-- PluginManager                                  |
   |   |     +-- SovdServiceInterface (MODULE .so)        |
   |   |           |-- ~/list_entities service             |
   |   |           |-- ~/get_entity_data service           |
   |   |           |-- ~/get_capabilities service          |
   |   |           +-- PluginContext (read-only access)     |
   |   +-- EntityCache <---+                               |
   |   +-- FaultManager <--+                               |
   +------------------------------------------------------+

Key Components
--------------

**SovdServiceInterface** (``src/sovd_service_interface.cpp``)
  Main plugin class inheriting ``GatewayPlugin``. Creates ROS 2 services during
  ``configure()`` and destroys them during ``shutdown()``. Service callbacks query
  the ``PluginContext`` entity cache for data.

**Service Exports** (``src/service_exports.cpp``)
  Standard plugin C API: ``plugin_api_version()``, ``create_plugin()``,
  ``destroy_plugin()``.

Message Types
-------------

The plugin uses custom message types from ``ros2_medkit_msgs``:

- ``EntityInfo.msg`` - entity representation with ID, type, name, capabilities
- ``ListEntities.srv`` - request/response for entity listing with type filter
- ``GetEntityData.srv`` - request/response for entity data retrieval
- ``GetCapabilities.srv`` - request/response for entity capability queries

Design Decisions
----------------

ROS 2 Services over Topics
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Services were chosen over topics because entity queries are inherently request-response.
Topics would require a pub/sub pattern that adds complexity without benefit for
point-in-time queries.

No Provider Interfaces
~~~~~~~~~~~~~~~~~~~~~~

This plugin reads from the gateway's entity cache rather than implementing DataProvider
or OperationProvider. It is a consumer of gateway data, not a provider of new data sources.
This keeps it simple and avoids circular dependencies.

PluginContext Read-Only Access
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The plugin only uses read-only ``PluginContext`` methods (``list_entities()``,
``get_entity()``, ``get_entity_data()``). It never mutates gateway state.
