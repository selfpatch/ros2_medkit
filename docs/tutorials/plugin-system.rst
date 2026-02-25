Plugin System
=============

The gateway supports a plugin system for extending functionality with shared libraries (``.so`` files).
Plugins can provide software update backends, platform-specific introspection, and custom REST endpoints.

Overview
--------

Plugins implement the ``GatewayPlugin`` C++ base class plus one or more typed provider interfaces:

- **UpdateProvider** - software update backend (CRUD, prepare/execute, automated, status)
- **IntrospectionProvider** *(preview)* - enriches discovered entities with platform-specific metadata.
  This interface is defined and can be implemented, but is not yet wired into the discovery cycle.

A single plugin can implement multiple provider interfaces. For example, a "systemd" plugin
could provide both introspection (discover systemd units) and updates (manage service restarts).

Configuration
-------------

Add plugins to ``gateway_params.yaml``:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       plugins: ["my_ota_plugin"]
       plugins.my_ota_plugin.path: "/opt/ros2_medkit/lib/libmy_ota_plugin.so"

       # Enable updates if your plugin implements UpdateProvider
       updates:
         enabled: true

Each plugin name in the ``plugins`` array requires a corresponding ``plugins.<name>.path``
parameter with the absolute path to the ``.so`` file. Plugins are loaded in the order listed.
An empty list (default) means no plugins are loaded, with zero overhead.

Plugin names must contain only alphanumeric characters, underscores, and hyphens (max 256 chars).

Writing a Plugin
----------------

1. Create a C++ shared library implementing ``GatewayPlugin`` and optionally one or more providers:

.. code-block:: cpp

   #include "ros2_medkit_gateway/plugins/gateway_plugin.hpp"
   #include "ros2_medkit_gateway/plugins/plugin_types.hpp"
   #include "ros2_medkit_gateway/providers/update_provider.hpp"

   using namespace ros2_medkit_gateway;

   class MyPlugin : public GatewayPlugin, public UpdateProvider {
    public:
     std::string name() const override { return "my_plugin"; }

     void configure(const nlohmann::json& config) override {
       // Read plugin-specific configuration (env vars, files, etc.)
     }

     void shutdown() override {
       // Clean up resources
     }

     // UpdateProvider methods - all 7 must be implemented:

     tl::expected<std::vector<std::string>, UpdateBackendErrorInfo>
       list_updates(const UpdateFilter& filter) override { /* ... */ }

     tl::expected<nlohmann::json, UpdateBackendErrorInfo>
       get_update(const std::string& id) override { /* ... */ }

     tl::expected<void, UpdateBackendErrorInfo>
       register_update(const nlohmann::json& metadata) override { /* ... */ }

     tl::expected<void, UpdateBackendErrorInfo>
       delete_update(const std::string& id) override { /* ... */ }

     tl::expected<void, UpdateBackendErrorInfo>
       prepare(const std::string& id, UpdateProgressReporter& reporter) override { /* ... */ }

     tl::expected<void, UpdateBackendErrorInfo>
       execute(const std::string& id, UpdateProgressReporter& reporter) override { /* ... */ }

     tl::expected<bool, UpdateBackendErrorInfo>
       supports_automated(const std::string& id) override { /* ... */ }
   };

2. Export the required ``extern "C"`` symbols:

.. code-block:: cpp

   // Required: API version check
   extern "C" GATEWAY_PLUGIN_EXPORT int plugin_api_version() {
     return ros2_medkit_gateway::PLUGIN_API_VERSION;
   }

   // Required: factory function
   extern "C" GATEWAY_PLUGIN_EXPORT GatewayPlugin* create_plugin() {
     return new MyPlugin();
   }

   // Required if your plugin implements UpdateProvider:
   extern "C" GATEWAY_PLUGIN_EXPORT UpdateProvider* get_update_provider(GatewayPlugin* p) {
     return static_cast<MyPlugin*>(p);
   }

   // Required if your plugin implements IntrospectionProvider:
   extern "C" GATEWAY_PLUGIN_EXPORT IntrospectionProvider* get_introspection_provider(GatewayPlugin* p) {
     return static_cast<MyPlugin*>(p);
   }

The ``get_update_provider`` and ``get_introspection_provider`` functions use ``extern "C"``
to avoid RTTI issues across shared library boundaries. The ``static_cast`` is safe because
these functions execute inside the plugin's own ``.so`` where the type hierarchy is known.

Without the corresponding ``get_*_provider`` export, the gateway cannot detect that your plugin
implements the provider interface, even if the class inherits from it.

3. Build as a MODULE library:

.. code-block:: cmake

   add_library(my_plugin MODULE src/my_plugin.cpp)
   target_link_libraries(my_plugin gateway_lib)

4. Install the ``.so`` and add its path to ``gateway_params.yaml``.

Complete Minimal Plugin
-----------------------

A self-contained plugin implementing UpdateProvider (copy-paste starting point):

.. code-block:: cpp

   // my_ota_plugin.cpp
   #include "ros2_medkit_gateway/plugins/gateway_plugin.hpp"
   #include "ros2_medkit_gateway/plugins/plugin_types.hpp"
   #include "ros2_medkit_gateway/providers/update_provider.hpp"

   #include <httplib.h>
   #include <nlohmann/json.hpp>

   using namespace ros2_medkit_gateway;

   class MyOtaPlugin : public GatewayPlugin, public UpdateProvider {
    public:
     std::string name() const override { return "my_ota"; }

     void configure(const nlohmann::json& /*config*/) override {}

     void shutdown() override {}

     // UpdateProvider CRUD
     tl::expected<std::vector<std::string>, UpdateBackendErrorInfo>
     list_updates(const UpdateFilter& /*filter*/) override {
       return std::vector<std::string>{};
     }

     tl::expected<nlohmann::json, UpdateBackendErrorInfo>
     get_update(const std::string& id) override {
       return tl::make_unexpected(
         UpdateBackendErrorInfo{UpdateBackendError::NotFound, "not found: " + id});
     }

     tl::expected<void, UpdateBackendErrorInfo>
     register_update(const nlohmann::json& /*metadata*/) override { return {}; }

     tl::expected<void, UpdateBackendErrorInfo>
     delete_update(const std::string& /*id*/) override { return {}; }

     // UpdateProvider async operations
     tl::expected<void, UpdateBackendErrorInfo>
     prepare(const std::string& /*id*/, UpdateProgressReporter& reporter) override {
       reporter.set_progress(100);
       return {};
     }

     tl::expected<void, UpdateBackendErrorInfo>
     execute(const std::string& /*id*/, UpdateProgressReporter& reporter) override {
       reporter.set_progress(100);
       return {};
     }

     tl::expected<bool, UpdateBackendErrorInfo>
     supports_automated(const std::string& /*id*/) override { return false; }
   };

   // Required exports
   extern "C" GATEWAY_PLUGIN_EXPORT int plugin_api_version() {
     return PLUGIN_API_VERSION;
   }

   extern "C" GATEWAY_PLUGIN_EXPORT GatewayPlugin* create_plugin() {
     return new MyOtaPlugin();
   }

   // Required for UpdateProvider detection
   extern "C" GATEWAY_PLUGIN_EXPORT UpdateProvider* get_update_provider(GatewayPlugin* p) {
     return static_cast<MyOtaPlugin*>(p);
   }

Plugin Lifecycle
----------------

1. ``dlopen`` loads the ``.so`` with ``RTLD_NOW | RTLD_LOCAL``
2. ``plugin_api_version()`` is checked against the gateway's ``PLUGIN_API_VERSION``
3. ``create_plugin()`` factory function creates the plugin instance
4. Provider interfaces are queried via ``get_update_provider()`` / ``get_introspection_provider()``
5. ``configure()`` is called with per-plugin JSON config
6. ``set_node()`` provides optional ROS 2 node access
7. ``register_routes()`` allows registering custom REST endpoints
8. Runtime: subsystem managers call provider methods as needed
9. ``shutdown()`` is called before the plugin is destroyed

Custom REST Endpoints
---------------------

Any plugin can register vendor-specific endpoints via ``register_routes()``:

.. code-block:: cpp

   void register_routes(httplib::Server& server, const std::string& api_prefix) override {
     server.Get(api_prefix + "/x-myvendor/status",
       [this](const httplib::Request&, httplib::Response& res) {
         res.set_content(get_status_json(), "application/json");
       });
   }

Use the ``x-`` prefix for vendor-specific endpoints per SOVD convention.

Multiple Plugins
----------------

Multiple plugins can be loaded simultaneously:

- **UpdateProvider**: Only one plugin's UpdateProvider is used (first in config order)
- **IntrospectionProvider**: All plugins' results are merged *(preview - not yet wired)*
- **Custom routes**: All plugins can register endpoints (use unique path prefixes)

Error Handling
--------------

If a plugin throws during any lifecycle method (``configure``, ``set_node``, ``register_routes``,
``shutdown``), the exception is caught and logged. The plugin is disabled but the gateway continues
operating. A failing plugin never crashes the gateway.

API Versioning
--------------

Plugins export ``plugin_api_version()`` which must return the gateway's ``PLUGIN_API_VERSION``.
If the version does not match, the plugin is rejected with a clear error message suggesting
a rebuild against matching gateway headers.

The current API version is **1**. It will be incremented when breaking changes are made to
``GatewayPlugin`` or provider interfaces.

Build Requirements
------------------

- **Same compiler and ABI** as the gateway executable
- **RTTI must be enabled** - do NOT compile plugins with ``-fno-rtti``
- The ``GATEWAY_PLUGIN_EXPORT`` macro ensures correct symbol visibility
