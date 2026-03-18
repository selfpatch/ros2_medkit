Server Configuration
====================

This reference describes all server-related configuration options for the
ros2_medkit gateway.

Quick Start
-----------

The gateway can be configured via:

1. **Command line**: ``--ros-args -p server.port:=9000``
2. **Launch files**: ``parameters=[{'server.port': 9000}]``
3. **YAML file**: See ``src/ros2_medkit_gateway/config/gateway_params.yaml``

Network Settings
----------------

.. list-table::
   :header-rows: 1
   :widths: 25 15 15 45

   * - Parameter
     - Type
     - Default
     - Description
   * - ``server.host``
     - string
     - ``"127.0.0.1"``
     - Host to bind the REST server. Use ``"0.0.0.0"`` for Docker or network access.
   * - ``server.port``
     - int
     - ``8080``
     - Port for REST API. Valid range: 1024-65535.

Example:

.. code-block:: bash

   # Expose on all interfaces (Docker/network)
   ros2 run ros2_medkit_gateway gateway_node --ros-args \
       -p server.host:=0.0.0.0 \
       -p server.port:=8080

TLS/HTTPS Configuration
-----------------------

.. list-table::
   :header-rows: 1
   :widths: 25 15 15 45

   * - Parameter
     - Type
     - Default
     - Description
   * - ``server.tls.enabled``
     - bool
     - ``false``
     - Enable HTTPS using OpenSSL.
   * - ``server.tls.cert_file``
     - string
     - ``""``
     - Path to PEM-encoded certificate file.
   * - ``server.tls.key_file``
     - string
     - ``""``
     - Path to PEM-encoded private key file. Restrict permissions (chmod 600).
   * - ``server.tls.ca_file``
     - string
     - ``""``
     - Path to CA certificate file (reserved for mutual TLS).
   * - ``server.tls.min_version``
     - string
     - ``"1.2"``
     - Minimum TLS version: ``"1.2"`` (compatible) or ``"1.3"`` (secure).

Example:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       server:
         tls:
           enabled: true
           cert_file: "/etc/ros2_medkit/certs/cert.pem"
           key_file: "/etc/ros2_medkit/certs/key.pem"
           min_version: "1.2"

See :doc:`/tutorials/https` for a complete HTTPS setup tutorial.

CORS Configuration
------------------

Cross-Origin Resource Sharing (CORS) settings for browser-based clients.

.. list-table::
   :header-rows: 1
   :widths: 30 15 20 35

   * - Parameter
     - Type
     - Default
     - Description
   * - ``cors.allowed_origins``
     - list
     - ``[""]``
     - List of allowed origins. Use ``["*"]`` for all (not recommended).
   * - ``cors.allowed_methods``
     - list
     - ``["GET", "PUT", "OPTIONS"]``
     - Allowed HTTP methods.
   * - ``cors.allowed_headers``
     - list
     - ``["Content-Type", "Accept"]``
     - Allowed headers in requests.
   * - ``cors.allow_credentials``
     - bool
     - ``false``
     - Allow credentials (cookies, auth headers).
   * - ``cors.max_age_seconds``
     - int
     - ``86400``
     - Preflight response cache duration (24 hours).

Example for development with sovd_web_ui:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       cors:
         allowed_origins: ["http://localhost:5173"]
         allowed_methods: ["GET", "PUT", "POST", "DELETE", "OPTIONS"]
         allowed_headers: ["Content-Type", "Accept", "Authorization"]
         allow_credentials: true

Data Access Settings
--------------------

.. list-table::
   :header-rows: 1
   :widths: 35 10 10 45

   * - Parameter
     - Type
     - Default
     - Description
   * - ``max_parallel_topic_samples``
     - int
     - ``10``
     - Max concurrent topic samples. Higher values use more resources. Range: 1-50.
   * - ``topic_sample_timeout_sec``
     - float
     - ``1.0``
     - Timeout for sampling topics with active publishers. Range: 0.1-30.0.

Logging Configuration
---------------------

Configure the in-memory log buffer that collects ``/rosout`` messages.

.. list-table::
   :header-rows: 1
   :widths: 25 10 15 50

   * - Parameter
     - Type
     - Default
     - Description
   * - ``logs.buffer_size``
     - int
     - ``200``
     - Maximum log entries retained per node. Valid range: 1-100000
       (values outside this range are clamped with a warning).

Example:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       logs:
         buffer_size: 500

A ``LogProvider`` plugin can replace the default ``/rosout`` backend.
See :doc:`/tutorials/plugin-system` for details.

Performance Tuning
------------------

.. list-table::
   :header-rows: 1
   :widths: 30 10 15 45

   * - Parameter
     - Type
     - Default
     - Description
   * - ``refresh_interval_ms``
     - int
     - ``10000``
     - Cache refresh interval. How often to discover ROS 2 nodes. Range: 100-60000 (0.1s-60s).

Lower values provide faster updates but increase CPU usage.

Bulk Data Storage
-----------------

Configure file-based bulk data storage for uploads (calibration files, firmware, etc.).
The ``rosbags`` category is always available via the Fault Manager and does not need
configuration.

.. list-table::
   :header-rows: 1
   :widths: 30 10 25 35

   * - Parameter
     - Type
     - Default
     - Description
   * - ``bulk_data.storage_dir``
     - string
     - ``/tmp/ros2_medkit_bulk_data``
     - Base directory for uploaded files. Each entity gets a subdirectory.
   * - ``bulk_data.max_upload_size``
     - int
     - ``104857600``
     - Maximum upload file size in bytes (default: 100 MB). Set to 0 for unlimited.
   * - ``bulk_data.categories``
     - string[]
     - ``[]``
     - Allowed bulk data categories for upload/download (e.g., ``calibration``, ``firmware``).

Example:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       bulk_data:
         storage_dir: "/var/ros2_medkit/bulk_data"
         max_upload_size: 52428800   # 50 MB
         categories: ["calibration", "firmware", "logs"]

.. note::

   The ``rosbags`` category is always available through the Fault Manager and cannot be
   used for uploads or deletes. It is automatically included in the category list.

.. note::

   The gateway uses native rclcpp APIs for all ROS 2 interactions - no ROS 2 CLI
   dependencies. Topic discovery, sampling, publishing, service calls, and
   action operations are implemented in pure C++ using ros2_medkit_serialization.

SSE (Server-Sent Events)
------------------------

Configure limits for SSE-based streaming (fault events and cyclic subscriptions).

.. list-table::
   :header-rows: 1
   :widths: 30 10 25 35

   * - Parameter
     - Type
     - Default
     - Description
   * - ``sse.max_clients``
     - int
     - ``10``
     - Maximum number of concurrent SSE connections (fault stream + cyclic subscription streams combined).
   * - ``sse.max_subscriptions``
     - int
     - ``100``
     - Maximum number of active cyclic subscriptions across all entities. Returns HTTP 503 when this limit is reached.
   * - ``sse.max_duration_sec``
     - int
     - ``3600``
     - Maximum allowed subscription duration in seconds. Requests exceeding this are rejected with HTTP 400.

Example:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       sse:
         max_clients: 10
         max_subscriptions: 100
         max_duration_sec: 3600

Rate Limiting
-------------

Token-bucket-based rate limiting for API requests. Disabled by default.

.. list-table::
   :header-rows: 1
   :widths: 35 10 15 40

   * - Parameter
     - Type
     - Default
     - Description
   * - ``rate_limiting.enabled``
     - bool
     - ``false``
     - Enable rate limiting.
   * - ``rate_limiting.global_requests_per_minute``
     - int
     - ``600``
     - Maximum RPM across all clients combined.
   * - ``rate_limiting.client_requests_per_minute``
     - int
     - ``60``
     - Maximum RPM per client IP.
   * - ``rate_limiting.endpoint_limits``
     - string[]
     - ``[]``
     - Per-endpoint overrides as ``"pattern:rpm"`` strings.
       Pattern uses ``*`` as single-segment wildcard
       (e.g., ``"/api/v1/*/operations/*:10"``).
   * - ``rate_limiting.client_cleanup_interval_seconds``
     - int
     - ``300``
     - How often to scan and remove idle client tracking entries (seconds).
   * - ``rate_limiting.client_max_idle_seconds``
     - int
     - ``600``
     - Remove client entries idle longer than this (seconds).

Example:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       rate_limiting:
         enabled: true
         global_requests_per_minute: 600
         client_requests_per_minute: 60
         endpoint_limits: ["/api/v1/*/operations/*:10"]

See :doc:`/api/rest` for rate limiting response headers and 429 behavior.

Plugin Framework
----------------

Extend the gateway with custom plugins loaded from shared libraries (``.so``).
Plugins can implement provider interfaces (e.g., ``UpdateProvider``, ``IntrospectionProvider``)
that are automatically detected and wired into the gateway's subsystem managers.

.. list-table::
   :header-rows: 1
   :widths: 25 15 15 45

   * - Parameter
     - Type
     - Default
     - Description
   * - ``plugins``
     - string[]
     - ``[]``
     - List of plugin names to load. Each plugin requires a corresponding ``plugins.<name>.path`` parameter.
   * - ``plugins.<name>.path``
     - string
     - (required)
     - Absolute path to the plugin ``.so`` file. Must exist and have ``.so`` extension.

Plugin loading lifecycle:

1. Shared library is loaded via ``dlopen`` with ``RTLD_NOW | RTLD_LOCAL``
2. API version is checked (must match gateway headers)
3. ``create_plugin()`` factory is called to instantiate the plugin
4. Provider interfaces are queried via ``extern "C"`` functions
5. ``configure()`` is called with per-plugin config
6. ``set_context()`` passes the gateway context to the plugin
7. ``register_routes()`` allows the plugin to add custom REST endpoints

Error isolation: if a plugin throws during any lifecycle call, it is disabled
without crashing the gateway. Other plugins continue to operate normally.

Example:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       plugins: ["my_ota_plugin"]
       plugins.my_ota_plugin.path: "/opt/ros2_medkit/lib/libmy_ota_plugin.so"

Software Updates
----------------

Configure the software updates system. Updates are disabled by default.
When enabled, a plugin implementing ``UpdateProvider`` is required to provide
the backend functionality (see `Plugin Framework`_ above).

.. list-table::
   :header-rows: 1
   :widths: 25 15 15 45

   * - Parameter
     - Type
     - Default
     - Description
   * - ``updates.enabled``
     - bool
     - ``false``
     - Enable/disable software updates endpoints. When disabled, ``/updates`` routes are not registered.

Example:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       plugins: ["my_update_plugin"]
       plugins.my_update_plugin.path: "/opt/ros2_medkit/lib/libmy_update_plugin.so"
       updates:
         enabled: true

Complete Example
----------------

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       server:
         host: "0.0.0.0"
         port: 8080
         tls:
           enabled: false

       refresh_interval_ms: 5000
       max_parallel_topic_samples: 30
       topic_sample_timeout_sec: 3.0

       cors:
         allowed_origins: ["http://localhost:5173", "https://dashboard.example.com"]
         allowed_methods: ["GET", "PUT", "POST", "DELETE", "OPTIONS"]
         allowed_headers: ["Content-Type", "Accept", "Authorization"]
         allow_credentials: true
         max_age_seconds: 86400

       bulk_data:
         storage_dir: "/var/ros2_medkit/bulk_data"
         max_upload_size: 104857600
         categories: ["calibration", "firmware"]

       sse:
         max_clients: 10
         max_subscriptions: 100
         max_duration_sec: 3600

       logs:
         buffer_size: 200

       plugins: ["my_ota_plugin"]
       plugins.my_ota_plugin.path: "/opt/ros2_medkit/lib/libmy_ota_plugin.so"

       updates:
         enabled: true

       rate_limiting:
         enabled: false

API Documentation
-----------------

Configure the self-describing OpenAPI capability description endpoint.

.. list-table::
   :header-rows: 1
   :widths: 25 15 15 45

   * - Parameter
     - Type
     - Default
     - Description
   * - ``docs.enabled``
     - bool
     - ``true``
     - Enable/disable ``/docs`` capability description endpoints. When disabled,
       all ``/docs`` endpoints return HTTP 501.

**Build option:** ``ENABLE_SWAGGER_UI``

Set ``-DENABLE_SWAGGER_UI=ON`` during CMake configure to embed Swagger UI assets
in the gateway binary. This provides an interactive API browser at
``/api/v1/swagger-ui``. Requires network access to download assets from unpkg.com
during configure. Disabled by default.

Example:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       docs:
         enabled: true

Locking
-------

SOVD resource locking (ISO 17978-3, Section 7.17). Clients acquire locks on
components and apps to prevent concurrent modification.

.. list-table::
   :header-rows: 1
   :widths: 40 15 15 30

   * - Parameter
     - Type
     - Default
     - Description
   * - ``locking.enabled``
     - bool
     - ``true``
     - Enable the LockManager and lock endpoints
   * - ``locking.default_max_expiration``
     - int
     - ``3600``
     - Maximum lock TTL in seconds
   * - ``locking.cleanup_interval``
     - int
     - ``30``
     - Seconds between expired lock cleanup sweeps
   * - ``locking.defaults.components.lock_required_scopes``
     - [string]
     - ``[""]``
     - Collections requiring a lock on components (empty = no requirement)
   * - ``locking.defaults.components.breakable``
     - bool
     - ``true``
     - Whether component locks can be broken
   * - ``locking.defaults.apps.lock_required_scopes``
     - [string]
     - ``[""]``
     - Collections requiring a lock on apps (empty = no requirement)
   * - ``locking.defaults.apps.breakable``
     - bool
     - ``true``
     - Whether app locks can be broken

Per-entity overrides are configured in the manifest ``lock:`` section.
See :doc:`manifest-schema` and :doc:`/api/locking`.

See Also
--------

- :doc:`/tutorials/authentication` - JWT authentication setup
- :doc:`/tutorials/https` - HTTPS configuration
- :doc:`discovery-options` - Discovery and entity mapping options
