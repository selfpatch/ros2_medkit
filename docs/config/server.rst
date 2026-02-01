Server Configuration
====================

This reference describes all server-related configuration options for the
ros2_medkit gateway.

Quick Start
-----------

The gateway can be configured via:

1. **Command line**: ``--ros-args -p server.port:=9000``
2. **Launch files**: ``parameters=[{'server.port': 9000}]``
3. **YAML file**: See ``config/gateway_params.yaml``

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

.. note::

   The gateway uses native rclcpp APIs for all ROS 2 interactions—no ROS 2 CLI
   dependencies. Topic discovery, sampling, publishing, service calls, and
   action operations are implemented in pure C++ using ros2_medkit_serialization.

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

See Also
--------

- :doc:`/tutorials/authentication` - JWT authentication setup
- :doc:`/tutorials/https` - HTTPS configuration
- :doc:`discovery-options` - Discovery and entity mapping options
