^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_gateway
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Initial rosdistro release
* HTTP REST gateway for ros2_medkit diagnostics system
* SOVD-compatible entity discovery with four entity types:

  * Areas, Components, Apps, Functions
  * HATEOAS links and capabilities in all responses
  * Relationship endpoints (subareas, subcomponents, related-apps, hosts)

* Three discovery modes:

  * Runtime-only: automatic ROS 2 graph introspection
  * Manifest-only: YAML manifest with validation (11 rules)
  * Hybrid: manifest as source of truth + runtime linking

* REST API endpoints:

  * Fault management: GET/POST/DELETE /api/v1/faults
  * Data access: topic sampling via GenericSubscription
  * Operations: service calls and action goals via GenericClient
  * Configuration: parameter get/set via ROS 2 parameter API
  * Snapshots: GET /api/v1/faults/{code}/snapshots
  * Rosbag: GET /api/v1/faults/{code}/snapshots/bag

* Server-Sent Events (SSE) at /api/v1/faults/stream:

  * Multi-client support with thread-safe event queue
  * Keepalive, Last-Event-ID reconnection, configurable max_clients

* JWT-based authentication with configurable policies
* HTTPS/TLS support via OpenSSL and cpp-httplib
* Native C++ ROS 2 serialization via ros2_medkit_serialization (no CLI dependencies)
* Contributors: Bartosz Burda, Michal Faferek
