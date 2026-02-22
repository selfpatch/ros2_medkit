REST API Reference
==================

The ros2_medkit gateway exposes a RESTful API for interacting with ROS 2 systems.
All endpoints are prefixed with ``/api/v1``.

.. note::

   Entity endpoints (``/components``, ``/apps``) share the same handler implementations.
   The examples use ``/components`` but the same patterns apply to ``/apps``.

.. contents:: Table of Contents
   :local:
   :depth: 2

Server Capabilities
-------------------

``GET /api/v1/``
   Get server capabilities and entry points.

   **Example Response:**

   .. code-block:: json

      {
        "api_version": "1.0.0",
        "gateway_version": "0.1.0",
        "endpoints": [
          {"path": "/areas", "supported_methods": ["GET"]},
          {"path": "/components", "supported_methods": ["GET"]},
          {"path": "/apps", "supported_methods": ["GET"]}
        ]
      }

``GET /api/v1/version-info``
   Get gateway version and status information.

``GET /api/v1/health``
   Health check endpoint. Returns HTTP 200 if gateway is operational.

Discovery Endpoints
-------------------

Areas
~~~~~

``GET /api/v1/areas``
   List all areas (logical/physical groupings).

   **Example Response:**

   .. code-block:: json

      {
        "items": [
          {
            "id": "powertrain",
            "name": "Powertrain",
            "self": "/api/v1/areas/powertrain"
          }
        ]
      }

``GET /api/v1/areas/{area_id}``
   Get area capabilities and metadata.

``GET /api/v1/areas/{area_id}/contains``
   List components contained in this area.

``GET /api/v1/areas/{area_id}/components``
   List components in a specific area.

Components
~~~~~~~~~~

``GET /api/v1/components``
   List all components with their operations and capabilities.

   **Example Response:**

   .. code-block:: json

      {
        "items": [
          {
            "id": "temp_sensor",
            "name": "temp_sensor",
            "self": "/api/v1/components/temp_sensor",
            "area": "powertrain",
            "resource_collections": ["data", "operations", "configurations", "faults"]
          }
        ]
      }

``GET /api/v1/components/{component_id}``
   Get component capabilities including available resource collections.

``GET /api/v1/components/{component_id}/hosts``
   List apps hosted on this component (SOVD 7.6.2.4).

``GET /api/v1/components/{component_id}/depends-on``
   List component dependencies.

Apps
~~~~

``GET /api/v1/apps``
   List all apps discovered by the gateway.

   The set of apps is populated either from the static manifest (manifest or hybrid mode)
   or via heuristic runtime discovery of ROS 2 nodes (see :doc:`/tutorials/heuristic-apps`).
   This endpoint may return an empty list if no apps are discovered or if app discovery is
   disabled in the gateway configuration.

``GET /api/v1/apps/{app_id}``
   Get capabilities for a single discovered app.

Functions
~~~~~~~~~

``GET /api/v1/functions``
   List all functions (requires manifest mode or hybrid mode).

``GET /api/v1/functions/{function_id}``
   Get function capabilities.

``GET /api/v1/functions/{function_id}/hosts``
   List apps that host this function.

Data Endpoints
--------------

Read and publish data from ROS 2 topics.

``GET /api/v1/components/{id}/data``
   Read all topic data from an entity.

   **Example Response:**

   .. code-block:: json

      {
        "items": [
          {
            "name": "temperature",
            "data_id": "powertrain%2Fengine%2Ftemperature",
            "type": "std_msgs/msg/Float64",
            "value": {"data": 85.5},
            "timestamp": "2025-01-15T10:30:00Z"
          }
        ],
        "x-medkit": {
          "entity_id": "temp_sensor",
          "total_count": 1
        }
      }

``GET /api/v1/components/{id}/data/{topic_path}``
   Read specific topic data. Topic path is URL-encoded (``/`` → ``%2F``).

   **Example:**

   .. code-block:: bash

      curl http://localhost:8080/api/v1/components/temp_sensor/data/powertrain%2Fengine%2Ftemperature

``PUT /api/v1/components/{id}/data/{topic_path}``
   Publish to a topic.

   - **Content-Type:** application/json
   - **200:** Message published successfully
   - **400:** Invalid message format
   - **401:** Unauthorized (when auth enabled)

   **Example:**

   .. code-block:: bash

      curl -X PUT http://localhost:8080/api/v1/components/brake_actuator/data/chassis%2Fbrakes%2Fcommand \
        -H "Content-Type: application/json" \
        -d '{"data": 50.0}'

Operations Endpoints
--------------------

Execute ROS 2 services and actions.

List Operations
~~~~~~~~~~~~~~~

``GET /api/v1/components/{id}/operations``
   List all operations (services and actions) for an entity.

   **Example Response:**

   .. code-block:: json

      {
        "items": [
          {
            "id": "calibrate",
            "name": "calibrate",
            "type": "service",
            "service_type": "std_srvs/srv/Trigger",
            "schema": {
              "request": {},
              "response": {"success": "bool", "message": "string"}
            }
          },
          {
            "id": "long_calibration",
            "name": "long_calibration",
            "type": "action",
            "action_type": "example_interfaces/action/Fibonacci",
            "schema": {
              "goal": {"order": "int32"},
              "result": {"sequence": "int32[]"},
              "feedback": {"partial_sequence": "int32[]"}
            }
          }
        ],
        "x-medkit": {
          "entity_id": "calibration",
          "total_count": 2
        }
      }

``GET /api/v1/components/{id}/operations/{operation_id}``
   Get operation details and schema.

Execute Operations
~~~~~~~~~~~~~~~~~~

``POST /api/v1/components/{id}/operations/{operation_id}/executions``
   Execute an operation (service call or action goal).

   - **Content-Type:** application/json
   - **200:** Service call completed (sync)
   - **202:** Action goal accepted (async)
   - **400:** Invalid input
   - **404:** Operation not found

   **Service Example (synchronous):**

   .. code-block:: bash

      curl -X POST http://localhost:8080/api/v1/components/calibration/operations/calibrate/executions \
        -H "Content-Type: application/json" \
        -d '{}'

   **Action Example (asynchronous):**

   .. code-block:: bash

      curl -X POST http://localhost:8080/api/v1/components/calibration/operations/long_calibration/executions \
        -H "Content-Type: application/json" \
        -d '{"order": 10}'

   **Action Response (202 Accepted):**

   .. code-block:: json

      {
        "id": "abc123-def456",
        "status": "running"
      }

``GET /api/v1/components/{id}/operations/{operation_id}/executions``
   List all executions for an operation.

``GET /api/v1/components/{id}/operations/{operation_id}/executions/{execution_id}``
   Get execution status and result.

   **Example Response (completed action):**

   .. code-block:: json

      {
        "execution_id": "abc123-def456",
        "status": "succeeded",
        "result": {"sequence": [0, 1, 1, 2, 3, 5, 8, 13, 21, 34]},
        "feedback": [
          {"partial_sequence": [0, 1]},
          {"partial_sequence": [0, 1, 1, 2, 3]}
        ]
      }

``DELETE /api/v1/components/{id}/operations/{operation_id}/executions/{execution_id}``
   Cancel a running execution.

   - **204:** Execution cancelled
   - **404:** Execution not found

Configurations Endpoints
------------------------

Manage ROS 2 node parameters.

``GET /api/v1/components/{id}/configurations``
   List all parameters for an entity.

   **Example Response:**

   .. code-block:: json

      {
        "items": [
          {
            "name": "publish_rate",
            "value": 10.0,
            "type": "double",
            "description": "Publishing rate in Hz"
          },
          {
            "name": "sensor_id",
            "value": "sensor_001",
            "type": "string"
          }
        ],
        "x-medkit": {
          "entity_id": "temp_sensor",
          "total_count": 2
        }
      }

``GET /api/v1/components/{id}/configurations/{param_name}``
   Get a specific parameter value.

``PUT /api/v1/components/{id}/configurations/{param_name}``
   Set a parameter value.

   - **Content-Type:** application/json
   - **200:** Parameter updated
   - **400:** Invalid value
   - **404:** Parameter not found

   **Example:**

   .. code-block:: bash

      curl -X PUT http://localhost:8080/api/v1/components/temp_sensor/configurations/publish_rate \
        -H "Content-Type: application/json" \
        -d '{"value": 20.0}'

``DELETE /api/v1/components/{id}/configurations/{param_name}``
   Reset parameter to default value.

``DELETE /api/v1/components/{id}/configurations``
   Reset all parameters to default values.

Faults Endpoints
----------------

Query and manage faults.

.. note::

   Faults are reported by ROS 2 nodes via the FaultReporter library, not via REST API.
   The gateway queries faults from the ros2_medkit_fault_manager node.

``GET /api/v1/faults``
   List all faults across the system.

``GET /api/v1/components/{id}/faults``
   List all faults for an entity.

   Both endpoints accept an optional ``?status=`` query parameter:

   +-----------------+--------------------------------------------------+
   | Value           | Returns                                          |
   +=================+==================================================+
   | *(default)*     | ``PREFAILED`` + ``CONFIRMED`` (active faults)    |
   +-----------------+--------------------------------------------------+
   | ``pending``     | ``PREFAILED`` only                               |
   +-----------------+--------------------------------------------------+
   | ``confirmed``   | ``CONFIRMED`` only                               |
   +-----------------+--------------------------------------------------+
   | ``cleared``     | ``CLEARED`` + ``HEALED`` + ``PREPASSED``         |
   |                 | (SOVD "cleared" semantics)                       |
   +-----------------+--------------------------------------------------+
   | ``healed``      | ``HEALED`` + ``PREPASSED`` only                  |
   +-----------------+--------------------------------------------------+
   | ``all``         | All statuses                                     |
   +-----------------+--------------------------------------------------+

   **Example Response:**

   .. code-block:: json

      {
        "items": [
          {
            "fault_code": "LIDAR_RANGE_INVALID",
            "severity": "ERROR",
            "message": "Invalid range configuration: min_range > max_range",
            "timestamp": "2025-01-15T10:30:00Z",
            "source": "lidar_driver"
          }
        ],
        "x-medkit": {
          "entity_id": "lidar_sensor",
          "total_count": 1
        }
      }

``GET /api/v1/components/{id}/faults/{fault_code}``
   Get details of a specific fault including environment data.

   **Example Response (200 OK):**

   .. code-block:: json

      {
        "item": {
          "code": "MOTOR_OVERHEAT",
          "fault_name": "Motor temperature exceeded threshold",
          "severity": 2,
          "status": {
            "aggregatedStatus": "active",
            "testFailed": "1",
            "confirmedDTC": "1",
            "pendingDTC": "0"
          }
        },
        "environment_data": {
          "extended_data_records": {
            "first_occurrence": "2026-02-04T10:30:00.000Z",
            "last_occurrence": "2026-02-04T10:35:00.000Z"
          },
          "snapshots": [
            {
              "type": "freeze_frame",
              "name": "motor_temperature",
              "data": 105.5,
              "x-medkit": {
                "topic": "/motor/temperature",
                "message_type": "sensor_msgs/msg/Temperature",
                "full_data": {"temperature": 105.5, "variance": 0.1},
                "captured_at": "2026-02-04T10:30:00.123Z"
              }
            },
            {
              "type": "rosbag",
              "name": "fault_recording",
              "bulk_data_uri": "/apps/motor_controller/bulk-data/rosbags/550e8400-e29b-41d4-a716-446655440000",
              "size_bytes": 1234567,
              "duration_sec": 6.0,
              "format": "mcap"
            }
          ]
        },
        "x-medkit": {
          "occurrence_count": 3,
          "reporting_sources": ["/powertrain/motor_controller"],
          "severity_label": "ERROR"
        }
      }

   **Status Object:**

   The ``status`` object follows SOVD fault status specification:

   - ``aggregatedStatus``: Overall status (``active``, ``passive``, ``cleared``)
   - ``testFailed``: Test failed indicator (``0`` or ``1``)
   - ``confirmedDTC``: Confirmed DTC indicator (``0`` or ``1``)
   - ``pendingDTC``: Pending DTC indicator (``0`` or ``1``)

   **Snapshot Types:**

   - ``freeze_frame``: Topic data captured at fault confirmation
   - ``rosbag``: Recording file available via bulk-data endpoint

``DELETE /api/v1/components/{id}/faults/{fault_code}``
   Clear a fault.

   - **204:** Fault cleared
   - **404:** Fault not found

``DELETE /api/v1/faults``
   Clear all faults across the system *(ros2_medkit extension, not SOVD)*.

   Accepts the optional ``?status=`` query parameter (same values as ``GET /faults``).
   Without it, clears pending and confirmed faults.

   - **204:** Faults cleared (or none to clear)
   - **400:** Invalid status parameter
   - **503:** Fault manager unavailable

Bulk Data Endpoints
-------------------

Access, upload, and delete large binary data (rosbags, calibration files, firmware, etc.)
associated with entities. Read endpoints (GET) support all entity types. Write endpoints
(POST, DELETE) are supported for components and apps only.

List Categories
~~~~~~~~~~~~~~~

``GET /api/v1/{entity-path}/bulk-data``

List available bulk-data categories for an entity. Returns the union of rosbag categories
(from the fault manager) and configured categories (from ``bulk_data.categories``).

**Supported entity paths:**

- ``/apps/{app-id}``
- ``/components/{component-id}``
- ``/areas/{area-id}``
- ``/functions/{function-id}``
- ``/areas/{area-id}/subareas/{subarea-id}``
- ``/components/{component-id}/subcomponents/{subcomponent-id}``

**Example:**

.. code-block:: bash

   curl http://localhost:8080/api/v1/apps/motor_controller/bulk-data

**Response (200 OK):**

.. code-block:: json

   {
     "items": ["rosbags", "calibration", "firmware"]
   }

List Bulk Data Items
~~~~~~~~~~~~~~~~~~~~

``GET /api/v1/{entity-path}/bulk-data/{category}``

List all bulk-data items in a category for the entity.

**Example:**

.. code-block:: bash

   curl http://localhost:8080/api/v1/apps/motor_controller/bulk-data/rosbags

**Response (200 OK):**

.. code-block:: json

   {
     "items": [
       {
         "id": "550e8400-e29b-41d4-a716-446655440000",
         "name": "MOTOR_OVERHEAT recording 2026-02-04T10:30:00Z",
         "mimetype": "application/x-mcap",
         "size": 1234567,
         "creation_date": "2026-02-04T10:30:00.000Z",
         "x-medkit": {
           "fault_code": "MOTOR_OVERHEAT",
           "duration_sec": 6.0,
           "format": "mcap"
         }
       }
     ]
   }

Download Bulk Data
~~~~~~~~~~~~~~~~~~

``GET /api/v1/{entity-path}/bulk-data/{category}/{id}``

Download a specific bulk-data file.

**Response Headers:**

- ``Content-Type``: ``application/x-mcap`` (MCAP format) or ``application/x-sqlite3`` (db3)
- ``Content-Disposition``: ``attachment; filename="FAULT_CODE.mcap"``
- ``Access-Control-Expose-Headers``: ``Content-Disposition``

**Example:**

.. code-block:: bash

   curl -O -J http://localhost:8080/api/v1/apps/motor_controller/bulk-data/rosbags/550e8400-e29b-41d4-a716-446655440000

**Response Codes:**

- **200 OK**: File content
- **404 Not Found**: Entity, category, or bulk-data ID not found

Upload Bulk Data
~~~~~~~~~~~~~~~~

``POST /api/v1/{entity-path}/bulk-data/{category}``

Upload a new bulk-data file to the specified category. Files are sent as
``multipart/form-data``. The ``rosbags`` category is read-only and cannot be
used for uploads.

**Supported entity types:** components, apps only. Areas and functions return 405.

**Form Fields:**

.. list-table::
   :header-rows: 1
   :widths: 20 15 65

   * - Field
     - Required
     - Description
   * - ``file``
     - Yes
     - The file to upload (binary data with filename and content type).
   * - ``description``
     - No
     - Human-readable description of the file.
   * - ``metadata``
     - No
     - JSON string with arbitrary key-value metadata.

**Example:**

.. code-block:: bash

   curl -X POST http://localhost:8080/api/v1/components/motor_controller/bulk-data/calibration \
     -F "file=@calibration_data.bin;type=application/octet-stream" \
     -F "description=Motor calibration parameters v2.1" \
     -F 'metadata={"version": "2.1", "author": "engineer_01"}'

**Response (201 Created):**

.. code-block:: json

   {
     "id": "calibration_1739612345000000000_ab12cd34",
     "name": "calibration_data.bin",
     "mimetype": "application/octet-stream",
     "size": 4096,
     "creation_date": "2026-03-15T14:30:00.000Z",
     "description": "Motor calibration parameters v2.1",
     "x-medkit": {
       "version": "2.1",
       "author": "engineer_01"
     }
   }

**Response Headers:**

- ``Location``: ``/api/v1/components/motor_controller/bulk-data/calibration/calibration_1739612345000000000_ab12cd34``

**Error Responses:**

- **400 Bad Request**: Missing ``file`` field, unknown category, or ``rosbags`` category
- **405 Method Not Allowed**: Upload attempted on areas or functions
- **413 Payload Too Large**: File exceeds ``bulk_data.max_upload_size``

Delete Bulk Data
~~~~~~~~~~~~~~~~

``DELETE /api/v1/{entity-path}/bulk-data/{category}/{id}``

Delete a specific bulk-data item. The ``rosbags`` category is managed by the
fault manager and cannot be deleted via this endpoint.

**Supported entity types:** components, apps only. Areas and functions return 405.

**Example:**

.. code-block:: bash

   curl -X DELETE http://localhost:8080/api/v1/components/motor_controller/bulk-data/calibration/calibration_1739612345000000000_ab12cd34

**Response Codes:**

- **204 No Content**: Item deleted successfully
- **400 Bad Request**: ``rosbags`` category (managed by fault manager)
- **404 Not Found**: Entity, category, or bulk-data ID not found
- **405 Method Not Allowed**: Delete attempted on areas or functions

Software Updates
----------------

Manage software update packages with an async prepare/execute lifecycle.
The updates feature requires a backend plugin to be loaded (see :doc:`/config/server`).
Without a plugin, all endpoints return ``501 Not Implemented``.

``GET /api/v1/updates``
   List all registered update packages.

   **Query Parameters:**

   - ``origin`` (optional): Filter by origin (``remote`` or ``proximity``)
   - ``target-version`` (optional): Filter by target version

   **Example Response (200 OK):**

   .. code-block:: json

      {
        "items": ["firmware-v2.1", "calibration-update-3"]
      }

``POST /api/v1/updates``
   Register a new update package.

   **Request Body:**

   .. code-block:: json

      {
        "id": "firmware-v2.1",
        "update_name": "Firmware Update v2.1",
        "automated": true,
        "origins": ["remote"],
        "duration": 600,
        "size": 52428800,
        "updated_components": ["ecu_main"],
        "affected_components": ["ecu_main", "ecu_secondary"]
      }

   **Response (201 Created):**

   .. code-block:: json

      {
        "id": "firmware-v2.1"
      }

   **Response Headers:**

   - ``Location``: ``/api/v1/updates/firmware-v2.1``

``GET /api/v1/updates/{id}``
   Get full metadata for a specific update package.

   **Response (200 OK):**

   Returns the JSON metadata as registered.

   - **404 Not Found:** Package does not exist

``DELETE /api/v1/updates/{id}``
   Delete an update package.

   - **204 No Content:** Package deleted
   - **404 Not Found:** Package does not exist
   - **409 Conflict:** Operation in progress for this package

``PUT /api/v1/updates/{id}/prepare``
   Trigger preparation of an update (download, verify, check dependencies).
   Runs asynchronously - poll the status endpoint for progress.

   - **202 Accepted:** Preparation started
   - **404 Not Found:** Package does not exist
   - **409 Conflict:** Operation already in progress

   **Response Headers:**

   - ``Location``: ``/api/v1/updates/{id}/status``

``PUT /api/v1/updates/{id}/execute``
   Trigger execution of a prepared update (install). Only succeeds after
   prepare has completed.

   - **202 Accepted:** Execution started
   - **400 Bad Request:** Package not prepared
   - **404 Not Found:** Package does not exist
   - **409 Conflict:** Operation already in progress

   **Response Headers:**

   - ``Location``: ``/api/v1/updates/{id}/status``

``PUT /api/v1/updates/{id}/automated``
   Trigger automated update (prepare + execute in one step). Only works
   for packages that support automated mode.

   - **202 Accepted:** Automated update started
   - **400 Bad Request:** Package does not support automated mode
   - **404 Not Found:** Package does not exist
   - **409 Conflict:** Operation already in progress

   **Response Headers:**

   - ``Location``: ``/api/v1/updates/{id}/status``

``GET /api/v1/updates/{id}/status``
   Get the current status and progress of an update operation.

   **Example Response (200 OK):**

   .. code-block:: json

      {
        "status": "inProgress",
        "progress": 65,
        "sub_progress": [
          {"name": "download", "progress": 100},
          {"name": "verify", "progress": 30}
        ]
      }

   **Status values:** ``pending``, ``inProgress``, ``completed``, ``failed``

   When ``status`` is ``failed``, an ``error`` object is included:

   .. code-block:: json

      {
        "status": "failed",
        "error": {
          "error_code": "internal-error",
          "message": "Download failed: connection timeout"
        }
      }

   - **404 Not Found:** No status available (package not found or no operation started)

Cyclic Subscriptions
--------------------

Cyclic subscriptions provide periodic push-based data delivery via Server-Sent Events (SSE).
A client creates a subscription specifying which data resource to observe and at what interval.
The server then pushes the latest value of that resource at the requested frequency.

Subscriptions are temporary — they do not survive server restart.

``POST /api/v1/{entity_type}/{entity_id}/cyclic-subscriptions``
   Create a new cyclic subscription.

   **Applies to:** ``/apps``, ``/components``

   **Request Body:**

   .. code-block:: json

      {
        "resource": "/api/v1/apps/temp_sensor/data/temperature",
        "protocol": "sse",
        "interval": "normal",
        "duration": 300
      }

   **Fields:**

   - ``resource`` (string, required): Full URI of the data resource to observe
   - ``protocol`` (string, optional): Transport protocol. Only ``"sse"`` supported. Default: ``"sse"``
   - ``interval`` (string, required): One of ``fast`` (<100ms), ``normal`` (100-250ms), ``slow`` (250-500ms)
   - ``duration`` (integer, required): Subscription lifetime in seconds (must be > 0)

   **Response 201 Created:**

   .. code-block:: json

      {
        "id": "sub_001",
        "observed_resource": "/api/v1/apps/temp_sensor/data/temperature",
        "event_source": "/api/v1/apps/temp_sensor/cyclic-subscriptions/sub_001/events",
        "protocol": "sse",
        "interval": "normal"
      }

``GET /api/v1/{entity_type}/{entity_id}/cyclic-subscriptions``
   List all active cyclic subscriptions for an entity. Returns ``{"items": [...]}``.

``GET /api/v1/{entity_type}/{entity_id}/cyclic-subscriptions/{id}``
   Get details of a single subscription.

``PUT /api/v1/{entity_type}/{entity_id}/cyclic-subscriptions/{id}``
   Update ``interval`` and/or ``duration`` of an existing subscription.
   Only provided fields are updated.

   **Request Body:**

   .. code-block:: json

      {
        "interval": "fast",
        "duration": 600
      }

``DELETE /api/v1/{entity_type}/{entity_id}/cyclic-subscriptions/{id}``
   Cancel and remove a subscription. Returns 204 No Content.

``GET /api/v1/{entity_type}/{entity_id}/cyclic-subscriptions/{id}/events``
   SSE event stream. Connect to receive periodic data updates.

   **Response Headers:**

   .. code-block:: text

      Content-Type: text/event-stream
      Cache-Control: no-cache
      Connection: keep-alive

   **Event Format (EventEnvelope):**

   .. code-block:: text

      data: {"timestamp":"2026-02-14T10:30:00.250Z","payload":{"id":"/temperature","data":{"data":23.5}}}

   The stream auto-closes when the duration expires, the client disconnects,
   or the subscription is deleted.

Rate Limiting
-------------

The gateway supports token-bucket-based rate limiting to protect endpoints from abuse. Rate limiting is disabled by default and can be enabled via configuration parameters.

Configuration
~~~~~~~~~~~~~

You can configure global and per-client RPM (requests per minute) limits:

- ``rate_limiting.enabled``: ``true`` to enable.
- ``rate_limiting.global_requests_per_minute``: Overarching limit across all clients.
- ``rate_limiting.client_requests_per_minute``: Limit per individual client IP.

Endpoint limits can also be overridden with patterns:

- ``rate_limiting.endpoint_limits``: List of ``"pattern:rpm"`` strings. For example, ``["/api/v1/*/operations/*:10"]`` limits execution calls without affecting other data endpoints.

Response Headers
~~~~~~~~~~~~~~~~

When rate limiting is enabled, the gateway includes the following HTTP response headers on every check:

- ``X-RateLimit-Limit``: The effective RPM limit applied.
- ``X-RateLimit-Remaining``: Number of requests remaining in the current minute window.
- ``X-RateLimit-Reset``: Unix epoch time (in seconds) when the limit bucket resets.

Rejection (429 Too Many Requests)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If a request exceeds the available tokens, it is rejected with an HTTP 429 status code and a ``Retry-After`` header indicating the number of seconds to wait before retrying.

**Example Response:**

.. code-block:: json

   {
     "error_code": 429,
     "message": "Too many requests. Please retry after 10 seconds.",
     "parameters": {
       "retry_after": 10,
       "limit": 60,
       "reset": 1739612355
     }
   }

Authentication Endpoints
------------------------

JWT-based authentication with Role-Based Access Control (RBAC).

.. seealso::

   :doc:`/tutorials/authentication` for configuration details.

``POST /api/v1/auth/authorize``
   Authenticate with client credentials.

   **Request:**

   .. code-block:: json

      {
        "grant_type": "client_credentials",
        "client_id": "admin",
        "client_secret": "admin_secret_key"
      }

   **Response:**

   .. code-block:: json

      {
        "access_token": "eyJhbGciOiJIUzI1NiIs...",
        "token_type": "Bearer",
        "expires_in": 3600,
        "refresh_token": "dGhpcyBpcyBhIHJlZnJlc2g...",
        "scope": "admin"
      }

``POST /api/v1/auth/token``
   Refresh access token.

   **Request:**

   .. code-block:: json

      {
        "grant_type": "refresh_token",
        "refresh_token": "dGhpcyBpcyBhIHJlZnJlc2g..."
      }

``POST /api/v1/auth/revoke``
   Revoke a token.

   **Request:**

   .. code-block:: json

      {"token": "dGhpcyBpcyBhIHJlZnJlc2g..."}

Error Responses
---------------

All error responses follow a consistent format:

.. code-block:: json

   {
     "error": {
       "code": "ERR_ENTITY_NOT_FOUND",
       "message": "Entity not found",
       "details": {
         "entity_id": "unknown_component"
       }
     }
   }

Common Error Codes
~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 30 15 55

   * - Error Code
     - HTTP Status
     - Description
   * - ``ERR_ENTITY_NOT_FOUND``
     - 404
     - The requested entity does not exist
   * - ``ERR_RESOURCE_NOT_FOUND``
     - 404
     - The requested resource (topic, service, parameter) does not exist
   * - ``ERR_INVALID_INPUT``
     - 400
     - Invalid request body or parameters
   * - ``ERR_INVALID_ENTITY_ID``
     - 400
     - Entity ID contains invalid characters
   * - ``ERR_OPERATION_FAILED``
     - 500
     - Operation failed during execution
   * - ``ERR_TIMEOUT``
     - 504
     - Operation timed out
   * - ``ERR_UNAUTHORIZED``
     - 401
     - Authentication required or token invalid
   * - ``ERR_FORBIDDEN``
     - 403
     - Insufficient permissions for this operation

URL Encoding
------------

Topic and parameter paths containing ``/`` must be URL-encoded:

.. list-table::
   :header-rows: 1
   :widths: 50 50

   * - Original Path
     - URL Encoded
   * - ``/powertrain/engine/temperature``
     - ``powertrain%2Fengine%2Ftemperature``
   * - ``/chassis/brakes/command``
     - ``chassis%2Fbrakes%2Fcommand``

SOVD Compliance
---------------

The gateway implements a subset of the SOVD (Service-Oriented Vehicle Diagnostics) specification:

**SOVD-Compliant Endpoints:**

- Discovery (``/areas``, ``/components``, ``/apps``, ``/functions``)
- Data access (``/data``)
- Operations (``/operations``, ``/executions``)
- Configurations (``/configurations``)
- Faults (``/faults``) with ``environment_data`` and SOVD status object
- Bulk Data (``/bulk-data``) for binary data downloads (rosbags, logs)
- Software Updates (``/updates``) with async prepare/execute lifecycle
- Cyclic Subscriptions (``/cyclic-subscriptions``) with SSE-based periodic data delivery

**ros2_medkit Extensions:**

- ``/health`` - Health check endpoint
- ``/version-info`` - Gateway version information
- ``/manifest/status`` - Manifest discovery status
- SSE fault streaming - Real-time fault notifications
- ``x-medkit`` extension fields in responses

See Also
--------

- :doc:`/tutorials/authentication` - Configure authentication
- :doc:`/config/server` - Server configuration options
- `Postman Collection <https://github.com/selfpatch/ros2_medkit/tree/main/postman>`_ - Interactive API testing
