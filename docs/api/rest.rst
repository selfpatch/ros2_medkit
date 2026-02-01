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

.. http:get:: /api/v1/

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

.. http:get:: /api/v1/version-info

   Get gateway version and status information.

.. http:get:: /api/v1/health

   Health check endpoint. Returns HTTP 200 if gateway is operational.

Discovery Endpoints
-------------------

Areas
~~~~~

.. http:get:: /api/v1/areas

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

.. http:get:: /api/v1/areas/{area_id}

   Get area capabilities and metadata.

.. http:get:: /api/v1/areas/{area_id}/contains

   List components contained in this area.

.. http:get:: /api/v1/areas/{area_id}/components

   List components in a specific area.

Components
~~~~~~~~~~

.. http:get:: /api/v1/components

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

.. http:get:: /api/v1/components/{component_id}

   Get component capabilities including available resource collections.

.. http:get:: /api/v1/components/{component_id}/hosts

   List apps hosted on this component (SOVD 7.6.2.4).

.. http:get:: /api/v1/components/{component_id}/depends-on

   List component dependencies.

Apps
~~~~

.. http:get:: /api/v1/apps

   List all apps (requires manifest mode or hybrid mode).

.. http:get:: /api/v1/apps/{app_id}

   Get app capabilities.

Functions
~~~~~~~~~

.. http:get:: /api/v1/functions

   List all functions (requires manifest mode or hybrid mode).

.. http:get:: /api/v1/functions/{function_id}

   Get function capabilities.

.. http:get:: /api/v1/functions/{function_id}/hosts

   List apps that host this function.

Data Endpoints
--------------

Read and publish data from ROS 2 topics.

.. http:get:: /api/v1/components/{id}/data

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
        ]
      }

.. http:get:: /api/v1/components/{id}/data/{topic_path}

   Read specific topic data. Topic path is URL-encoded (``/`` → ``%2F``).

   **Example:**

   .. code-block:: bash

      curl http://localhost:8080/api/v1/components/temp_sensor/data/powertrain%2Fengine%2Ftemperature

.. http:put:: /api/v1/components/{id}/data/{topic_path}

   Publish to a topic.

   :reqheader Content-Type: application/json
   :statuscode 200: Message published successfully
   :statuscode 400: Invalid message format
   :statuscode 401: Unauthorized (when auth enabled)

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

.. http:get:: /api/v1/components/{id}/operations

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
        ]
      }

.. http:get:: /api/v1/components/{id}/operations/{operation_id}

   Get operation details and schema.

Execute Operations
~~~~~~~~~~~~~~~~~~

.. http:post:: /api/v1/components/{id}/operations/{operation_id}/executions

   Execute an operation (service call or action goal).

   :reqheader Content-Type: application/json
   :statuscode 200: Service call completed (sync)
   :statuscode 201: Action goal accepted (async)
   :statuscode 400: Invalid input
   :statuscode 404: Operation not found

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

   **Action Response:**

   .. code-block:: json

      {
        "execution_id": "abc123-def456",
        "status": "executing",
        "self": "/api/v1/components/calibration/operations/long_calibration/executions/abc123-def456"
      }

.. http:get:: /api/v1/components/{id}/operations/{operation_id}/executions

   List all executions for an operation.

.. http:get:: /api/v1/components/{id}/operations/{operation_id}/executions/{execution_id}

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

.. http:delete:: /api/v1/components/{id}/operations/{operation_id}/executions/{execution_id}

   Cancel a running execution.

   :statuscode 200: Execution cancelled
   :statuscode 404: Execution not found

Configurations Endpoints
------------------------

Manage ROS 2 node parameters.

.. http:get:: /api/v1/components/{id}/configurations

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
        ]
      }

.. http:get:: /api/v1/components/{id}/configurations/{param_name}

   Get a specific parameter value.

.. http:put:: /api/v1/components/{id}/configurations/{param_name}

   Set a parameter value.

   :reqheader Content-Type: application/json
   :statuscode 200: Parameter updated
   :statuscode 400: Invalid value
   :statuscode 404: Parameter not found

   **Example:**

   .. code-block:: bash

      curl -X PUT http://localhost:8080/api/v1/components/temp_sensor/configurations/publish_rate \
        -H "Content-Type: application/json" \
        -d '{"value": 20.0}'

.. http:delete:: /api/v1/components/{id}/configurations/{param_name}

   Reset parameter to default value.

.. http:delete:: /api/v1/components/{id}/configurations

   Reset all parameters to default values.

Faults Endpoints
----------------

Query and manage faults.

.. note::

   Faults are reported by ROS 2 nodes via the FaultReporter library, not via REST API.
   The gateway queries faults from the ros2_medkit_fault_manager node.

.. http:get:: /api/v1/components/{id}/faults

   List all faults for an entity.

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
        ]
      }

.. http:get:: /api/v1/components/{id}/faults/{fault_code}

   Get specific fault details.

.. http:delete:: /api/v1/components/{id}/faults/{fault_code}

   Clear a fault.

   :statuscode 200: Fault cleared
   :statuscode 404: Fault not found

Authentication Endpoints
------------------------

JWT-based authentication with Role-Based Access Control (RBAC).

.. seealso::

   :doc:`/tutorials/authentication` for configuration details.

.. http:post:: /api/v1/auth/authorize

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

.. http:post:: /api/v1/auth/token

   Refresh access token.

   **Request:**

   .. code-block:: json

      {
        "grant_type": "refresh_token",
        "refresh_token": "dGhpcyBpcyBhIHJlZnJlc2g..."
      }

.. http:post:: /api/v1/auth/revoke

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
- Faults (``/faults``)

**ros2_medkit Extensions:**

- ``/health`` - Health check endpoint
- ``/version-info`` - Gateway version information
- ``/manifest/status`` - Manifest discovery status
- SSE fault streaming - Real-time fault notifications

See Also
--------

- :doc:`/tutorials/authentication` - Configure authentication
- :doc:`/config/server` - Server configuration options
- `Postman Collection <https://github.com/selfpatch/ros2_medkit/tree/main/postman>`_ - Interactive API testing
