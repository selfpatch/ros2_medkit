Operations
==========

This section describes the operations API for executing
diagnostic operations such as ROS 2 services and actions.

Operations Overview
-------------------

An **operation resource** represents an executable diagnostic object. In ros2_medkit,
operations map to:

- **ROS 2 Services** - Synchronous operations that return immediately with results
- **ROS 2 Actions** - Asynchronous operations that return an execution ID for status tracking

The API supports:

- **Synchronous execution**: Service calls return after the operation completes
- **Asynchronous execution**: Action goals return immediately with an execution ID; clients poll for status

.. req:: GET /{entity}/operations
   :id: REQ_INTEROP_033
   :status: verified
   :tags: Operations

   The endpoint shall list all supported operations that can be executed on the addressed entity.

   **Response attributes:**

   - ``items``: Array of OperationDescription objects
   - ``id``: Unique identifier for the operation (M)
   - ``name``: Operation name (O)
   - ``proximity_proof_required``: If true, requires co-location proof (M, always false for ROS 2)
   - ``asynchronous_execution``: If true, operation is asynchronous (M; true for actions, false for services)
   - ``x-medkit``: ROS 2-specific metadata (ros2_kind, ros2_service/ros2_action, ros2_type, type_info)

.. req:: GET /{entity}/operations/{op-id}
   :id: REQ_INTEROP_034
   :status: verified
   :tags: Operations

   The endpoint shall return the definition and metadata of the addressed operation.

   **Response attributes:**

   - ``item``: OperationDescription object with full operation details
   - ``modes``: Mode requirements for execution (C, not applicable for ROS 2)

.. req:: POST /{entity}/operations/{op-id}/executions
   :id: REQ_INTEROP_035
   :status: verified
   :tags: Operations

   The endpoint shall start a new execution of the addressed operation on the entity.

   **Request body:**

   - ``timeout``: Timeout in seconds (O)
   - ``parameters``: Operation parameters (C, maps to service request or action goal)

   **Response for synchronous execution (200 OK):**

   - ``parameters``: Response data from service call

   **Response for asynchronous execution (202 Accepted):**

   - ``id``: Execution ID (goal_id) for monitoring
   - ``status``: ExecutionStatus (running, completed, failed, stopped)
   - Location header pointing to execution status endpoint

.. req:: GET /{entity}/operations/{op-id}/executions
   :id: REQ_INTEROP_036
   :status: verified
   :tags: Operations

   The endpoint shall list active and past executions of the addressed operation.

   **Response:**

   - ``items``: Array of objects with ``id`` field containing execution identifiers

.. req:: GET /{entity}/operations/{op-id}/executions/{exec-id}
   :id: REQ_INTEROP_037
   :status: verified
   :tags: Operations

   The endpoint shall return the current status and any result details of the addressed operation execution.

   **Response:**

   - ``status``: ExecutionStatus (running, completed, failed, stopped)
   - ``capability``: Currently executing capability (execute, stop, etc.)
   - ``parameters``: Response parameters or feedback data (C)
   - ``x-medkit``: ROS 2-specific details (goal_id, ros2_status, ros2_action, ros2_type)

.. req:: PUT /{entity}/operations/{op-id}/executions/{exec-id}
   :id: REQ_INTEROP_038
   :status: verified
   :tags: Operations

   The endpoint shall control the addressed operation execution (e.g. execute, freeze, reset, stop)
   and may update execution parameters, if supported.

   **Request body:**

   - ``capability``: Capability to execute (M) - supported: ``stop``
   - ``timeout``: Timeout in seconds (O)
   - ``parameters``: Updated parameters (C)

   **Supported capabilities for ROS 2 actions:**

   - ``stop``: Maps to ROS 2 action cancel - stops the running action
   - ``execute``, ``freeze``, ``reset``: Not supported (returns 400 Bad Request)

   **Response:**

   - ``id``: Execution ID
   - ``status``: Updated execution status
   - Location header for status polling

.. req:: DELETE /{entity}/operations/{op-id}/executions/{exec-id}
   :id: REQ_INTEROP_039
   :status: verified
   :tags: Operations

   The endpoint shall terminate the addressed operation execution (if still running) and remove
   its execution resource, if cancellation is supported.

   **Response:** 204 No Content on successful termination

ExecutionStatus Values
----------------------

The SOVD ExecutionStatus type maps to ROS 2 action statuses:

+---------------+-----------------------------------+
| SOVD Status   | ROS 2 Action Status               |
+===============+===================================+
| running       | ACCEPTED, EXECUTING, CANCELING    |
+---------------+-----------------------------------+
| completed     | SUCCEEDED                         |
+---------------+-----------------------------------+
| failed        | CANCELED, ABORTED                 |
+---------------+-----------------------------------+
| stopped       | (not directly mapped)             |
+---------------+-----------------------------------+

Capability Mapping
------------------

SOVD operation capabilities map to ROS 2 as follows:

+------------+-----------------+-------------------+--------+
| Capability | Description     | ROS 2 Mapping     | Method |
+============+=================+===================+========+
| execute    | Start operation | Send action goal  | POST   |
+------------+-----------------+-------------------+--------+
| stop       | Stop operation  | Cancel action     | PUT    |
+------------+-----------------+-------------------+--------+
| terminate  | Stop + remove   | Cancel + cleanup  | DELETE |
+------------+-----------------+-------------------+--------+
| status     | Get status      | Get goal status   | GET    |
+------------+-----------------+-------------------+--------+
| freeze     | I/O control     | Not applicable    | —      |
+------------+-----------------+-------------------+--------+
| reset      | I/O control     | Not applicable    | —      |
+------------+-----------------+-------------------+--------+

