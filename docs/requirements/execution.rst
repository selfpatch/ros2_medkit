Execution (Operations & Scripts)
================================

.. req:: POST /{entity}/triggers
   :id: REQ_SOVD_034
   :status: open
   :tags: P

   **Goal:** Define trigger

   **ROS 2 Equivalent:** Event rules

   **Implementation Notes:** Example: diag.level >= WARN triggers an action or service call.

.. req:: GET /{entity}/triggers
   :id: REQ_SOVD_035
   :status: open
   :tags: P

   **Goal:** List triggers

   **ROS 2 Equivalent:** Event rules

   **Implementation Notes:** Return all configured triggers from DB or parameters.

.. req:: PUT /{entity}/triggers/{id}
   :id: REQ_SOVD_036
   :status: open
   :tags: P

   **Goal:** Update trigger

   **ROS 2 Equivalent:** Event rules

   **Implementation Notes:** Update trigger condition, target operation or severity.

.. req:: DELETE /{entity}/triggers/{id}
   :id: REQ_SOVD_037
   :status: open
   :tags: P

   **Goal:** Delete trigger

   **ROS 2 Equivalent:** Event rules

   **Implementation Notes:** Remove a trigger definition.

.. req:: GET /{entity}/operations
   :id: REQ_SOVD_038
   :status: open
   :tags: A

   **Goal:** List operations

   **ROS 2 Equivalent:** Available actions/services

   **Implementation Notes:** Collect all actions and services in the entity's namespace.

.. req:: GET /{entity}/operations/{op-id}
   :id: REQ_SOVD_039
   :status: open
   :tags: A

   **Goal:** Operation details

   **ROS 2 Equivalent:** Action/service definition

   **Implementation Notes:** Expose .action/.srv types, QoS and basic documentation.

.. req:: POST /{entity}/operations/{op-id}/executions
   :id: REQ_SOVD_040
   :status: open
   :tags: A

   **Goal:** Start operation

   **ROS 2 Equivalent:** Send action goal or call service

   **Implementation Notes:** Map one execution to one service call or action goal (goal ID = execution ID).

.. req:: GET /{entity}/operations/{op-id}/executions
   :id: REQ_SOVD_041
   :status: open
   :tags: P

   **Goal:** Execution history / active executions

   **ROS 2 Equivalent:** List of goals / calls

   **Implementation Notes:** Keep a registry of past and active executions in the gateway.

.. req:: GET /{entity}/operations/executions/{exec-id}
   :id: REQ_SOVD_042
   :status: open
   :tags: A/P

   **Goal:** Execution status

   **ROS 2 Equivalent:** Goal status / service call state

   **Implementation Notes:** Read feedback and result for actions or status for long-running service calls.

.. req:: PUT /{entity}/operations/executions/{exec-id}
   :id: REQ_SOVD_043
   :status: open
   :tags: P

   **Goal:** Modify execution

   **ROS 2 Equivalent:** Goal update (if supported)

   **Implementation Notes:** Support goal updates only if the underlying action supports it.

.. req:: DELETE /{entity}/operations/executions/{exec-id}
   :id: REQ_SOVD_044
   :status: open
   :tags: A

   **Goal:** Cancel execution

   **ROS 2 Equivalent:** CancelGoal / cancel service

   **Implementation Notes:** Map to CancelGoal for actions or a custom cancel mechanism for services.

.. req:: POST /{entity}/scripts
   :id: REQ_SOVD_045
   :status: open
   :tags: P

   **Goal:** Upload script

   **ROS 2 Equivalent:** Script runner (launch/BT/process)

   **Implementation Notes:** Upload a script and register it with a runner that can start/stop processes.

.. req:: GET /{entity}/scripts
   :id: REQ_SOVD_046
   :status: open
   :tags: P

   **Goal:** List scripts

   **ROS 2 Equivalent:** Registered scripts

   **Implementation Notes:** Return metadata and paths of all scripts registered for the entity.

.. req:: GET /{entity}/scripts/{id}
   :id: REQ_SOVD_047
   :status: open
   :tags: P

   **Goal:** Script details

   **ROS 2 Equivalent:** Registered script

   **Implementation Notes:** Return metadata and path for a specific script.

.. req:: DELETE /{entity}/scripts/{id}
   :id: REQ_SOVD_048
   :status: open
   :tags: P

   **Goal:** Delete script

   **ROS 2 Equivalent:** Stop and remove script

   **Implementation Notes:** Stop any running instance and remove the script from the registry.

.. req:: POST /{entity}/scripts/{id}/executions
   :id: REQ_SOVD_049
   :status: open
   :tags: P

   **Goal:** Start script

   **ROS 2 Equivalent:** Run ros2 launch / behavior tree / process

   **Implementation Notes:** Start the script; execution ID can be PID or a generated UUID.

.. req:: GET /{entity}/scripts/{id}/executions
   :id: REQ_SOVD_050
   :status: open
   :tags: P

   **Goal:** List script executions

   **ROS 2 Equivalent:** Process monitoring

   **Implementation Notes:** Return recent executions with start time, end time and exit code.

.. req:: GET /{entity}/scripts/{id}/executions/{exec-id}
   :id: REQ_SOVD_051
   :status: open
   :tags: P

   **Goal:** Script execution status

   **ROS 2 Equivalent:** Process monitoring

   **Implementation Notes:** Expose logs and exit code for a single execution.

.. req:: PUT /{entity}/scripts/{id}/executions/{exec-id}
   :id: REQ_SOVD_052
   :status: open
   :tags: P

   **Goal:** Control script execution

   **ROS 2 Equivalent:** Signals (pause / continue / stop)

   **Implementation Notes:** Requires runner support for signals like pause, resume and graceful stop.

