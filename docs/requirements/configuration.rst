Configuration & Lifecycle
=========================

.. req:: GET /{entity}/configurations
   :id: REQ_SOVD_053
   :status: open
   :tags: D

   **Goal:** Configuration overview

   **ROS 2 Equivalent:** ROS 2 parameters

   **Implementation Notes:** Use rcl_interfaces/GetParameters or equivalent to read current parameters.

.. req:: GET /{entity}/configurations/{id}
   :id: REQ_SOVD_054
   :status: open
   :tags: D/P

   **Goal:** Read configuration snapshot

   **ROS 2 Equivalent:** Parameter snapshot

   **Implementation Notes:** Read a logical group of parameters by ID; grouping is a proposed convention.

.. req:: PUT /{entity}/configurations/{id}
   :id: REQ_SOVD_055
   :status: open
   :tags: D/P

   **Goal:** Apply configuration

   **ROS 2 Equivalent:** Set parameters

   **Implementation Notes:** Apply a saved parameter set transactionally with optional rollback on failure.

.. req:: DELETE /{entity}/configurations
   :id: REQ_SOVD_056
   :status: open
   :tags: P

   **Goal:** Clear configurations

   **ROS 2 Equivalent:** Restore defaults

   **Implementation Notes:** Custom implementation to reset parameters to entity-defined defaults.

.. req:: DELETE /{entity}/configurations/{id}
   :id: REQ_SOVD_057
   :status: open
   :tags: P

   **Goal:** Delete configuration snapshot

   **ROS 2 Equivalent:** Remove stored configuration

   **Implementation Notes:** Remove a stored configuration profile from DB or parameters.

.. req:: GET /{entity}/modes
   :id: REQ_SOVD_058
   :status: open
   :tags: D

   **Goal:** List modes

   **ROS 2 Equivalent:** Lifecycle states / custom modes

   **Implementation Notes:** Expose standard lifecycle states (unconfigured/configured/active, etc.) and optional custom modes.

.. req:: GET /{entity}/modes/{mode-id}
   :id: REQ_SOVD_059
   :status: open
   :tags: D

   **Goal:** Read mode

   **ROS 2 Equivalent:** Lifecycle state

   **Implementation Notes:** Return current lifecycle state or custom mode for the entity.

.. req:: PUT /{entity}/modes/{mode-id}
   :id: REQ_SOVD_060
   :status: open
   :tags: D

   **Goal:** Change mode

   **ROS 2 Equivalent:** Lifecycle transition

   **Implementation Notes:** Trigger lifecycle transitions: configure, activate, deactivate, cleanup, shutdown.

.. req:: GET /{entity}/clear-data
   :id: REQ_SOVD_061
   :status: open
   :tags: P

   **Goal:** Clear options overview

   **ROS 2 Equivalent:** Available reset/clear operations

   **Implementation Notes:** Expose which clear/reset operations the entity supports.

.. req:: PUT /{entity}/clear-data/cached-data
   :id: REQ_SOVD_062
   :status: open
   :tags: P

   **Goal:** Clear cached data

   **ROS 2 Equivalent:** Reset node buffers

   **Implementation Notes:** Custom service that clears caches or rolling buffers inside the node.

.. req:: PUT /{entity}/clear-data/learned-data
   :id: REQ_SOVD_063
   :status: open
   :tags: P

   **Goal:** Clear learned data

   **ROS 2 Equivalent:** Reset models/adaptation

   **Implementation Notes:** Custom maintenance service that resets learned models or adaptive parameters.

.. req:: PUT /{entity}/clear-data/client-defined-resources
   :id: REQ_SOVD_064
   :status: open
   :tags: P

   **Goal:** Clear client-defined resources

   **ROS 2 Equivalent:** Delete presets/logs

   **Implementation Notes:** Gateway removes stored presets, logs or other client-managed artifacts.

.. req:: GET /{entity}/clear-data/status
   :id: REQ_SOVD_065
   :status: open
   :tags: P

   **Goal:** Clear operation status

   **ROS 2 Equivalent:** Progress/result

   **Implementation Notes:** Track clear operations and expose their status from the gateway registry.

.. req:: GET /{entity}/status
   :id: REQ_SOVD_080
   :status: open
   :tags: A

   **Goal:** Entity status

   **ROS 2 Equivalent:** Lifecycle state + health summary

   **Implementation Notes:** Combine lifecycle state and aggregated diagnostics into a single status object.

.. req:: PUT /{entity}/status/start
   :id: REQ_SOVD_081
   :status: open
   :tags: A/P

   **Goal:** Start entity

   **ROS 2 Equivalent:** Activate lifecycle / start process

   **Implementation Notes:** For lifecycle nodes: activate; for external processes: delegate to systemd or a supervisor.

.. req:: PUT /{entity}/status/restart
   :id: REQ_SOVD_082
   :status: open
   :tags: P

   **Goal:** Restart entity

   **ROS 2 Equivalent:** Restart node/process

   **Implementation Notes:** Use systemd/supervisor or a custom runner to stop and start the entity.

.. req:: PUT /{entity}/status/restart/force-restart
   :id: REQ_SOVD_083
   :status: open
   :tags: P

   **Goal:** Force restart entity

   **ROS 2 Equivalent:** Force kill + start

   **Implementation Notes:** Send SIGKILL or equivalent and then restart via runner or systemd.

.. req:: PUT /{entity}/status/shutdown
   :id: REQ_SOVD_084
   :status: open
   :tags: A/P

   **Goal:** Shutdown entity

   **ROS 2 Equivalent:** Deactivate / shutdown

   **Implementation Notes:** For lifecycle nodes: deactivate/shutdown; for processes: stop via supervisor.

.. req:: PUT /{entity}/status/force-shutdown
   :id: REQ_SOVD_085
   :status: open
   :tags: P

   **Goal:** Force shutdown entity

   **ROS 2 Equivalent:** SIGKILL

   **Implementation Notes:** Last resort: hard kill the process or node.

