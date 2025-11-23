Data & Bulk Data
================

.. req:: GET /{entity}/data-categories
   :id: REQ_SOVD_021
   :status: open
   :tags: A

   **Goal:** Data categories

   **ROS 2 Equivalent:** Message packages (sensor_msgs, nav_msgs, etc.)

   **Implementation Notes:** Expose a list of message packages / types used by the entity.

.. req:: GET /{entity}/data-groups
   :id: REQ_SOVD_022
   :status: open
   :tags: P

   **Goal:** Signal groups

   **ROS 2 Equivalent:** Bundles / namespaces

   **Implementation Notes:** Define named bundles of topics in gateway configuration.

.. req:: GET /{entity}/data
   :id: REQ_SOVD_023
   :status: open
   :tags: P

   **Goal:** Data snapshot

   **ROS 2 Equivalent:** Temporary subscription and sample return

   **Implementation Notes:** Subscribe briefly, cache the latest sample and return it with a timeout.

.. req:: GET /{entity}/data/{data-id}
   :id: REQ_SOVD_024
   :status: open
   :tags: A

   **Goal:** Single signal

   **ROS 2 Equivalent:** Specific topic

   **Implementation Notes:** Map data-id to a fully qualified topic name.

.. req:: PUT /{entity}/data/{data-id}
   :id: REQ_SOVD_025
   :status: open
   :tags: A/P

   **Goal:** Write / command

   **ROS 2 Equivalent:** Publish to a 'command' topic or call a service

   **Implementation Notes:** Prefer services for side effects; use command topics when no service exists.

.. req:: GET /{entity}/data-lists
   :id: REQ_SOVD_026
   :status: open
   :tags: P

   **Goal:** List data sets

   **ROS 2 Equivalent:** Subscription presets

   **Implementation Notes:** Store named subscription presets (topic lists) in a DB or parameters.

.. req:: POST /{entity}/data-lists
   :id: REQ_SOVD_027
   :status: open
   :tags: P

   **Goal:** Create data set

   **ROS 2 Equivalent:** Define presets

   **Implementation Notes:** Persist a list of topics plus QoS settings under a preset name.

.. req:: GET /{entity}/data-lists/{id}
   :id: REQ_SOVD_028
   :status: open
   :tags: P

   **Goal:** Get data set

   **ROS 2 Equivalent:** Manage preset

   **Implementation Notes:** Return the preset definition; can be integrated with rosbag2 recording.

.. req:: DELETE /{entity}/data-lists/{id}
   :id: REQ_SOVD_029
   :status: open
   :tags: P

   **Goal:** Delete data set

   **ROS 2 Equivalent:** Manage preset

   **Implementation Notes:** Remove the preset definition from configuration or DB.

.. req:: GET /{entity}/cyclic-subscriptions
   :id: REQ_SOVD_030
   :status: open
   :tags: P

   **Goal:** List cyclic subscriptions

   **ROS 2 Equivalent:** Subscription manager in the gateway

   **Implementation Notes:** Maintain a registry of active subscriptions with rate and QoS configuration.

.. req:: GET /{entity}/cyclic-subscriptions/{id}
   :id: REQ_SOVD_031
   :status: open
   :tags: P

   **Goal:** Get cyclic subscription

   **ROS 2 Equivalent:** Subscription manager entry

   **Implementation Notes:** Return details of a specific cyclic subscription (topics, rate, QoS).

.. req:: PUT /{entity}/cyclic-subscriptions/{id}
   :id: REQ_SOVD_032
   :status: open
   :tags: P

   **Goal:** Modify cyclic subscription

   **ROS 2 Equivalent:** Subscription + downsampling

   **Implementation Notes:** Update rate, filters or output channel (SSE/WebSocket/gRPC).

.. req:: DELETE /{entity}/cyclic-subscriptions/{id}
   :id: REQ_SOVD_033
   :status: open
   :tags: P

   **Goal:** Delete cyclic subscription

   **ROS 2 Equivalent:** Subscription manager entry

   **Implementation Notes:** Stop and remove a cyclic subscription.

.. req:: GET /{entity}/bulk-data
   :id: REQ_SOVD_076
   :status: open
   :tags: A/P

   **Goal:** Bulk data overview

   **ROS 2 Equivalent:** rosbag2, maps, models

   **Implementation Notes:** Expose categories of large artifacts (bags, maps, ML models) per entity.

.. req:: GET /{entity}/bulk-data/{id}
   :id: REQ_SOVD_077
   :status: open
   :tags: A/P

   **Goal:** Get bulk data item

   **ROS 2 Equivalent:** Download artifact

   **Implementation Notes:** Provide download links or streaming for a specific artifact.

.. req:: PUT /{entity}/bulk-data/{id}
   :id: REQ_SOVD_078
   :status: open
   :tags: P

   **Goal:** Upload/replace bulk data item

   **ROS 2 Equivalent:** Upload artifact

   **Implementation Notes:** Upload new artifacts and register them under an ID.

.. req:: DELETE /{entity}/bulk-data/{id}
   :id: REQ_SOVD_079
   :status: open
   :tags: P

   **Goal:** Delete bulk data item

   **ROS 2 Equivalent:** Remove artifact

   **Implementation Notes:** Delete an artifact and its metadata.

