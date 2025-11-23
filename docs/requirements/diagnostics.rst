Diagnostics & Logs
==================

.. req:: GET /{entity}/faults
   :id: REQ_SOVD_017
   :status: open
   :tags: A

   **Goal:** Read faults

   **ROS 2 Equivalent:** diagnostic_msgs/DiagnosticArray

   **Implementation Notes:** Collect current diagnostic state from /diagnostics (or diagnostic_aggregator) and map to entities.

.. req:: GET /{entity}/faults/{code}
   :id: REQ_SOVD_018
   :status: open
   :tags: A

   **Goal:** Fault details

   **ROS 2 Equivalent:** Filtered diagnostic entry

   **Implementation Notes:** Filter diagnostics by hardware_id, name or status code.

.. req:: DELETE /{entity}/faults
   :id: REQ_SOVD_019
   :status: open
   :tags: P

   **Goal:** Clear all faults

   **ROS 2 Equivalent:** Clear-faults service

   **Implementation Notes:** Define a ClearFaults service per node/app to reset its diagnostic state.

.. req:: DELETE /{entity}/faults/{code}
   :id: REQ_SOVD_020
   :status: open
   :tags: P

   **Goal:** Clear specific fault

   **ROS 2 Equivalent:** Clear-faults service with code parameter

   **Implementation Notes:** Implementation-specific per node vendor; service takes a fault code.

.. req:: GET /{entity}/logs
   :id: REQ_SOVD_066
   :status: open
   :tags: A

   **Goal:** Log overview

   **ROS 2 Equivalent:** /rosout snapshot

   **Implementation Notes:** Buffer /rosout and filter by time and severity for the given entity.

.. req:: GET /{entity}/logs/entries
   :id: REQ_SOVD_067
   :status: open
   :tags: A

   **Goal:** Paged log entries

   **ROS 2 Equivalent:** /rosout stream or pages

   **Implementation Notes:** Store logs in a backend (e.g. SQLite) and expose them with pagination.

.. req:: GET /{entity}/logs/config
   :id: REQ_SOVD_068
   :status: open
   :tags: A/P

   **Goal:** Read logging configuration

   **ROS 2 Equivalent:** set_logger_level / sinks

   **Implementation Notes:** Expose current logger levels and registered sinks for the entity.

.. req:: PUT /{entity}/logs/config
   :id: REQ_SOVD_069
   :status: open
   :tags: A/P

   **Goal:** Update logging configuration

   **ROS 2 Equivalent:** set_logger_level / sinks

   **Implementation Notes:** Dynamically change logger levels and sinks via ROS 2 logging APIs.

.. req:: DELETE /{entity}/logs/config
   :id: REQ_SOVD_070
   :status: open
   :tags: P

   **Goal:** Reset logging configuration

   **ROS 2 Equivalent:** Restore default logging config

   **Implementation Notes:** Reset logger levels and sinks back to defaults.

.. req:: GET /{entity}/communication-logs
   :id: REQ_SOVD_071
   :status: open
   :tags: P

   **Goal:** Communication logs

   **ROS 2 Equivalent:** DDS / pcap traces

   **Implementation Notes:** Integrate with DDS or network tracing tools (Cyclone DDS, Fast DDS, pcap).

.. req:: POST /{entity}/communication-logs
   :id: REQ_SOVD_072
   :status: open
   :tags: P

   **Goal:** Start communication logging

   **ROS 2 Equivalent:** Enable capture

   **Implementation Notes:** Start capturing DDS or network traffic for the entity.

.. req:: GET /{entity}/communication-logs/{id}
   :id: REQ_SOVD_073
   :status: open
   :tags: P

   **Goal:** Get communication log artifact

   **ROS 2 Equivalent:** Trace artifact

   **Implementation Notes:** Return a pcap/pcapng file location plus metadata.

.. req:: PUT /{entity}/communication-logs/{id}
   :id: REQ_SOVD_074
   :status: open
   :tags: P

   **Goal:** Configure communication logging

   **ROS 2 Equivalent:** Trace configuration

   **Implementation Notes:** Update filters, duration or capture scope for an existing trace job.

.. req:: DELETE /{entity}/communication-logs/{id}
   :id: REQ_SOVD_075
   :status: open
   :tags: P

   **Goal:** Delete communication log artifact

   **ROS 2 Equivalent:** Trace artifact cleanup

   **Implementation Notes:** Delete stored trace artifacts and associated metadata.

