System Management
=================

.. req:: GET /updates
   :id: REQ_SOVD_086
   :status: open
   :tags: P

   **Goal:** List updates

   **ROS 2 Equivalent:** Package/container manager

   **Implementation Notes:** Integrate with package or container managers to list available updates.

.. req:: POST /updates
   :id: REQ_SOVD_087
   :status: open
   :tags: P

   **Goal:** Create update job

   **ROS 2 Equivalent:** Package/container manager

   **Implementation Notes:** Create an update plan (artifacts, targets, timing).

.. req:: PUT /updates/{id}
   :id: REQ_SOVD_088
   :status: open
   :tags: P

   **Goal:** Modify update job

   **ROS 2 Equivalent:** Package/container manager

   **Implementation Notes:** Change rollout parameters, targets or timing for an update job.

.. req:: DELETE /updates/{id}
   :id: REQ_SOVD_089
   :status: open
   :tags: P

   **Goal:** Cancel update job

   **ROS 2 Equivalent:** Package/container manager

   **Implementation Notes:** Cancel a planned or running update job if still possible.

.. req:: GET /updates/{id}
   :id: REQ_SOVD_090
   :status: open
   :tags: P

   **Goal:** Get update job status

   **ROS 2 Equivalent:** OTA progress

   **Implementation Notes:** Report progress and result for a specific update job.

.. req:: POST /authorize
   :id: REQ_SOVD_091
   :status: open
   :tags: P

   **Goal:** Authorization

   **ROS 2 Equivalent:** JWT/OAuth2 at gateway + SROS2

   **Implementation Notes:** Gateway validates tokens; SROS2 enforces DDS security policies.

.. req:: POST /token
   :id: REQ_SOVD_092
   :status: open
   :tags: P

   **Goal:** Token issuance

   **ROS 2 Equivalent:** JWT/OAuth2 at gateway

   **Implementation Notes:** Issue or refresh tokens according to the chosen auth flow.

.. needtable::
   :columns: id, title, status, tags

