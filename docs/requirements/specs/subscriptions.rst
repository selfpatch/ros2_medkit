Subscriptions
=============

.. req:: GET /{entity}/cyclic-subscriptions
   :id: REQ_INTEROP_025
   :status: open
   :tags: Subscriptions

   The endpoint shall list all configured cyclic data subscriptions on the addressed entity.

.. req:: GET /{entity}/cyclic-subscriptions/{id}
   :id: REQ_INTEROP_026
   :status: open
   :tags: Subscriptions

   The endpoint shall return the configuration and status of the addressed cyclic subscription.

.. req:: PUT /{entity}/cyclic-subscriptions/{id}
   :id: REQ_INTEROP_027
   :status: open
   :tags: Subscriptions

   The endpoint shall update the configuration of the addressed cyclic subscription on the entity.

.. req:: DELETE /{entity}/cyclic-subscriptions/{id}
   :id: REQ_INTEROP_028
   :status: open
   :tags: Subscriptions

   The endpoint shall delete the addressed cyclic subscription from the entity.

.. req:: POST /{entity}/cyclic-subscriptions
   :id: REQ_INTEROP_089
   :status: open
   :tags: Subscriptions

   The endpoint shall create a new cyclic data subscription on the addressed entity, specifying the observed resource URI, delivery interval (fast/normal/slow), transport protocol (SSE), and subscription duration in seconds. On success it shall return 201 with the subscription ID, observed resource, event source URI, protocol, and interval.

.. req:: GET /{entity}/cyclic-subscriptions/{id}/events
   :id: REQ_INTEROP_090
   :status: open
   :tags: Subscriptions

   The endpoint shall open an SSE (Server-Sent Events) stream that delivers the current value of the subscribed data resource at the configured interval. Each event shall be an EventEnvelope containing a UTC timestamp and either a payload (ReadValue format) or an error (GenericError format). The stream shall auto-close when the subscription duration expires, the client disconnects, or the subscription is deleted.

.. req:: POST /{entity}/triggers
   :id: REQ_INTEROP_029
   :status: open
   :tags: Subscriptions

   The endpoint shall create a new trigger definition on the addressed entity.

.. req:: GET /{entity}/triggers
   :id: REQ_INTEROP_030
   :status: open
   :tags: Subscriptions

   The endpoint shall list all trigger definitions configured on the addressed entity.

.. req:: PUT /{entity}/triggers/{id}
   :id: REQ_INTEROP_031
   :status: open
   :tags: Subscriptions

   The endpoint shall update the addressed trigger definition on the entity.

.. req:: DELETE /{entity}/triggers/{id}
   :id: REQ_INTEROP_032
   :status: open
   :tags: Subscriptions

   The endpoint shall delete the addressed trigger definition from the entity.
