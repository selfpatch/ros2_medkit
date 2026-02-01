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

