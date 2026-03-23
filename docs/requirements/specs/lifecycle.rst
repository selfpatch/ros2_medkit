Lifecycle
=========

.. req:: GET /{entity}/status
   :id: REQ_INTEROP_076
   :status: open
   :tags: Lifecycle

   The endpoint shall return the current lifecycle status of the addressed entity.

.. req:: PUT /{entity}/status/start
   :id: REQ_INTEROP_077
   :status: open
   :tags: Lifecycle

   The endpoint shall request that the addressed entity transitions into a running or started state.

.. req:: PUT /{entity}/status/restart
   :id: REQ_INTEROP_078
   :status: open
   :tags: Lifecycle

   The endpoint shall request a controlled restart of the addressed entity.

.. req:: PUT /{entity}/status/force-restart
   :id: REQ_INTEROP_079
   :status: open
   :tags: Lifecycle

   The endpoint shall request a forced restart of the addressed entity, bypassing normal restart procedures.

.. req:: PUT /{entity}/status/shutdown
   :id: REQ_INTEROP_080
   :status: open
   :tags: Lifecycle

   The endpoint shall request a controlled shutdown of the addressed entity.

.. req:: PUT /{entity}/status/force-shutdown
   :id: REQ_INTEROP_081
   :status: open
   :tags: Lifecycle

   The endpoint shall request a forced shutdown of the addressed entity, bypassing normal procedures.

