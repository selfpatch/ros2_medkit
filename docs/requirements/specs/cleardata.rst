ClearData
=========

.. req:: GET /{entity}/clear-data
   :id: REQ_INTEROP_056
   :status: open
   :tags: ClearData

   The endpoint shall provide an overview of the available clear-data options for the addressed entity.

.. req:: PUT /{entity}/clear-data/cached-data
   :id: REQ_INTEROP_057
   :status: open
   :tags: ClearData

   The endpoint shall clear cached diagnostic data on the addressed entity.

.. req:: PUT /{entity}/clear-data/learned-data
   :id: REQ_INTEROP_058
   :status: open
   :tags: ClearData

   The endpoint shall clear learned or adapted data on the addressed entity.

.. req:: PUT /{entity}/clear-data/client-defined-resources
   :id: REQ_INTEROP_059
   :status: open
   :tags: ClearData

   The endpoint shall clear client-defined diagnostic resources on the addressed entity.

.. req:: GET /{entity}/clear-data/status
   :id: REQ_INTEROP_060
   :status: open
   :tags: ClearData

   The endpoint shall return the status of the most recent clear-data operation on the entity.

