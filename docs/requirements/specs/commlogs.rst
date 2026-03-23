CommLogs
========

.. req:: GET /{entity}/communication-logs
   :id: REQ_INTEROP_066
   :status: open
   :tags: CommLogs

   The endpoint shall provide an overview of available communication log sessions on the addressed entity.

.. req:: POST /{entity}/communication-logs
   :id: REQ_INTEROP_067
   :status: open
   :tags: CommLogs

   The endpoint shall start a new communication logging session on the addressed entity.

.. req:: GET /{entity}/communication-logs/{id}
   :id: REQ_INTEROP_068
   :status: open
   :tags: CommLogs

   The endpoint shall return the artifact or metadata of the addressed communication log session.

.. req:: PUT /{entity}/communication-logs/{id}
   :id: REQ_INTEROP_069
   :status: open
   :tags: CommLogs

   The endpoint shall modify the configuration of the addressed communication log session, if supported.

.. req:: DELETE /{entity}/communication-logs/{id}
   :id: REQ_INTEROP_070
   :status: open
   :tags: CommLogs

   The endpoint shall delete the artifact or configuration of the addressed communication log session.

