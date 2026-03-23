Logs
====

.. req:: GET /{entity}/logs
   :id: REQ_INTEROP_061
   :status: verified
   :tags: Logs

   The endpoint shall return log entries for the addressed entity, optionally filtered by
   severity or context identifier. *(ISO 17978-3 §7.21)*

.. req:: GET /{entity}/logs/configuration
   :id: REQ_INTEROP_063
   :status: verified
   :tags: Logs

   The endpoint shall return the current logging configuration of the addressed entity.

.. req:: PUT /{entity}/logs/configuration
   :id: REQ_INTEROP_064
   :status: verified
   :tags: Logs

   The endpoint shall update the logging configuration of the addressed entity.
