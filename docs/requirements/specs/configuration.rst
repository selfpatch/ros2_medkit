Configuration
=============

.. req:: GET /{entity}/configurations
   :id: REQ_INTEROP_048
   :status: verified
   :tags: Configuration

   The endpoint shall list configuration snapshots or items stored on the addressed entity.

.. req:: GET /{entity}/configurations/{id}
   :id: REQ_INTEROP_049
   :status: verified
   :tags: Configuration

   The endpoint shall return the content of the addressed configuration snapshot or item.

.. req:: PUT /{entity}/configurations/{id}
   :id: REQ_INTEROP_050
   :status: verified
   :tags: Configuration

   The endpoint shall write configuration data for the addressed configuration resource on the entity, applying the provided content.

.. req:: DELETE /{entity}/configurations
   :id: REQ_INTEROP_051
   :status: verified
   :tags: Configuration

   The endpoint shall reset all configuration resources of the addressed entity to their default values, if permitted.

.. req:: DELETE /{entity}/configurations/{id}
   :id: REQ_INTEROP_052
   :status: verified
   :tags: Configuration

   The endpoint shall reset the addressed configuration resource to its default value, if permitted.

