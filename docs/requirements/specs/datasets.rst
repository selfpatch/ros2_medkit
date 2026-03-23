DataSets
========

.. req:: GET /{entity}/data-lists
   :id: REQ_INTEROP_021
   :status: verified
   :tags: DataSets

   The endpoint shall list all defined data sets (data-lists) available on the addressed entity.

.. req:: POST /{entity}/data-lists
   :id: REQ_INTEROP_022
   :status: verified
   :tags: DataSets

   The endpoint shall create a new temporary data set definition used to read multiple data items with a single request on the addressed entity.

.. req:: GET /{entity}/data-lists/{id}
   :id: REQ_INTEROP_023
   :status: verified
   :tags: DataSets

   The endpoint shall return the current values of all data items referenced by the addressed data set (data-list).

.. req:: DELETE /{entity}/data-lists/{id}
   :id: REQ_INTEROP_024
   :status: verified
   :tags: DataSets

   The endpoint shall delete the addressed data set definition from the entity.

