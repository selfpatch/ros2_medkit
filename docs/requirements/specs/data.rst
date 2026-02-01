Data
====

.. req:: GET /{entity}/data-categories
   :id: REQ_INTEROP_016
   :status: open
   :tags: Data

   The endpoint shall provide metadata about available data categories on the addressed entity.

.. req:: GET /{entity}/data-groups
   :id: REQ_INTEROP_017
   :status: open
   :tags: Data

   The endpoint shall provide metadata about defined data or signal groups on the addressed entity.

.. req:: GET /{entity}/data
   :id: REQ_INTEROP_018
   :status: verified
   :tags: Data

   The endpoint shall return a snapshot of selected data items for the addressed entity.

.. req:: GET /{entity}/data/{data-id}
   :id: REQ_INTEROP_019
   :status: verified
   :tags: Data

   The endpoint shall read the current value of the addressed data item from the entity.

.. req:: PUT /{entity}/data/{data-id}
   :id: REQ_INTEROP_020
   :status: verified
   :tags: Data

   The endpoint shall write a new value to the addressed data item on the entity, if it is writable.

