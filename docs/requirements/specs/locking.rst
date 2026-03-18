Locking
=======

.. req:: POST /{entity}/locks
   :id: REQ_INTEROP_100
   :status: verified
   :tags: Locking

   The endpoint shall create a new lock on the addressed entity with mandatory ``lock_expiration`` (seconds), optional ``scopes`` (collections to lock), and optional ``break_lock`` flag. Response ``201`` with LockInfo (id, owned, scopes, lock_expiration).

.. req:: GET /{entity}/locks
   :id: REQ_INTEROP_101
   :status: verified
   :tags: Locking

   The endpoint shall list all active locks on the addressed entity. Response ``200`` with ``{ "items": LockInfo[] }``.

.. req:: GET /{entity}/locks/{lock-id}
   :id: REQ_INTEROP_102
   :status: verified
   :tags: Locking

   The endpoint shall return details of the addressed lock including ``lock_expiration`` (date-time), ``owned``, and ``scopes``. Response ``200`` with LockInfo.

.. req:: PUT /{entity}/locks/{lock-id}
   :id: REQ_INTEROP_103
   :status: verified
   :tags: Locking

   The endpoint shall update the ``lock_expiration`` of the addressed lock. Response ``204``.

.. req:: DELETE /{entity}/locks/{lock-id}
   :id: REQ_INTEROP_104
   :status: verified
   :tags: Locking

   The endpoint shall release the addressed lock. Response ``204``.
