Locking API
===========

SOVD-compliant resource locking for components and apps (ISO 17978-3, Section 7.17).
Locks prevent concurrent modification of entity resources by multiple clients.

.. contents:: Table of Contents
   :local:
   :depth: 2

Overview
--------

Locking provides mutual exclusion for entity resource collections. A client acquires a lock
on a component or app, specifying which resource collections (scopes) are protected. Other
clients are blocked from modifying locked collections until the lock is released or expires.

Key concepts:

- **Client identification**: Clients use the ``X-Client-Id`` header (client-generated UUID)
- **Scoped locks**: Locks can protect specific collections (e.g., ``configurations``, ``operations``)
  or all collections when no scopes are specified
- **Parent propagation**: A lock on a component also protects its child apps (lazy check)
- **Lock breaking**: Privileged clients can break existing locks with ``break_lock: true``
- **Automatic expiry**: Locks expire after the specified TTL and are cleaned up periodically

Configuration
-------------

Enable locking in ``gateway_params.yaml``:

.. code-block:: yaml

   locking:
     enabled: true
     default_max_expiration: 3600
     cleanup_interval: 30
     defaults:
       components:
         lock_required_scopes: [configurations, operations]
         breakable: true
       apps:
         lock_required_scopes: [configurations]
         breakable: true

Per-entity overrides in the manifest:

.. code-block:: yaml

   components:
     - id: safety_controller
       name: Safety Controller
       lock:
         required_scopes: [configurations, operations, data]
         breakable: false
         max_expiration: 7200

Endpoints
---------

Acquire Lock
~~~~~~~~~~~~

``POST /api/v1/{components|apps}/{entity_id}/locks``

Acquire a lock on an entity. Requires ``X-Client-Id`` header.

**Request Body:**

.. code-block:: json

   {
     "lock_expiration": 300,
     "scopes": ["configurations", "operations"],
     "break_lock": false
   }

- ``lock_expiration`` (required): Lock TTL in seconds
- ``scopes`` (optional): Resource collections to lock. If omitted, all collections are locked.
  Valid scopes: ``data``, ``operations``, ``configurations``, ``faults``, ``bulk-data``, ``modes``, ``scripts``
- ``break_lock`` (optional, default ``false``): If true, replaces any existing lock

**Response (201 Created):**

.. code-block:: json

   {
     "id": "lock_1",
     "owned": true,
     "scopes": ["configurations", "operations"],
     "lock_expiration": "2026-03-18T21:30:00Z"
   }

List Locks
~~~~~~~~~~

``GET /api/v1/{components|apps}/{entity_id}/locks``

List locks on an entity. ``X-Client-Id`` header is optional (determines ``owned`` field).

**Response (200 OK):**

.. code-block:: json

   {
     "items": [
       {
         "id": "lock_1",
         "owned": true,
         "scopes": ["configurations"],
         "lock_expiration": "2026-03-18T21:30:00Z"
       }
     ]
   }

Get Lock Details
~~~~~~~~~~~~~~~~

``GET /api/v1/{components|apps}/{entity_id}/locks/{lock_id}``

Get details of a specific lock. Returns 404 if not found.

Extend Lock
~~~~~~~~~~~

``PUT /api/v1/{components|apps}/{entity_id}/locks/{lock_id}``

Extend a lock's expiration. Requires ``X-Client-Id`` header (must be lock owner).

**Request Body:**

.. code-block:: json

   {
     "lock_expiration": 600
   }

**Response:** 204 No Content

Release Lock
~~~~~~~~~~~~

``DELETE /api/v1/{components|apps}/{entity_id}/locks/{lock_id}``

Release a lock. Requires ``X-Client-Id`` header (must be lock owner).

**Response:** 204 No Content

Error Responses
---------------

.. list-table::
   :header-rows: 1

   * - Status
     - Error Code
     - Condition
   * - 400
     - ``invalid-parameter``
     - Missing ``X-Client-Id``, invalid ``lock_expiration``, unknown scope
   * - 403
     - ``forbidden``
     - Trying to extend/release a lock owned by another client
   * - 404
     - ``resource-not-found``
     - Lock or entity not found
   * - 409
     - ``invalid-request``
     - Entity already locked by another client (without ``break_lock``)
   * - 409
     - ``lock-broken``
     - Mutating operation blocked by another client's lock
   * - 501
     - ``not-implemented``
     - Locking is disabled on this gateway
