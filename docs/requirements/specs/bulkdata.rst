BulkData
========

.. req:: GET /{entity}/bulk-data
   :id: REQ_INTEROP_071
   :status: verified
   :tags: BulkData, SOVD

   The endpoint shall provide an overview of bulk-data categories supported by the addressed entity.

   Gateway supports all entity types: apps, components, areas, functions, and nested entities.
   Currently implemented category: ``rosbags``.

.. req:: GET /{entity}/bulk-data/{category}
   :id: REQ_INTEROP_072
   :status: verified
   :tags: BulkData, SOVD

   The endpoint shall list all bulk-data items available in the addressed bulk-data category on the entity.

   Returns BulkDataDescriptor array with id, name, mimetype, size, creation_date fields.
   Includes ``x-medkit`` extensions with fault_code, duration_sec, format.

.. req:: GET /{entity}/bulk-data/{category}/{bulk-data-id}
   :id: REQ_INTEROP_073
   :status: verified
   :tags: BulkData, SOVD

   The endpoint shall return the content of the addressed bulk-data item or its access information (e.g. download location).

   Returns binary file content with appropriate Content-Type (application/x-mcap or application/x-sqlite3)
   and Content-Disposition headers for browser download.

.. req:: POST /{entity}/bulk-data/{category}
   :id: REQ_INTEROP_074
   :status: verified
   :tags: BulkData, SOVD
   :links: REQ_INTEROP_071

   The endpoint shall upload new bulk data in the addressed category and create a
   corresponding bulk-data resource on the entity.

   Accepts multipart/form-data with a required ``file`` field and optional ``description``
   and ``metadata`` fields. Returns 201 Created with a Location header pointing to the new
   resource. Supported for components and apps. The ``rosbags`` category is read-only.

.. req:: DELETE /{entity}/bulk-data/{category}/{bulk-data-id}
   :id: REQ_INTEROP_075
   :status: verified
   :tags: BulkData, SOVD
   :links: REQ_INTEROP_071

   The endpoint shall delete the addressed bulk-data item from the entity, if permitted.

   Returns 204 No Content on success. Supported for components and apps.
   The ``rosbags`` category is managed by the fault manager and cannot be deleted.

