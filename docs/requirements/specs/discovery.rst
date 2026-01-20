Discovery
=========

.. req:: GET /version-info
   :id: REQ_INTEROP_001
   :status: open
   :tags: Discovery

   The endpoint shall provide version information for the SOVD server and its implementation.

.. req:: GET /{any}/docs
   :id: REQ_INTEROP_002
   :status: open
   :tags: Discovery

   The endpoint shall provide human- and machine-readable documentation of the addressed resource and its capabilities.

.. req:: GET /{entity-collection}
   :id: REQ_INTEROP_003
   :status: open
   :tags: Discovery

   The endpoint shall list all entities of the requested collection together with their basic metadata.

.. req:: GET /areas/{id}/subareas
   :id: REQ_INTEROP_004
   :status: open
   :tags: Discovery

   The endpoint shall return the list of subareas that are contained in the addressed area.

.. req:: GET /components/{id}/subcomponents
   :id: REQ_INTEROP_005
   :status: open
   :tags: Discovery

   The endpoint shall return the list of subcomponents that are logically contained in the addressed component.

.. req:: GET /areas/{id}/contains
   :id: REQ_INTEROP_006
   :status: open
   :tags: Discovery

   The endpoint shall return all entities that are contained in the addressed area.

.. req:: GET /components/{id}/hosts
   :id: REQ_INTEROP_007
   :status: open
   :tags: Discovery

   The endpoint shall list all applications that are hosted on the addressed component.

.. req:: GET /components/{id}/depends-on
   :id: REQ_INTEROP_008
   :status: implemented
   :tags: Discovery

   The endpoint shall return the list of components that the addressed component depends on.

.. req:: GET /apps/{id}/depends-on
   :id: REQ_INTEROP_009
   :status: open
   :tags: Discovery

   The endpoint shall return the list of other entities that the addressed application depends on.

.. req:: GET /
   :id: REQ_INTEROP_010
   :status: open
   :tags: Discovery

   The endpoint shall provide a summary of the SOVD server capabilities and entry points.

.. req:: tags=... (query)
   :id: REQ_INTEROP_011
   :status: open
   :tags: Discovery

   The server shall support tag-based query parameters that filter discovery responses by tags.

