Discovery
=========

.. req:: GET /version-info
   :id: REQ_INTEROP_001
   :status: verified
   :tags: Discovery

   The endpoint shall provide version information for the SOVD server and its implementation.

.. req:: GET /{any}/docs
   :id: REQ_INTEROP_002
   :status: verified
   :tags: Discovery

   The endpoint shall provide human- and machine-readable documentation of the addressed resource and its capabilities.

.. req:: GET /{entity-collection}
   :id: REQ_INTEROP_003
   :status: verified
   :tags: Discovery

   The endpoint shall list all entities of the requested collection together with their basic metadata.

.. req:: GET /areas/{id}/subareas
   :id: REQ_INTEROP_004
   :status: verified
   :tags: Discovery

   The endpoint shall return the list of subareas that are contained in the addressed area.

.. req:: GET /components/{id}/subcomponents
   :id: REQ_INTEROP_005
   :status: verified
   :tags: Discovery

   The endpoint shall return the list of subcomponents that are logically contained in the addressed component.

.. req:: GET /areas/{id}/contains
   :id: REQ_INTEROP_006
   :status: verified
   :tags: Discovery

   The endpoint shall return all entities that are contained in the addressed area.

.. req:: GET /components/{id}/hosts
   :id: REQ_INTEROP_007
   :status: verified
   :tags: Discovery

   The endpoint shall list all applications that are hosted on the addressed component.

.. req:: GET /components/{id}/depends-on
   :id: REQ_INTEROP_008
   :status: implemented
   :tags: Discovery

   The endpoint shall return the list of components that the addressed component depends on.

.. req:: GET /apps/{id}/depends-on
   :id: REQ_INTEROP_009
   :status: verified
   :tags: Discovery

   The endpoint shall return the list of other entities that the addressed application depends on.

.. req:: GET /
   :id: REQ_INTEROP_010
   :status: verified
   :tags: Discovery

   The endpoint shall provide a summary of the SOVD server capabilities and entry points.

.. req:: tags=... (query)
   :id: REQ_INTEROP_011
   :status: verified
   :tags: Discovery

   The server shall support tag-based query parameters that filter discovery responses by tags.

.. req:: Topic beacon enriches entities
   :id: REQ_DISCO_BEACON_01
   :status: verified
   :tags: Discovery, Beacon

   The TopicBeaconPlugin shall enrich discovered entities with metadata from MedkitDiscoveryHint messages published to a configurable ROS 2 topic.

.. req:: Parameter beacon enriches entities
   :id: REQ_DISCO_BEACON_02
   :status: verified
   :tags: Discovery, Beacon

   The ParameterBeaconPlugin shall enrich discovered entities by polling node parameters matching a configurable prefix.

.. req:: Beacon hint lifecycle
   :id: REQ_DISCO_BEACON_03
   :status: verified
   :tags: Discovery, Beacon

   Beacon hints shall follow an ACTIVE, STALE, EXPIRED lifecycle based on configurable TTL and expiry durations.

.. req:: Vendor endpoints expose beacon data
   :id: REQ_DISCO_BEACON_04
   :status: verified
   :tags: Discovery, Beacon

   Each beacon plugin shall register vendor extension endpoints that return per-entity beacon metadata in JSON format.

.. req:: Input validation rejects malformed hints
   :id: REQ_DISCO_BEACON_05
   :status: verified
   :tags: Discovery, Beacon

   The beacon validator shall reject hints with empty entity IDs, oversized fields, or excessive metadata entries.
