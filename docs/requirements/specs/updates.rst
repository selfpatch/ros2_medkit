Updates
=======

.. req:: GET /updates
   :id: REQ_INTEROP_082
   :status: verified
   :tags: Updates

   The endpoint shall list software update packages known to the SOVD server.

.. req:: POST /updates
   :id: REQ_INTEROP_083
   :status: verified
   :tags: Updates

   The endpoint shall register a new software update package at the SOVD server using the provided parameters.

.. req:: DELETE /updates/{id}
   :id: REQ_INTEROP_084
   :status: verified
   :tags: Updates

   The endpoint shall delete the addressed software update package from the SOVD server, if permitted (e.g. if no update is currently in execution).

.. req:: GET /updates/{id}
   :id: REQ_INTEROP_085
   :status: verified
   :tags: Updates

   The endpoint shall return the details of the addressed software update package.

.. req:: PUT /updates/{id}/prepare
   :id: REQ_INTEROP_091
   :status: verified
   :tags: Updates

   The endpoint shall trigger preparation of the addressed software update package (download, integrity check, dependency validation) and return 202 Accepted with a Location header pointing to the status resource.

.. req:: PUT /updates/{id}/execute
   :id: REQ_INTEROP_092
   :status: verified
   :tags: Updates

   The endpoint shall begin the actual installation of a previously prepared software update package and return 202 Accepted with a Location header pointing to the status resource.

.. req:: PUT /updates/{id}/automated
   :id: REQ_INTEROP_093
   :status: verified
   :tags: Updates

   The endpoint shall trigger a fully automated update (prepare + execute) for packages where automated mode is supported and return 202 Accepted with a Location header pointing to the status resource.

.. req:: GET /updates/{id}/status
   :id: REQ_INTEROP_094
   :status: verified
   :tags: Updates

   The endpoint shall return the current progress and status of an update operation, including status value, progress percentage, and optional sub-progress details.
