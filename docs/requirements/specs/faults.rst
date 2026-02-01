Faults
======

.. req:: GET /{entity}/faults
   :id: REQ_INTEROP_012
   :status: verified
   :tags: Faults

   The endpoint shall return the list of diagnostic fault entries stored for the addressed entity, possibly including active and stored faults.

.. req:: GET /{entity}/faults/{code}
   :id: REQ_INTEROP_013
   :status: verified
   :tags: Faults

   The endpoint shall return detailed information for the addressed diagnostic fault code.

.. req:: DELETE /{entity}/faults
   :id: REQ_INTEROP_014
   :status: verified
   :tags: Faults

   The endpoint shall clear all diagnostic fault entries stored for the addressed entity, if permitted.

.. req:: DELETE /{entity}/faults/{code}
   :id: REQ_INTEROP_015
   :status: verified
   :tags: Faults

   The endpoint shall clear the addressed diagnostic fault code for the entity, if permitted.

.. req:: GET /{entity}/faults/{code}/snapshots
   :id: REQ_INTEROP_088
   :status: verified
   :tags: Faults, Snapshots

   The endpoint shall return topic data snapshots captured when the addressed fault transitioned to CONFIRMED status, enabling post-mortem debugging of system state at the time of fault occurrence.

   .. note::

      This endpoint corresponds to SOVD "environment data" concept. In SOVD terminology,
      environment data refers to system state captured at the time of fault occurrence
      (similar to UDS freeze frames). We use "snapshots" as ROS 2-specific terminology
      for consistency with the ROS 2 ecosystem.

