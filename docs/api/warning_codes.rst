Warning Codes
=============

The ``/api/v1/health`` endpoint surfaces operator-actionable aggregation
anomalies in the top-level ``warnings`` array (x-medkit extension).
Each entry has the shape:

.. code-block:: json

   {
     "code": "leaf_id_collision",
     "message": "Component 'ecu-x' is announced by multiple peers (peer_b, peer_c); routing falls back to last-writer-wins which is non-deterministic. Resolve by renaming the Component on one side or by modelling it as a hierarchical parent (declare a child Component with parentComponentId='ecu-x' on the owning peer).",
     "entity_ids": ["ecu-x"],
     "peer_names": ["peer_b", "peer_c"]
   }

Codes are stable machine-readable identifiers: renaming a code is a
breaking change for downstream consumers that key on the string.

The canonical list of codes is maintained in
``src/ros2_medkit_gateway/include/ros2_medkit_gateway/http/warning_codes.hpp``;
this page mirrors it for API consumers.

.. list-table::
   :header-rows: 1
   :widths: 25 75

   * - Code
     - Meaning / Remediation
   * - ``leaf_id_collision``
     - More than one peer announces the same **leaf** (non-hierarchical)
       Component ID during aggregation merge. Routing falls back to
       last-writer-wins, so requests for the affected Component reach one
       peer non-deterministically. Resolve by renaming the Component on one
       side, or by modelling it as a hierarchical parent - declare a child
       Component with ``parentComponentId`` pointing at the colliding ID on
       the owning peer. The warning lists every claiming peer in
       ``peer_names``.

When aggregation is enabled (``GET /`` -> ``capabilities.aggregation``
is ``true``), ``warnings`` is always an array on the ``/health`` response:
empty when no anomalies are active, non-empty otherwise. When aggregation
is disabled, the field is omitted entirely.
