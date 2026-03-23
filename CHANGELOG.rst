^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_beacon_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.0 (2026-03-20)
------------------
* Initial release - shared utilities for beacon discovery plugins
* ``BeaconHintStore`` with TTL-based hint transitions and thread safety
* ``BeaconValidator`` input validation gate
* ``BeaconEntityMapper`` to convert discovery hints into ``IntrospectionResult``
* ``build_beacon_response()`` shared response builder
* ``BeaconPlugin`` base class for topic and parameter beacon plugins
* ``TokenBucket`` rate limiter with thread-safe access
* Contributors: @bburda
