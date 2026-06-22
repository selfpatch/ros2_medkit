^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_topic_beacon
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.0 (2026-06-22)
------------------
* No functional changes; version bump for the coordinated 0.6.0 release.
* Contributors: @bburda

0.5.0 (2026-06-08)
------------------
* Migrated ``TopicBeaconPlugin`` to the ``get_routes()`` plugin API
* Added shutdown guards and ``noexcept`` destructors (with ``override``) that reset rclcpp resources before member destruction, preventing teardown SIGSEGV
* Added post-shutdown guard unit tests
* Build: adopt the centralized ``ROS2MedkitWarnings`` cmake module
* Contributors: @bburda

0.4.0 (2026-03-20)
------------------
* Initial release - topic-based beacon discovery plugin
* ``TopicBeaconPlugin`` with push-based topic subscription for entity enrichment
* ``x-medkit-topic-beacon`` vendor extension REST endpoint
* Stamp-based TTL for topic beacon hints
* Diagnostic logging for beacon hint processing
* Contributors: @bburda
