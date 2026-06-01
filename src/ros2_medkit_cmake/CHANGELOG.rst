^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_cmake
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.0 (2026-06-01)
------------------
* ``ROS2MedkitWarnings.cmake`` module centralizes compiler warning flags across all packages, with selective ``-Werror`` (namespaced ``MEDKIT_ENABLE_WERROR``, defaults OFF) applied only to flags safe against external headers
* ``ROS2MedkitSanitizers.cmake`` module adds ASan/UBSan and TSan support for sanitizer CI jobs
* ``ROS2MedkitTestDomain.cmake`` centralizes ``ROS_DOMAIN_ID`` allocation for per-test DDS isolation
* Vendored cpp-httplib 0.14.3 as a build-farm fallback (``VENDORED_DIR`` parameter), marked as a SYSTEM include to suppress third-party warnings
* Contributors: @bburda, @mfaferek93

0.4.0 (2026-03-20)
------------------
* Initial release - shared cmake modules extracted from gateway package (`#294 <https://github.com/selfpatch/ros2_medkit/pull/294>`_)
* ``ROS2MedkitCcache.cmake`` - auto-detect ccache for faster incremental rebuilds
* ``ROS2MedkitLinting.cmake`` - centralized clang-tidy configuration (opt-in locally, mandatory in CI)
* ``ROS2MedkitCompat.cmake`` - multi-distro compatibility shims for ROS 2 Humble/Jazzy/Rolling
* Contributors: @bburda
