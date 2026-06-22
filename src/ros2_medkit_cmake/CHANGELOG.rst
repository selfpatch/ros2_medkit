^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_cmake
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.0 (2026-06-22)
------------------
* ``ROS2MedkitTestDomain``: carve dedicated ``ROS_DOMAIN_ID`` ranges for the new log bridge (210-214) and action-status bridge (215-219) test suites out of the integration-tests range (`#422 <https://github.com/selfpatch/ros2_medkit/pull/422>`_)
* Contributors: @mfaferek93

0.5.0 (2026-06-08)
------------------
* ``ROS2MedkitWarnings.cmake`` module centralizes compiler warning flags across all packages, with selective ``-Werror`` (namespaced ``MEDKIT_ENABLE_WERROR``, defaults OFF) applied only to flags safe against external headers
* ``ROS2MedkitSanitizers.cmake`` module adds ASan/UBSan and TSan support for sanitizer CI jobs
* ``ROS2MedkitTestDomain.cmake`` centralizes ``ROS_DOMAIN_ID`` allocation for per-test DDS isolation
* Vendored cpp-httplib 0.14.3 as a build-farm fallback (``VENDORED_DIR`` parameter), marked as a SYSTEM include to suppress third-party warnings
* ``medkit_find_cpp_httplib`` caps cpp-httplib at ``< 0.20`` across both the pkg-config and ``find_package(httplib)`` tiers, so distros shipping 0.20+ (Ubuntu 26.04 ships 0.26, which dropped the multipart ``Request::has_file`` API the gateway uses) fall through to the vendored 0.14.3 header instead of failing the build; ``ROS2MedkitCompat.cmake`` extended to cover ROS 2 Lyrical / Ubuntu 26.04 Resolute (rclcpp 32, ``yaml_cpp_vendor`` target export, ``ament_target_dependencies`` removal in ament_cmake 2.8.5+), which replaces Rolling in CI (`#405 <https://github.com/selfpatch/ros2_medkit/pull/405>`_)
* Contributors: @bburda, @mfaferek93

0.4.0 (2026-03-20)
------------------
* Initial release - shared cmake modules extracted from gateway package (`#294 <https://github.com/selfpatch/ros2_medkit/pull/294>`_)
* ``ROS2MedkitCcache.cmake`` - auto-detect ccache for faster incremental rebuilds
* ``ROS2MedkitLinting.cmake`` - centralized clang-tidy configuration (opt-in locally, mandatory in CI)
* ``ROS2MedkitCompat.cmake`` - multi-distro compatibility shims for ROS 2 Humble/Jazzy/Rolling
* Contributors: @bburda
