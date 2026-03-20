^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_cmake
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.0 (2026-03-20)
------------------
* Initial release - shared cmake modules extracted from gateway package (`#294 <https://github.com/selfpatch/ros2_medkit/pull/294>`_)
* ``ROS2MedkitCcache.cmake`` - auto-detect ccache for faster incremental rebuilds
* ``ROS2MedkitLinting.cmake`` - centralized clang-tidy configuration (opt-in locally, mandatory in CI)
* ``ROS2MedkitCompat.cmake`` - multi-distro compatibility shims for ROS 2 Humble/Jazzy/Rolling
* Contributors: @bburda
