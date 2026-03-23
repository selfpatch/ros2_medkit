^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_medkit_linux_introspection
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.0 (2026-03-20)
------------------
* Initial release - Linux process introspection plugins for ros2_medkit gateway
* ``procfs_plugin`` - process-level diagnostics via ``/proc`` filesystem (CPU, memory, threads, file descriptors)
* ``systemd_plugin`` - systemd unit status and resource usage via D-Bus
* ``container_plugin`` - container runtime detection and cgroup resource limits
* ``PidCache`` with TTL-based refresh for efficient PID-to-node mapping
* ``proc_reader`` and ``cgroup_reader`` utilities with configurable proc root
* Cross-distro support for ROS 2 Humble, Jazzy, and Rolling
* Contributors: @bburda
