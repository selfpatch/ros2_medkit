#!/usr/bin/env bash
# Compile-only: does fault_storage.hpp build without any ROS include path?
# Exit 0 means it does. Run it in a shell where /opt/ros is not sourced.
set -u
root="$(cd "$(dirname "$0")/../.." && pwd)"
exec g++ -std=c++17 -fsyntax-only \
  -I"$root/src/ros2_medkit_fault_manager/include" \
  "$root/scripts/check_cores_without_ros/fault_storage_probe.cpp"
