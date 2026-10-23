#!/usr/bin/env bash
# Build the ROS-free cores on Ubuntu 20.04 / gcc 9 and run the check program.
# Sources go in as the build context: bind mounts of a worktree arrive empty on Docker Desktop.
set -euo pipefail
root="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$root"
tar -c src/ros2_medkit_fault_detection src/ros2_medkit_fault_reporter src/ros2_medkit_fault_manager scripts/check_cores_without_ros >/tmp/no_ros_ctx.$$.tar
image=$(docker build -q -f scripts/check_cores_without_ros/Dockerfile - </tmp/no_ros_ctx.$$.tar)
rm -f /tmp/no_ros_ctx.$$.tar
rc=0
docker run --rm "$image" || rc=$?
docker rmi "$image" >/dev/null 2>&1 || true
exit "$rc"
