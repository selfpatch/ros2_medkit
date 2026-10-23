#!/usr/bin/env bash
# Build the ROS-free cores on Ubuntu 20.04 / gcc 9 and run the check program.
# Sources go in as the build context: bind mounts of a worktree arrive empty on Docker Desktop.
set -uo pipefail
root="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$root" || exit 1
ctx=$(mktemp /tmp/no_ros_ctx.XXXXXX)
image=""
container=""
# shellcheck disable=SC2317
cleanup() {
  rm -f "$ctx"
  [ -n "$container" ] && docker rm -f "$container" >/dev/null 2>&1
  [ -n "$image" ] && docker rmi "$image" >/dev/null 2>&1
  return 0
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

tar -c src/ros2_medkit_fault_detection src/ros2_medkit_fault_reporter src/ros2_medkit_fault_manager scripts/check_cores_without_ros >"$ctx" || exit 1
image=$(docker build -q -f scripts/check_cores_without_ros/Dockerfile - <"$ctx") || exit 1
# Detached run: attached stdout can be cut short when the daemon is remote.
container=$(docker run -d "$image") || exit 1
rc=$(docker wait "$container") || rc=1
docker logs "$container" || true
exit "$rc"
