#!/usr/bin/env bash
# Build the test_alarm_server Docker image.
# Run from the workspace root (the directory containing src/ and build/).

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../../.." && pwd)"
cd "$REPO_ROOT"

# The Dockerfile copies open62541 source from build/.../_deps/, which is
# populated by `colcon build --packages-select ros2_medkit_opcua` (FetchContent
# step). Fail fast if the user has not built the plugin yet.
DEPS_DIR="build/ros2_medkit_opcua/_deps/open62541pp-src/3rdparty/open62541"
if [[ ! -d "$DEPS_DIR" ]]; then
  echo "error: $DEPS_DIR does not exist." >&2
  echo "Run 'colcon build --packages-select ros2_medkit_opcua' first to" >&2
  echo "populate the FetchContent source cache." >&2
  exit 1
fi

exec docker build \
  -f src/ros2_medkit_plugins/ros2_medkit_opcua/docker/test_alarm_server/Dockerfile \
  -t ros2_medkit_alarm_test_server:dev .
