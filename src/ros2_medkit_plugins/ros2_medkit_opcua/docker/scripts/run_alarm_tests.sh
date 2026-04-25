#!/usr/bin/env bash
# Copyright 2026 mfaferek93
#
# Integration test for the native OPC-UA AlarmConditionType subscription
# bridge (issue #386). Boots test_alarm_server, points the gateway at it,
# fires alarms via the server's stdin CLI, and asserts that the gateway's
# SOVD /faults endpoint reflects the expected lifecycle.
#
# Designed to run from CI alongside the existing OpenPLC threshold-mode
# integration. Both suites can run in parallel because each owns its own
# docker network and ports.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/../../../../.." && pwd)"
NET_NAME=alarm-test-net
SERVER_NAME=alarm-test-server
GATEWAY_NAME=alarm-test-gateway
SERVER_PORT=4842
GATEWAY_PORT=8088

cleanup() {
  docker rm -f "${SERVER_NAME}" "${GATEWAY_NAME}" 2>/dev/null || true
  docker network rm "${NET_NAME}" 2>/dev/null || true
}
trap cleanup EXIT

# Poll <url> until <jq-expr> evaluates true, with a timeout. Mirrors the
# existing run_integration_tests.sh convention; never sleeps blindly.
wait_for() {
  local url="$1" expr="$2" deadline="${3:-60}"
  for i in $(seq 1 "${deadline}"); do
    if curl -sf "${url}" 2>/dev/null | jq -e "${expr}" >/dev/null 2>&1; then
      return 0
    fi
    sleep 2
  done
  echo "wait_for timed out after ${deadline} polls: ${url} ${expr}" >&2
  curl -sf "${url}" 2>/dev/null | jq . >&2 || true
  return 1
}

assert_status() {
  local fault_code="$1" expected="$2"
  local actual
  actual=$(curl -sf "http://localhost:${GATEWAY_PORT}/api/v1/apps/tank_process/faults/${fault_code}" \
             | jq -r '.status')
  if [[ "${actual}" != "${expected}" ]]; then
    echo "ASSERT FAILED: ${fault_code} status=${actual}, expected=${expected}" >&2
    return 1
  fi
  echo "  OK ${fault_code}: ${actual}"
}

cd "${REPO_ROOT}"

echo "[1/5] Build test_alarm_server image"
docker build --network=host \
  -f src/ros2_medkit_plugins/ros2_medkit_opcua/docker/test_alarm_server/Dockerfile \
  -t ros2_medkit_alarm_test_server:dev . >/dev/null

echo "[2/5] Build gateway-opcua image (re-uses existing Dockerfile.gateway)"
docker build --network=host \
  -f src/ros2_medkit_plugins/ros2_medkit_opcua/docker/Dockerfile.gateway \
  -t gateway-opcua:alarm-test . >/dev/null

docker network create "${NET_NAME}" >/dev/null

echo "[3/5] Start test_alarm_server (with stdin pipe for CLI commands)"
SERVER_CTRL=$(mktemp -d)
mkfifo "${SERVER_CTRL}/stdin"
# shellcheck disable=SC2094
docker run -d --rm --name "${SERVER_NAME}" --network "${NET_NAME}" \
  -i ros2_medkit_alarm_test_server:dev --port "${SERVER_PORT}" \
  < "${SERVER_CTRL}/stdin" >/dev/null
exec 3>"${SERVER_CTRL}/stdin"
# Wait for server to bind; the binary prints "READY ..." after listen.
for i in $(seq 1 30); do
  if docker logs "${SERVER_NAME}" 2>&1 | grep -q '^READY '; then
    break
  fi
  sleep 1
done

echo "[4/5] Start gateway with alarm-mode node_map"
mkdir -p /tmp/alarm_test_config
cat >/tmp/alarm_test_config/alarm_nodes.yaml <<EOF
area_id: plc_systems
component_id: alarm_test_runtime
nodes: []
event_alarms:
  - alarm_source: "ns=2;s=Alarms.Overpressure"
    entity_id: tank_process
    fault_code: PLC_OVERPRESSURE
EOF

docker run -d --name "${GATEWAY_NAME}" --network "${NET_NAME}" \
  -p "${GATEWAY_PORT}:8080" \
  -v /tmp/alarm_test_config:/config:ro \
  -e ROS_DOMAIN_ID=66 \
  -e OPCUA_ENDPOINT_URL="opc.tcp://${SERVER_NAME}:${SERVER_PORT}" \
  -e OPCUA_NODE_MAP_PATH="/config/alarm_nodes.yaml" \
  gateway-opcua:alarm-test \
  bash -c '
    set -e
    mkdir -p /var/lib/ros2_medkit/rosbags
    cat >/config/manifest.yaml <<MAN
manifest_version: "1.0"
MAN
    source /opt/ros/jazzy/setup.bash
    source /root/ws/install/setup.bash
    PLUGIN_PATH=$(find /root/ws/install -name "libros2_medkit_opcua_plugin.so" | head -1)
    exec ros2 run ros2_medkit_gateway gateway_node \
      --ros-args --params-file /config/gateway_params.yaml \
      -p plugins.opcua.path:="$PLUGIN_PATH" \
      -p discovery.mode:=hybrid \
      -p discovery.manifest_path:=/config/manifest.yaml \
      -p discovery.manifest_strict_validation:=false
  ' >/dev/null

# Wait for entity discovery; tank_process appears once node_map loads.
wait_for "http://localhost:${GATEWAY_PORT}/api/v1/apps" \
         '.items | map(.id) | contains(["tank_process"])' 60

echo "[5/5] Run alarm scenarios"

echo "  - fire Overpressure (severity=750)"
echo "fire Overpressure 750" >&3
wait_for "http://localhost:${GATEWAY_PORT}/api/v1/apps/tank_process/faults" \
         '.items | map(.code) | contains(["PLC_OVERPRESSURE"])' 30
assert_status PLC_OVERPRESSURE CONFIRMED

echo "  - ack Overpressure"
echo "ack Overpressure" >&3
sleep 1  # allow event to propagate; non-flaky because we re-poll status next

echo "  - clear Overpressure (latch via Retain=true via 'latch')"
echo "latch Overpressure" >&3
wait_for "http://localhost:${GATEWAY_PORT}/api/v1/apps/tank_process/faults/PLC_OVERPRESSURE" \
         '.status == "HEALED"' 20
assert_status PLC_OVERPRESSURE HEALED

echo "  - confirm Overpressure"
echo "confirm Overpressure" >&3
wait_for "http://localhost:${GATEWAY_PORT}/api/v1/apps/tank_process/faults/PLC_OVERPRESSURE" \
         '.status == "CLEARED" or (.status == "absent")' 20

echo "All alarm scenarios passed."
exec 3>&-
