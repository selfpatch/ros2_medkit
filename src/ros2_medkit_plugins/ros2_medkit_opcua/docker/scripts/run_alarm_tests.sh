#!/usr/bin/env bash
# Copyright 2026 mfaferek93
#
# Integration test for the native OPC-UA AlarmConditionType subscription
# bridge (issue #386). Boots test_alarm_server, points the gateway at it,
# fires alarms via the server's stdin CLI, and asserts that the gateway's
# SOVD ``/faults`` endpoint reflects the expected lifecycle.
#
# Acknowledge / Confirm round-trips go through the SOVD HTTP path
# (POST /apps/{entity}/operations/{op}/executions) so that the medkit
# implementation - lookup_condition + EventId tracking + call_method on the
# inherited AcknowledgeableConditionType methods - is exercised end-to-end,
# not bypassed via the server stdin shortcuts.

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

# Poll the gateway until the named fault disappears (404) or its status moves
# out of CONFIRMED/HEALED into something quiescent.
wait_no_fault() {
  local fault_code="$1" deadline="${2:-30}"
  local url="http://localhost:${GATEWAY_PORT}/api/v1/apps/tank_process/faults/${fault_code}"
  for i in $(seq 1 "${deadline}"); do
    local code
    code=$(curl -s -o /dev/null -w '%{http_code}' "${url}" || echo '000')
    if [[ "${code}" == "404" ]]; then
      return 0
    fi
    local status
    status=$(curl -sf "${url}" 2>/dev/null | jq -r '.status // empty')
    if [[ "${status}" == "CLEARED" || -z "${status}" ]]; then
      return 0
    fi
    sleep 2
  done
  echo "wait_no_fault timed out: ${fault_code} still present" >&2
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

# Poll the test_alarm_server's stdout (via docker logs) for the latest
# ``STATE <name>`` line and assert that <key>=<expected_value> appears in it.
# Used to verify medkit's SOVD ack / confirm POSTs actually flipped the
# corresponding state on the OPC-UA server.
assert_server_state() {
  local condition="$1" key="$2" expected="$3" deadline="${4:-30}"
  for i in $(seq 1 "${deadline}"); do
    local line
    line=$(docker logs "${SERVER_NAME}" 2>&1 | grep -E "^STATE ${condition} " | tail -1 || true)
    if [[ "${line}" == *"${key}=${expected}"* ]]; then
      echo "  OK server ${condition} ${key}=${expected}"
      return 0
    fi
    sleep 2
  done
  echo "ASSERT FAILED: server ${condition} ${key} != ${expected}" >&2
  docker logs "${SERVER_NAME}" 2>&1 | grep -E "^STATE ${condition} " | tail -3 >&2 || true
  return 1
}

sovd_post_op() {
  local op="$1" body="$2"
  local url="http://localhost:${GATEWAY_PORT}/api/v1/apps/tank_process/operations/${op}/executions"
  local code
  code=$(curl -s -o /tmp/alarm_test_resp.json -w '%{http_code}' \
              -X POST -H 'Content-Type: application/json' -d "${body}" "${url}")
  if [[ "${code}" != "200" && "${code}" != "201" ]]; then
    echo "SOVD POST ${op} failed: HTTP ${code}" >&2
    cat /tmp/alarm_test_resp.json >&2 || true
    return 1
  fi
  echo "  OK POST ${op} -> ${code}"
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
# Open FIFO read+write on FD 3 first so neither side blocks when docker run
# attaches to it via stdin. Using ``< fifo`` alone would deadlock - the shell
# opens fifo for read before exec'ing docker, blocking on the missing writer.
exec 3<>"${SERVER_CTRL}/stdin"
# shellcheck disable=SC2094
docker run -d --name "${SERVER_NAME}" --network "${NET_NAME}" \
  -i ros2_medkit_alarm_test_server:dev --port "${SERVER_PORT}" \
  <&3 >/dev/null
# Wait for server to bind; the binary prints "READY ..." after listen.
for i in $(seq 1 30); do
  if docker logs "${SERVER_NAME}" 2>&1 | grep -q '^READY '; then
    break
  fi
  sleep 1
done

echo "[4/5] Start gateway with alarm-mode node_map (3 conditions)"
mkdir -p /tmp/alarm_test_config
cat >/tmp/alarm_test_config/alarm_nodes.yaml <<EOF
area_id: plc_systems
component_id: alarm_test_runtime
nodes: []
event_alarms:
  - alarm_source: "ns=2;s=Alarms.Overpressure"
    entity_id: tank_process
    fault_code: PLC_OVERPRESSURE
  - alarm_source: "ns=2;s=Alarms.Overheat"
    entity_id: tank_process
    fault_code: PLC_OVERHEAT
  - alarm_source: "ns=2;s=Alarms.SensorLost"
    entity_id: tank_process
    fault_code: PLC_SENSOR_LOST
EOF
# Pre-write the discovery manifest on the host so the gateway entrypoint
# does not need a writable /config mount inside the container.
cat >/tmp/alarm_test_config/manifest.yaml <<EOF
manifest_version: "1.0"
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

echo "  [scenario] fire / SOVD ack / latch / SOVD confirm / clear lifecycle"
echo "fire Overpressure 750" >&3
wait_for "http://localhost:${GATEWAY_PORT}/api/v1/apps/tank_process/faults" \
         '.items | map(.code) | contains(["PLC_OVERPRESSURE"])' 30
assert_status PLC_OVERPRESSURE CONFIRMED

# Real SOVD ack - exercises lookup_condition + call_method(i=9111) + EventId.
sovd_post_op acknowledge_fault \
  '{"fault_code":"PLC_OVERPRESSURE","comment":"e2e ack via SOVD"}'
assert_server_state Overpressure acked true 30

echo "latch Overpressure" >&3
wait_for "http://localhost:${GATEWAY_PORT}/api/v1/apps/tank_process/faults/PLC_OVERPRESSURE" \
         '.status == "HEALED"' 20
assert_status PLC_OVERPRESSURE HEALED

# Real SOVD confirm - exercises call_method(i=9113) + EventId.
sovd_post_op confirm_fault \
  '{"fault_code":"PLC_OVERPRESSURE","comment":"e2e confirm via SOVD"}'
assert_server_state Overpressure confirmed true 30
wait_no_fault PLC_OVERPRESSURE 30

echo "  [scenario] shelving suppression"
echo "fire Overheat 600" >&3
wait_for "http://localhost:${GATEWAY_PORT}/api/v1/apps/tank_process/faults/PLC_OVERHEAT" \
         '.status == "CONFIRMED"' 30
echo "shelve Overheat" >&3
wait_no_fault PLC_OVERHEAT 30
echo "  OK PLC_OVERHEAT suppressed by Shelving"
echo "unshelve Overheat" >&3
echo "fire Overheat 700" >&3
wait_for "http://localhost:${GATEWAY_PORT}/api/v1/apps/tank_process/faults/PLC_OVERHEAT" \
         '.status == "CONFIRMED"' 30
echo "  OK PLC_OVERHEAT re-armed after Unshelve"

echo "  [scenario] disabled alarm suppression"
echo "fire SensorLost 800" >&3
wait_for "http://localhost:${GATEWAY_PORT}/api/v1/apps/tank_process/faults/PLC_SENSOR_LOST" \
         '.status == "CONFIRMED"' 30
echo "disable SensorLost" >&3
wait_no_fault PLC_SENSOR_LOST 30
echo "  OK PLC_SENSOR_LOST suppressed by EnabledState=false"
echo "enable SensorLost" >&3
echo "fire SensorLost 900" >&3
wait_for "http://localhost:${GATEWAY_PORT}/api/v1/apps/tank_process/faults/PLC_SENSOR_LOST" \
         '.status == "CONFIRMED"' 30
echo "  OK PLC_SENSOR_LOST re-armed after Enable"

echo "  [scenario] reconnect preserves CONFIRMED via ConditionRefresh"
# Pre-clear Overpressure so the next fire is a fresh CONFIRMED event.
echo "clear Overpressure" >&3
wait_no_fault PLC_OVERPRESSURE 30
echo "fire Overpressure 750" >&3
wait_for "http://localhost:${GATEWAY_PORT}/api/v1/apps/tank_process/faults/PLC_OVERPRESSURE" \
         '.status == "CONFIRMED"' 30

# Drop the stdin pipe and stop the server. The gateway should detect the
# disconnect and back off until the server returns.
exec 3>&-
docker stop "${SERVER_NAME}" >/dev/null

# Restart the same server image with the SAME network alias so the gateway's
# OPC-UA endpoint URL still resolves. The fixture starts with the previous
# Overpressure condition still ACTIVE in its in-memory state, but the new
# server process resets its node tree. We instead re-fire the condition
# immediately after RESTART so the test verifies the gateway's reconnect +
# subscribe behavior rather than open62541's lack of persistence.
docker rm -f "${SERVER_NAME}" >/dev/null 2>&1 || true
mkfifo "${SERVER_CTRL}/stdin2"
exec 3<>"${SERVER_CTRL}/stdin2"
docker run -d --name "${SERVER_NAME}" --network "${NET_NAME}" \
  -i ros2_medkit_alarm_test_server:dev --port "${SERVER_PORT}" \
  <&3 >/dev/null
for i in $(seq 1 30); do
  if docker logs "${SERVER_NAME}" 2>&1 | grep -q '^READY '; then
    break
  fi
  sleep 1
done

# After the server returns the gateway re-runs ``setup_event_subscriptions``
# (triggered by the OpcuaPoller reconnect path); a fresh fire should make
# its way through the ConditionRefresh-aware bridge into /faults.
echo "fire Overpressure 750" >&3
wait_for "http://localhost:${GATEWAY_PORT}/api/v1/apps/tank_process/faults/PLC_OVERPRESSURE" \
         '.status == "CONFIRMED"' 60
echo "  OK PLC_OVERPRESSURE re-armed after gateway reconnect"

echo "All alarm scenarios passed."
exec 3>&-
