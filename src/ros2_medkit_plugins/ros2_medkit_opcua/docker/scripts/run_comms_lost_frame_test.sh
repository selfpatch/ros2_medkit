#!/usr/bin/env bash
# Copyright 2026 mfaferek93
#
# Integration test for the loss-of-comms freeze-frame on a plugin-backed
# component.
#
# The field failure this reproduces: the bridge reports PLC_COMMS_LOST under
# the PLC runtime component, a component holds no data values of its own, and
# the fault detail therefore carried no freeze-frame at all - the operator got
# a fault about a dead link with nothing saying what the line was doing when it
# died. The values were never gone: the app the component hosts keeps serving
# the last row it read.
#
# So this scenario connects the gateway to an OPC-UA server, writes a
# distinctive value, kills the server, and asserts that the component's
# PLC_COMMS_LOST detail carries a freeze-frame named after the hosted app and
# holding that pre-outage value. The distinctive value is what separates a real
# frozen row from a coincidence: nothing can read it back once the server is
# gone.
#
# Why no existing suite catches this: run_integration_tests.sh keeps its server
# up throughout, and while run_alarm_tests.sh does stop its server once (to
# prove the client re-subscribes afterwards) it asserts only on fault status
# transitions. No suite here reads a fault's environment_data at all, so a
# fault detail that lost its freeze-frame looks exactly like one that kept it.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/../../../../.." && pwd)"
NET_NAME=comms-lost-frame-net
SERVER_NAME=comms-lost-frame-server
GATEWAY_NAME=comms-lost-frame-gateway
SERVER_PORT=4843
GATEWAY_PORT=8090
CONFIG_DIR=/tmp/comms_lost_frame_config
COMPONENT_ID=plc_runtime
APP_ID=tank_process
FAULT_CODE=PLC_COMMS_LOST
# Written while the link is up, so the frozen row is provably the one read
# before the outage: after the server is killed nothing can read it back.
FROZEN_STATUS_WORD=5

cleanup() {
  local rc=$?
  if [[ ${rc} -ne 0 ]]; then
    for c in "${SERVER_NAME}" "${GATEWAY_NAME}"; do
      echo "=== ${c} logs (cleanup trap) ===" >&2
      docker logs "${c}" >&2 2>&1 || true
    done
  fi
  docker rm -f "${SERVER_NAME}" "${GATEWAY_NAME}" >/dev/null 2>&1 || true
  docker network rm "${NET_NAME}" >/dev/null 2>&1 || true
  if [[ -n "${SERVER_DOCKER_PID:-}" ]]; then
    kill "${SERVER_DOCKER_PID}" 2>/dev/null || true
    wait "${SERVER_DOCKER_PID}" 2>/dev/null || true
  fi
  rm -rf "${CONFIG_DIR}" "${SERVER_CTRL:-/tmp/comms_lost_frame_ctrl_unset}"
}
trap cleanup EXIT

fail() {
  echo "ASSERT FAILED: $*" >&2
  return 1
}

# Poll <url> until <jq-expr> evaluates true. Same convention as the other
# scenarios here: never sleep blindly.
wait_for() {
  local url="$1" expr="$2" deadline="${3:-60}"
  for _ in $(seq 1 "${deadline}"); do
    if curl -sf "${url}" 2>/dev/null | jq -e "${expr}" >/dev/null 2>&1; then
      return 0
    fi
    sleep 2
  done
  echo "wait_for timed out after ${deadline} polls: ${url} ${expr}" >&2
  curl -sf "${url}" 2>/dev/null | jq . >&2 || true
  return 1
}

wait_until_status() {
  local fault_code="$1" expected="$2" deadline="${3:-30}"
  local actual=""
  for _ in $(seq 1 "${deadline}"); do
    # ``|| true``: pipefail would otherwise abort the whole run the first time
    # the gateway is momentarily unreachable, which is the state this poll
    # exists to sit through.
    actual=$(curl -sf "http://localhost:${GATEWAY_PORT}/api/v1/faults" 2>/dev/null \
             | jq -r --arg code "${fault_code}" \
               '.items[] | select(.fault_code == $code) | .status' \
             | head -1) || true
    if [[ "${actual}" == "${expected}" ]]; then
      echo "  OK ${fault_code}: ${actual}"
      return 0
    fi
    sleep 2
  done
  echo "wait_until_status timed out: ${fault_code} expected=${expected} actual=${actual:-<absent>}" >&2
  curl -sf "http://localhost:${GATEWAY_PORT}/api/v1/faults" 2>/dev/null | jq . >&2 || true
  return 1
}

cd "${REPO_ROOT}"

# Idempotent teardown of leftovers from a run that was killed outside bash's
# signal handling, or ``docker network create`` below fails under set -e.
docker rm -f "${SERVER_NAME}" "${GATEWAY_NAME}" >/dev/null 2>&1 || true
docker network rm "${NET_NAME}" >/dev/null 2>&1 || true

echo "[1/6] Build test_alarm_server image"
docker build --network=host \
  -f src/ros2_medkit_plugins/ros2_medkit_opcua/docker/test_alarm_server/Dockerfile \
  -t ros2_medkit_alarm_test_server:dev . >/dev/null

echo "[2/6] Build gateway-opcua image"
docker build --network=host \
  -f src/ros2_medkit_plugins/ros2_medkit_opcua/docker/Dockerfile.gateway \
  -t gateway-opcua:comms-lost-frame-test . >/dev/null

docker network create "${NET_NAME}" >/dev/null

echo "[3/6] Start the OPC-UA server (stdin pipe for the CLI)"
SERVER_CTRL=$(mktemp -d)
mkfifo "${SERVER_CTRL}/stdin"
# Open the FIFO read+write on FD 3 first so neither end blocks, and keep the
# foreground docker run as a shell job so the FIFO stays connected (see
# run_alarm_tests.sh for the full reasoning).
exec 3<>"${SERVER_CTRL}/stdin"
# shellcheck disable=SC2094
docker run --rm --name "${SERVER_NAME}" --network "${NET_NAME}" \
  -i ros2_medkit_alarm_test_server:dev --port "${SERVER_PORT}" \
  <&3 >/dev/null 2>&1 &
SERVER_DOCKER_PID=$!
for _ in $(seq 1 30); do
  if docker logs "${SERVER_NAME}" 2>&1 | grep -q '^READY '; then
    break
  fi
  sleep 1
done

echo "[4/6] Start the gateway against it"
mkdir -p "${CONFIG_DIR}"
# Two polled data points on the hosted App and none on the Component: that is
# the shape the ruling is about. The Component owns the connection (and so the
# comms-lost fault), the App owns the values.
cat >"${CONFIG_DIR}/comms_lost_nodes.yaml" <<EOF
area_id: plc_systems
component_id: ${COMPONENT_ID}
nodes:
  - node_id: "ns=2;s=StatusWord"
    entity_id: ${APP_ID}
    data_name: status_word
    data_type: int
  - node_id: "ns=2;s=FaultCode"
    entity_id: ${APP_ID}
    data_name: fault_code
    data_type: int
EOF
cat >"${CONFIG_DIR}/manifest.yaml" <<EOF
manifest_version: "1.0"
EOF
# The :ro bind mount shadows the image's baked /config, so every file the
# gateway reads has to be staged into it.
cp src/ros2_medkit_plugins/ros2_medkit_opcua/docker/gateway_params.yaml \
   "${CONFIG_DIR}/gateway_params.yaml"

docker run -d --name "${GATEWAY_NAME}" --network "${NET_NAME}" \
  -p "${GATEWAY_PORT}:8080" \
  -v "${CONFIG_DIR}":/config:ro \
  -e ROS_DOMAIN_ID=81 \
  -e OPCUA_ENDPOINT_URL="opc.tcp://${SERVER_NAME}:${SERVER_PORT}" \
  -e OPCUA_NODE_MAP_PATH="/config/comms_lost_nodes.yaml" \
  gateway-opcua:comms-lost-frame-test \
  bash -c '
    set -e
    mkdir -p /var/lib/ros2_medkit/rosbags
    source /opt/ros/jazzy/setup.bash
    source /root/ws/install/setup.bash
    # fault_manager_node first, so its services are advertised before the
    # plugin reports anything to them.
    ros2 run ros2_medkit_fault_manager fault_manager_node \
      > /var/lib/ros2_medkit/fault_manager.log 2>&1 &
    for i in $(seq 1 30); do
      if ros2 service list 2>/dev/null | grep -q "/fault_manager/report_fault"; then
        break
      fi
      sleep 0.2
    done
    PLUGIN_PATH=$(find /root/ws/install -name "libros2_medkit_opcua_plugin.so" | head -1)
    exec ros2 run ros2_medkit_gateway gateway_node \
      --ros-args --params-file /config/gateway_params.yaml \
      -p plugins.opcua.path:="$PLUGIN_PATH" \
      -p discovery.mode:=hybrid \
      -p discovery.manifest_path:=/config/manifest.yaml \
      -p discovery.manifest_strict_validation:=false
  ' >/dev/null

wait_for "http://localhost:${GATEWAY_PORT}/api/v1/apps" \
         ".items | map(.id) | contains([\"${APP_ID}\"])" 60

echo "[5/6] Freeze a distinctive value, then kill the link"
echo "set StatusWord ${FROZEN_STATUS_WORD}" >&3
# The gateway must have polled it before the server dies, or the frame would
# hold the pre-write value and the assertion below would be vacuous.
wait_for "http://localhost:${GATEWAY_PORT}/api/v1/apps/${APP_ID}/x-plc-data" \
         ".connected == true and (.items[] | select(.name == \"status_word\") | .value) == ${FROZEN_STATUS_WORD}" 30
echo "  OK status_word=${FROZEN_STATUS_WORD} read over a live link"

exec 3>&-
docker rm -f "${SERVER_NAME}" >/dev/null 2>&1 || true
# comms_lost_debounce is 5 s by default, so the fault needs a few poll cycles.
wait_until_status "${FAULT_CODE}" CONFIRMED 40

echo "[6/6] Assert the component's fault detail carries the hosted app's frame"
DETAIL=$(curl -sf "http://localhost:${GATEWAY_PORT}/api/v1/components/${COMPONENT_ID}/faults/${FAULT_CODE}") \
  || fail "GET /components/${COMPONENT_ID}/faults/${FAULT_CODE} did not answer 2xx"

FRAME=$(jq -c --arg app "${APP_ID}" \
  '(.environment_data.snapshots // []) | map(select(.type == "freeze_frame" and .name == $app)) | .[0] // empty' \
  <<<"${DETAIL}")
if [[ -z "${FRAME}" ]]; then
  echo "no freeze_frame named '${APP_ID}' on ${FAULT_CODE}; detail was:" >&2
  jq . <<<"${DETAIL}" >&2 || echo "${DETAIL}" >&2
  fail "the component's loss-of-comms fault carries no frame from the app it hosts"
fi
echo "  OK freeze_frame named ${APP_ID}"

jq -e --argjson want "${FROZEN_STATUS_WORD}" '."x-medkit".full_data.status_word == $want' <<<"${FRAME}" >/dev/null \
  || { jq . <<<"${FRAME}" >&2; fail "the frame does not hold the value read before the link died"; }
echo "  OK frame holds status_word=${FROZEN_STATUS_WORD} from before the outage"

# The capture path is the only provenance an entity frame has: its values are
# not a ROS message, so topic and message_type are necessarily empty.
jq -e '(."x-medkit".source // "") | length > 0' <<<"${FRAME}" >/dev/null \
  || { jq . <<<"${FRAME}" >&2; fail "the frame names no capture path in x-medkit.source"; }
echo "  OK frame names its capture path: $(jq -r '."x-medkit".source' <<<"${FRAME}")"

# Nothing is readable on the component itself, so it must not claim a frame.
jq -e --arg comp "${COMPONENT_ID}" \
  '[(.environment_data.snapshots // [])[] | select(.name == $comp)] | length == 0' <<<"${DETAIL}" >/dev/null \
  || { jq '.environment_data.snapshots' <<<"${DETAIL}" >&2; fail "a frame was claimed for the component itself"; }
echo "  OK no frame claimed for the component"

# The frame's provenance block, for whoever reads a run: which path read the
# values, what the payload said about the link, and when they were read.
echo "  frame x-medkit: $(jq -c '."x-medkit" | del(.full_data)' <<<"${FRAME}")"

echo "Loss-of-comms freeze-frame scenario passed."
