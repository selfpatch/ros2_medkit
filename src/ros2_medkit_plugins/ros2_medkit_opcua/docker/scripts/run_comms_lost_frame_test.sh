#!/usr/bin/env bash
# Copyright 2026 bburda
#
# E2E: PLC_COMMS_LOST on the plc_runtime component is framed from the apps it hosts.
# The frames hold the values from before the outage, keep captured_at across a restart,
# and are not served for occurrence 2 after a restart with the link down.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/../../../../.." && pwd)"
NET_NAME=comms-lost-frame-net
SERVER_NAME=comms-lost-frame-server
GATEWAY_NAME=comms-lost-frame-gateway
SERVER_PORT=4843
GATEWAY_PORT=8090
API="http://localhost:${GATEWAY_PORT}/api/v1"
CONFIG_DIR=/tmp/comms_lost_frame_config
COMPONENT_ID=plc_runtime
APP_A=tank_process
APP_B=valve_process
ALARMS_APP="${COMPONENT_ID}_alarms"
FAULT_CODE=PLC_COMMS_LOST
# Written while the link is up, so the frozen values are the ones read before the outage.
FROZEN_STATUS_WORD=5
FROZEN_FAULT_CODE=77

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
  return 0
}
trap cleanup EXIT

fail() {
  echo "ASSERT FAILED: $*" >&2
  exit 1
}

ok() {
  echo "  OK $*"
}

# Poll <url> until <jq-expr> is true.
wait_for() {
  local url="$1" expr="$2" deadline="${3:-60}"
  for _ in $(seq 1 "${deadline}"); do
    if curl -sf "${url}" 2>/dev/null | jq -e "${expr}" >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done
  curl -sf "${url}" 2>/dev/null | jq . >&2 || true
  fail "timed out: ${url} ${expr}"
}

wait_until_status() {
  local expected="$1" deadline="${2:-60}" actual=""
  for _ in $(seq 1 "${deadline}"); do
    actual=$(curl -sf "${API}/faults" 2>/dev/null |
      jq -r --arg code "${FAULT_CODE}" '.items[] | select(.fault_code == $code) | .status' | head -1) || true
    if [[ "${actual}" == "${expected}" ]]; then
      ok "${FAULT_CODE}: ${actual}"
      return 0
    fi
    sleep 1
  done
  fail "${FAULT_CODE} expected=${expected} actual=${actual:-<absent>}"
}

# The component's fault detail, or nothing.
detail() {
  curl -sf "${API}/components/${COMPONENT_ID}/faults/${FAULT_CODE}" 2>/dev/null || true
}

# The freeze_frame of <app> in a fault detail.
frame_in() {
  jq -c --arg app "$2" \
    '[(.environment_data.snapshots // [])[] | select(.type == "freeze_frame" and .name == $app)] | first // empty' <<<"$1"
}

# Poll until the component's detail carries a frame of <app> that satisfies <jq-expr>.
wait_frame() {
  local app="$1" expr="$2" frame=""
  for _ in $(seq 1 60); do
    frame="$(frame_in "$(detail)" "${app}")"
    if [[ -n "${frame}" ]] && jq -e "${expr}" <<<"${frame}" >/dev/null 2>&1; then
      echo "${frame}"
      return 0
    fi
    sleep 1
  done
  fail "no frame of ${app} on ${FAULT_CODE} matching ${expr} (last: ${frame:-none})"
}

check() {
  local json="$1" expr="$2" what="$3"
  jq -e "${expr}" <<<"${json}" >/dev/null || { jq . <<<"${json}" >&2; fail "${what}"; }
  ok "${what}"
}

cd "${REPO_ROOT}"

docker rm -f "${SERVER_NAME}" "${GATEWAY_NAME}" >/dev/null 2>&1 || true
docker network rm "${NET_NAME}" >/dev/null 2>&1 || true

echo "[1/8] Build test_alarm_server image"
docker build --network=host \
  -f src/ros2_medkit_plugins/ros2_medkit_opcua/docker/test_alarm_server/Dockerfile \
  -t ros2_medkit_alarm_test_server:dev . >/dev/null

echo "[2/8] Build gateway-opcua image"
docker build --network=host \
  -f src/ros2_medkit_plugins/ros2_medkit_opcua/docker/Dockerfile.gateway \
  -t gateway-opcua:comms-lost-frame-test . >/dev/null

docker network create "${NET_NAME}" >/dev/null

# Starts the server container with its CLI on FD 3 and waits for READY.
start_server() {
  # Read and write on FD 3, so neither end of the FIFO blocks on open (see run_alarm_tests.sh).
  exec 3<>"${SERVER_CTRL}/stdin"
  # shellcheck disable=SC2094
  docker run --rm --name "${SERVER_NAME}" --network "${NET_NAME}" \
    -i ros2_medkit_alarm_test_server:dev --port "${SERVER_PORT}" \
    <&3 >/dev/null 2>&1 &
  SERVER_DOCKER_PID=$!
  for _ in $(seq 1 60); do
    if [[ "$(docker logs "${SERVER_NAME}" 2>&1)" == *"READY "* ]]; then
      return 0
    fi
    sleep 1
  done
  fail "test_alarm_server never printed READY"
}

echo "[3/8] Start the OPC-UA server (stdin pipe for the CLI)"
SERVER_CTRL=$(mktemp -d)
mkfifo "${SERVER_CTRL}/stdin"
start_server

echo "[4/8] Start the gateway against it"
mkdir -p "${CONFIG_DIR}"
# Two apps with one polled value each, and none on the component. The component owns the
# connection, and so the comms-lost fault.
cat >"${CONFIG_DIR}/comms_lost_nodes.yaml" <<EOF
area_id: plc_systems
component_id: ${COMPONENT_ID}
nodes:
  - node_id: "ns=2;s=StatusWord"
    entity_id: ${APP_A}
    data_name: status_word
    data_type: int
  - node_id: "ns=2;s=FaultCode"
    entity_id: ${APP_B}
    data_name: fault_code
    data_type: int
# Adds the ${ALARMS_APP} app: hosted by the component, no data of its own.
auto_alarms: true
EOF
cat >"${CONFIG_DIR}/manifest.yaml" <<EOF
manifest_version: "1.0"
EOF
# The read-only bind mount shadows the image's /config, so every file the gateway reads is staged.
cp src/ros2_medkit_plugins/ros2_medkit_opcua/docker/gateway_params.yaml "${CONFIG_DIR}/gateway_params.yaml"

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
    # The fault manager first, so its services exist before the plugin reports.
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

wait_for "${API}/apps" ".items | map(.id) | contains([\"${APP_A}\", \"${APP_B}\", \"${ALARMS_APP}\"])" 90

echo "[5/8] Set a distinctive value on each app's server node, then kill the link"
echo "set StatusWord ${FROZEN_STATUS_WORD}" >&3
echo "set FaultCode ${FROZEN_FAULT_CODE}" >&3
# The gateway must have polled both before the server dies, or the frames would hold the old values.
wait_for "${API}/apps/${APP_A}/x-plc-data" \
  ".connected == true and (.items[] | select(.name == \"status_word\") | .value) == ${FROZEN_STATUS_WORD}" 30
wait_for "${API}/apps/${APP_B}/x-plc-data" \
  ".connected == true and (.items[] | select(.name == \"fault_code\") | .value) == ${FROZEN_FAULT_CODE}" 30
ok "status_word=${FROZEN_STATUS_WORD} and fault_code=${FROZEN_FAULT_CODE} read over a live link"
for app in "${APP_A}" "${APP_B}"; do
  wait_for "${API}/apps/${app}/data" '.["x-medkit"].connected == true' 30
  ok "GET /apps/${app}/data reports x-medkit.connected == true over a live link"
done
# The snapshot time while the values were live: a frame taken after the outage must not be older.
LIVE_TS_A="$(curl -sf "${API}/apps/${APP_A}/data" | jq -r '.["x-medkit"].timestamp')"
LIVE_TS_B="$(curl -sf "${API}/apps/${APP_B}/data" | jq -r '.["x-medkit"].timestamp')"

exec 3>&-
docker rm -f "${SERVER_NAME}" >/dev/null 2>&1 || true
OUTAGE_START="$(date +%s)"
# comms_lost_debounce_ms defaults to 5000, so the fault confirms seconds after the link dies.
wait_until_status CONFIRMED 90

echo "[6/8] Assert the component's fault detail carries a frame per hosted app"
FRAME_A="$(wait_frame "${APP_A}" ".[\"x-medkit\"].full_data.status_word == ${FROZEN_STATUS_WORD}")"
ok "freeze_frame named ${APP_A} holds status_word=${FROZEN_STATUS_WORD} from before the outage"
FRAME_B="$(wait_frame "${APP_B}" ".[\"x-medkit\"].full_data.fault_code == ${FROZEN_FAULT_CODE}")"
ok "freeze_frame named ${APP_B} holds fault_code=${FROZEN_FAULT_CODE} from before the outage"
for pair in "${FRAME_A}|${LIVE_TS_A}" "${FRAME_B}|${LIVE_TS_B}"; do
  frame="${pair%|*}"
  live_ts="${pair#*|}"
  check "${frame}" '.["x-medkit"].source == "plugin_data_provider"' "the frame was read through the app's DataProvider"
  check "${frame}" '.["x-medkit"].connected == false' "the frame reports x-medkit.connected == false"
  # The snapshot time is when the values were last seen: not before they were live, not after the link died.
  check "${frame}" "(.[\"x-medkit\"].source_timestamp | type == \"number\") and .[\"x-medkit\"].source_timestamp >= ${live_ts} and .[\"x-medkit\"].source_timestamp <= ${OUTAGE_START}" \
    "the frame's source_timestamp is from before the outage"
done
check "$(detail)" "[(.environment_data.snapshots // [])[] | select(.name == \"${COMPONENT_ID}\")] | length == 0" \
  "no frame is claimed for the component itself"
check "$(detail)" "[(.environment_data.snapshots // [])[] | select(.type == \"freeze_frame\")] | length == 2" \
  "exactly one frame per hosted app"
CAPTURED_A="$(jq -r '.["x-medkit"].captured_at' <<<"${FRAME_A}")"
CAPTURED_B="$(jq -r '.["x-medkit"].captured_at' <<<"${FRAME_B}")"

echo "[7/8] Restart the gateway container: the frames keep their captured_at"
docker restart "${GATEWAY_NAME}" >/dev/null
wait_for "${API}/apps" ".items | map(.id) | contains([\"${APP_A}\", \"${APP_B}\", \"${ALARMS_APP}\"])" 90
FRAME_A="$(wait_frame "${APP_A}" ".[\"x-medkit\"].full_data.status_word == ${FROZEN_STATUS_WORD}")"
FRAME_B="$(wait_frame "${APP_B}" ".[\"x-medkit\"].full_data.fault_code == ${FROZEN_FAULT_CODE}")"
check "${FRAME_A}" ".[\"x-medkit\"].captured_at == \"${CAPTURED_A}\"" "${APP_A}: captured_at ${CAPTURED_A} survives the restart"
check "${FRAME_B}" ".[\"x-medkit\"].captured_at == \"${CAPTURED_B}\"" "${APP_B}: captured_at ${CAPTURED_B} survives the restart"
check "${FRAME_A}" '.["x-medkit"] | has("capture_origin") | not' "the reloaded frame is not marked as a startup read"
# Read the whole log first: grep -q closing a pipe makes docker logs fail under pipefail.
gw_log="$(docker logs "${GATEWAY_NAME}" 2>&1)"
grep -qF "Entity freeze-frame: reloaded frames for 1 fault(s) from the store" <<<"${gw_log}" ||
  fail "the restarted gateway did not log a reload of the stored frames"
ok "log: reloaded frames for 1 fault(s) from the store"

echo "[8/8] A new occurrence after a restart with the link down serves no frame of the old one"
# The gateway is down while the link dies, so occurrence 2 confirms in a gateway life
# whose hosted reads are empty.
start_server
wait_for "${API}/faults?status=all" ".items[] | select(.fault_code == \"${FAULT_CODE}\" and .status == \"CLEARED\")" 150
ok "${FAULT_CODE}: CLEARED after the link came back"
docker stop "${GATEWAY_NAME}" >/dev/null
[[ "$(docker inspect -f '{{.State.Status}}' "${GATEWAY_NAME}")" == "exited" ]] || fail "the gateway container did not exit"
ok "the gateway container has exited"
exec 3>&-
docker rm -f "${SERVER_NAME}" >/dev/null 2>&1 || true
docker start "${GATEWAY_NAME}" >/dev/null
wait_for "${API}/apps" ".items | map(.id) | contains([\"${APP_A}\", \"${APP_B}\", \"${ALARMS_APP}\"])" 90
wait_for "${API}/components/${COMPONENT_ID}/faults/${FAULT_CODE}" \
  '.["x-medkit"].status_raw == "CONFIRMED" and .["x-medkit"].occurrence_count == 2' 150
ok "${FAULT_CODE}: CONFIRMED as occurrence 2 with the link down"
# An absent frame proves nothing until a drop warning shows. The judge at start or the
# confirm of occurrence 2 logs it, whichever runs first.
DROP_CONFIRM="Entity freeze-frame for fault '${FAULT_CODE}': occurrence 2 confirmed"
DROP_JUDGE="Entity freeze-frame for fault '${FAULT_CODE}': the stored frame of entity"
dropped=""
for _ in $(seq 1 90); do
  gw_log="$(docker logs "${GATEWAY_NAME}" 2>&1)"
  if grep -qF "${DROP_CONFIRM}" <<<"${gw_log}"; then
    dropped="when occurrence 2 confirmed"
    break
  fi
  if grep -qF "${DROP_JUDGE}" <<<"${gw_log}"; then
    dropped="by the judge at start"
    break
  fi
  sleep 1
done
[[ -n "${dropped}" ]] || fail "the gateway never logged that the frame of ${FAULT_CODE} occurrence 1 was dropped"
ok "log: the frame of occurrence 1 was dropped ${dropped}"
wait_for "${API}/components/${COMPONENT_ID}/faults/${FAULT_CODE}" \
  "[(.environment_data.snapshots // [])[] | select(.type == \"freeze_frame\" and (.[\"x-medkit\"].captured_at == \"${CAPTURED_A}\" or .[\"x-medkit\"].captured_at == \"${CAPTURED_B}\"))] | length == 0" 60
ok "no frame of ${FAULT_CODE} carries the captured_at of occurrence 1"

echo "Loss-of-comms freeze-frame scenario passed."
