#!/usr/bin/env bash
# Copyright 2026 bburda
#
# End-to-end test: a plugin entity's freeze frame across gateway restarts.
#
# Boots test_alarm_server and one gateway container that runs fault_manager_node
# and gateway_node, with the gateway's state on a named docker volume. The
# server's values are set through its stdin CLI; the frame is read from the
# fault detail over HTTP and the store file with sqlite3.
#
# Scenarios:
#   1. The gateway container is replaced: the frame keeps its values, its
#      captured_at and no capture_origin. A fault cleared while the gateway ran
#      keeps its frame too.
#   2. The fault is cleared and reported again while gateway_node is down: the
#      stored frame is discarded with a warning, and the startup catch-up reads
#      the entity and marks the frame capture_origin: startup.
#   3. A fault cleared, confirmed again and cleared again while gateway_node is
#      down: the frame of the first occurrence is discarded with a warning.
#   4. Scenario 2 with the OPC-UA server unreachable: the frame is discarded with a
#      warning that names the fault and the entity, and no frame replaces it.
#   5. The fault manager restarts with empty storage: nothing is dropped. With one
#      unrelated fault in it, the frames of the faults it does not hold are dropped
#      with a warning that names them.
#   6. entity_freeze_frame.storage.path: next to triggers.storage.path, neither
#      path, and a path that cannot be opened (scenario 1 covers an explicit path).
#   7. Subscription mode with a publishing interval of 60 s: GET /data serves values
#      at once, and scenario 2's startup frame holds the current value. GET /data
#      also serves values at once when a server that was down at gateway start
#      comes up, and serves the new value at once when the server restarts with it.
#
# A check that something did NOT happen first waits for a sentinel fault the
# plugin confirms after start: its frame comes from the drain loop, which runs
# only after the startup catch-up has returned.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/../../../../.." && pwd)"
PREFIX=ff-restart
NET_NAME="${PREFIX}-net"
SERVER_NAME="${PREFIX}-server"
GATEWAY_NAME="${PREFIX}-gateway"
VOLUME="${PREFIX}-state"
SERVER_PORT=4842
GATEWAY_PORT="${FF_RESTART_GATEWAY_PORT:-8089}"
API="http://localhost:${GATEWAY_PORT}/api/v1"
ENTITY=tank_process
FAULT=PLC_PUMP_OVERLOAD
STATE_DIR=/var/lib/ros2_medkit
EXPLICIT_STORE="${STATE_DIR}/entity_freeze_frames.db"
ROS_ENV="source /opt/ros/jazzy/setup.bash && source /root/ws/install/setup.bash"

# StatusWord bits of the node map below.
BIT_FAULT=8       # bit 3: PLC_PUMP_OVERLOAD
BIT_CLEARED=16    # bit 4: PLC_VALVE_STUCK
BIT_SENTINEL_A=256  # bit 8: PLC_SENTINEL_A
BIT_SENTINEL_B=512  # bit 9: PLC_SENTINEL_B

GW_LOG=""
CONFIG_DIR=""
SERVER_CTRL=""

cleanup() {
  local rc=$?
  if [[ ${rc} -ne 0 ]]; then
    echo "=== ${SERVER_NAME} logs ===" >&2
    docker logs "${SERVER_NAME}" 2>&1 | tail -40 >&2 || true
    echo "=== fault manager log ===" >&2
    docker exec "${GATEWAY_NAME}" bash -c 'tail -n 60 /tmp/fault_manager.log' >&2 || true
    if [[ -n "${GW_LOG}" ]]; then
      echo "=== ${GW_LOG} ===" >&2
      docker exec "${GATEWAY_NAME}" cat "${GW_LOG}" >&2 || true
    fi
  fi
  docker rm -f "${SERVER_NAME}" "${GATEWAY_NAME}" >/dev/null 2>&1 || true
  docker network rm "${NET_NAME}" >/dev/null 2>&1 || true
  docker volume rm "${VOLUME}" >/dev/null 2>&1 || true
  if [[ -n "${SERVER_DOCKER_PID:-}" ]]; then
    kill "${SERVER_DOCKER_PID}" 2>/dev/null || true
    wait "${SERVER_DOCKER_PID}" 2>/dev/null || true
  fi
  [[ -n "${CONFIG_DIR}" ]] && rm -rf "${CONFIG_DIR}"
  [[ -n "${SERVER_CTRL}" ]] && rm -rf "${SERVER_CTRL}"
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

# Runs a shell command in the gateway container with ROS sourced.
in_gw() {
  docker exec "${GATEWAY_NAME}" bash -c "${ROS_ENV} && $1"
}

# Poll the current gateway log until it holds <text>.
wait_log() {
  local text="$1" deadline="${2:-60}"
  for _ in $(seq 1 "${deadline}"); do
    if docker exec "${GATEWAY_NAME}" grep -qF -- "${text}" "${GW_LOG}"; then
      ok "log: ${text}"
      return 0
    fi
    sleep 1
  done
  fail "${GW_LOG} never logged: ${text}"
}

server() {
  echo "$*" >&3
}

# Poll the server's output until it holds <text>.
wait_server_log() {
  local text="$1"
  for _ in $(seq 1 30); do
    if [[ "$(docker logs "${SERVER_NAME}" 2>&1)" == *"${text}"* ]]; then
      return 0
    fi
    sleep 1
  done
  fail "${SERVER_NAME} never printed: ${text}"
}

# Run test_alarm_server with its stdin on FD 3 and wait for READY. Args: docker
# network options. With none the server is on the default bridge, where the
# gateway cannot resolve its name. The docker client stays in the foreground of
# a background job, so the FIFO stays wired to the container's stdin (see
# run_alarm_tests.sh).
server_run() {
  # shellcheck disable=SC2094
  docker run --rm --name "${SERVER_NAME}" "$@" \
    -i ros2_medkit_alarm_test_server:freeze-frame-test --port "${SERVER_PORT}" \
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

# Wait until the entity serves <value> for data item <name>.
wait_value() {
  wait_for "${API}/apps/${ENTITY}/x-plc-data" ".items[] | select(.name == \"$1\") | .value == $2" 60
}

fault_status() {
  curl -sf "${API}/faults?status=all" 2>/dev/null | jq -r --arg code "$1" '.items[] | select(.fault_code == $code) | .status' |
    head -1
}

wait_status() {
  local code="$1" expected="$2"
  for _ in $(seq 1 60); do
    if [[ "$(fault_status "${code}")" == "${expected}" ]]; then
      ok "${code} is ${expected}"
      return 0
    fi
    sleep 1
  done
  fail "${code} never reached ${expected} (last: $(fault_status "${code}"))"
}

# The entity's freeze_frame snapshot in the fault detail, or nothing.
frame_of() {
  curl -sf "${API}/apps/${ENTITY}/faults/$1" 2>/dev/null |
    jq -c --arg e "${ENTITY}" '[.environment_data.snapshots[]? | select(.type == "freeze_frame" and .name == $e)] | first // empty'
}

# Poll until the frame of <code> satisfies <jq-expr>, then print it.
wait_frame() {
  local code="$1" expr="$2" frame=""
  for _ in $(seq 1 60); do
    frame="$(frame_of "${code}" || true)"
    if [[ -n "${frame}" ]] && jq -e "${expr}" <<<"${frame}" >/dev/null 2>&1; then
      echo "${frame}"
      return 0
    fi
    sleep 1
  done
  fail "frame of ${code} never matched ${expr} (last: ${frame:-none})"
}

# Assert <jq-expr> on a frame and say what it proves.
check_frame() {
  local frame="$1" expr="$2" what="$3"
  jq -e "${expr}" <<<"${frame}" >/dev/null || fail "${what}: ${expr} on ${frame}"
  ok "${what}"
}

stored_codes() {
  docker exec "${GATEWAY_NAME}" sqlite3 "$1" "SELECT DISTINCT fault_code FROM entity_freeze_frames ORDER BY fault_code" |
    tr '\n' ' ' | sed 's/ $//'
}

# Raise a sentinel fault and wait for its frame without capture_origin: the drain
# loop wrote it, so the startup catch-up of this gateway life has returned.
catch_up_done() {
  local bits="$1" code="$2"
  server set StatusWord "${bits}"
  wait_frame "${code}" '.["x-medkit"] | has("capture_origin") | not' >/dev/null
  ok "${code} framed on its confirm edge, so the startup catch-up has returned"
}

gw_container_start() {
  docker rm -f "${GATEWAY_NAME}" >/dev/null 2>&1 || true
  docker run -d --name "${GATEWAY_NAME}" --network "${NET_NAME}" -p "${GATEWAY_PORT}:8080" \
    -v "${VOLUME}:${STATE_DIR}" \
    -e ROS_DOMAIN_ID=67 \
    -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    -e OPCUA_ENDPOINT_URL="opc.tcp://${SERVER_NAME}:${SERVER_PORT}" \
    -e OPCUA_NODE_MAP_PATH=/config/ff_restart_nodes.yaml \
    gateway-opcua:freeze-frame-test sleep infinity >/dev/null
  docker cp "${CONFIG_DIR}/." "${GATEWAY_NAME}:/config/"
}

# Start fault_manager_node on <database path> and wait until it answers.
fm_start() {
  docker exec -d "${GATEWAY_NAME}" bash -c "${ROS_ENV} && exec \"\$(ros2 pkg prefix ros2_medkit_fault_manager)/lib/ros2_medkit_fault_manager/fault_manager_node\" --ros-args -p database_path:=$1 -p snapshots.rosbag.enabled:=false >>/tmp/fault_manager.log 2>&1"
  in_gw "timeout 60 ros2 service call /fault_manager/list_faults ros2_medkit_msgs/srv/ListFaults '{}' >/dev/null" ||
    fail "fault manager on $1 did not answer"
}

stop_process() {
  local pattern="$1"
  docker exec "${GATEWAY_NAME}" pkill -TERM -f "${pattern}" || true
  for _ in $(seq 1 60); do
    if ! docker exec "${GATEWAY_NAME}" pgrep -f "${pattern}" >/dev/null; then
      return 0
    fi
    sleep 1
  done
  fail "${pattern} did not exit"
}

fm_stop() {
  stop_process '[f]ault_manager_node'
}

# Start gateway_node as life <name> with <params file>.
gw_launch() {
  GW_LOG="/tmp/gateway-$1.log"
  echo "  [gateway life $1, $2]"
  docker exec -d "${GATEWAY_NAME}" bash -c "${ROS_ENV} && exec \"\$(ros2 pkg prefix ros2_medkit_gateway)/lib/ros2_medkit_gateway/gateway_node\" --ros-args --params-file $2 -p plugins.opcua.path:=\"\$(find /root/ws/install -name libros2_medkit_opcua_plugin.so | head -1)\" -p discovery.mode:=hybrid -p discovery.manifest_path:=/config/manifest.yaml -p discovery.manifest_strict_validation:=false >${GW_LOG} 2>&1"
}

# gw_launch, then wait for the entity.
gw_start() {
  gw_launch "$1" "$2"
  wait_for "${API}/apps" '.items | map(.id) | contains(["tank_process"])' 90
}

gw_stop() {
  stop_process '[l]ib/ros2_medkit_gateway/gateway_node'
}

# The fault manager's own services, while gateway_node is down.
fm_clear() {
  local reply
  reply="$(in_gw "timeout 30 ros2 service call /fault_manager/clear_fault ros2_medkit_msgs/srv/ClearFault \"{fault_code: '$1'}\"")"
  [[ "${reply}" == *"success=True"* ]] || fail "clear_fault $1: ${reply}"
  ok "${1} cleared through the fault manager"
}

fm_report() {
  local reply
  reply="$(in_gw "timeout 30 ros2 service call /fault_manager/report_fault ros2_medkit_msgs/srv/ReportFault \"{fault_code: '$1', event_type: 0, severity: 2, description: 'reported while gateway_node is down', source_id: '${ENTITY}'}\"")"
  [[ "${reply}" == *"accepted=True"* ]] || fail "report_fault $1: ${reply}"
  ok "${1} reported again through the fault manager"
}

assert_logged() {
  docker exec "${GATEWAY_NAME}" grep -qF -- "$1" "${GW_LOG}" || fail "${GW_LOG} does not hold: $1"
  ok "log: $1"
}

# A gateway life <name> on <params> whose fault manager holds no fault. Confirms the
# fault with FaultCode at <marker>, then moves FaultCode to <marker>+1.
# Sets CAPTURED to the frame's captured_at.
CAPTURED=""
first_life_with_frame() {
  local name="$1" params="$2" marker="$3"
  server set StatusWord 0
  server set FaultCode "${marker}"
  gw_start "${name}-1" "${params}"
  # Printed after the events subscription matched, so the confirmation below is seen.
  wait_log "Standing-fault freeze-frame catch-up: no confirmed, unmuted faults at startup"
  wait_value fault_code "${marker}"
  server set StatusWord "${BIT_FAULT}"
  wait_status "${FAULT}" CONFIRMED
  local frame
  frame="$(wait_frame "${FAULT}" ".[\"x-medkit\"].full_data.fault_code == ${marker}")"
  check_frame "${frame}" '.["x-medkit"] | has("capture_origin") | not' "${name}: frame of ${marker} captured on the confirm edge"
  CAPTURED="$(jq -r '.["x-medkit"].captured_at' <<<"${frame}")"
  server set FaultCode $((marker + 1))
  wait_value fault_code $((marker + 1))
}

cd "${REPO_ROOT}"

docker rm -f "${SERVER_NAME}" "${GATEWAY_NAME}" >/dev/null 2>&1 || true
docker network rm "${NET_NAME}" >/dev/null 2>&1 || true
docker volume rm "${VOLUME}" >/dev/null 2>&1 || true

echo "[1/5] Build test_alarm_server and gateway-opcua images"
docker build --network=host \
  -f src/ros2_medkit_plugins/ros2_medkit_opcua/docker/test_alarm_server/Dockerfile \
  -t ros2_medkit_alarm_test_server:freeze-frame-test . >/dev/null
docker build --network=host \
  -f src/ros2_medkit_plugins/ros2_medkit_opcua/docker/Dockerfile.gateway \
  -t gateway-opcua:freeze-frame-test . >/dev/null

echo "[2/5] Start test_alarm_server"
docker network create "${NET_NAME}" >/dev/null
docker volume create "${VOLUME}" >/dev/null
SERVER_CTRL="$(mktemp -d)"
mkfifo "${SERVER_CTRL}/stdin"
# Read and write on FD 3, so neither end of the FIFO blocks on open.
exec 3<>"${SERVER_CTRL}/stdin"
server_run --network "${NET_NAME}"

CONFIG_DIR="$(mktemp -d)"
cat >"${CONFIG_DIR}/ff_restart_nodes.yaml" <<EOF
area_id: plc_systems
component_id: ff_restart_runtime
nodes:
  - node_id: "ns=2;s=StatusWord"
    entity_id: ${ENTITY}
    data_name: status_word
    data_type: int
    status_bits:
      - bit: 3
        fault_code: ${FAULT}
        severity: ERROR
        message: "Pump overload"
      - bit: 4
        fault_code: PLC_VALVE_STUCK
        severity: ERROR
        message: "Valve stuck"
      - bit: 8
        fault_code: PLC_SENTINEL_A
        severity: ERROR
        message: "Catch-up sentinel A"
      - bit: 9
        fault_code: PLC_SENTINEL_B
        severity: ERROR
        message: "Catch-up sentinel B"
  # A plain value: the frame holds it, and the CLI changes it under a standing fault.
  - node_id: "ns=2;s=FaultCode"
    entity_id: ${ENTITY}
    data_name: fault_code
    data_type: int
EOF
cat >"${CONFIG_DIR}/manifest.yaml" <<EOF
manifest_version: "1.0"
EOF
# The demo parameters without the freeze-frame store, plus each storage case.
base_params() {
  cat <<EOF
ros2_medkit_gateway:
  ros__parameters:
    server:
      host: "0.0.0.0"
      port: 8080
    refresh_interval_ms: 2000
    plugins: ["opcua"]
    plugins.opcua.poll_interval_ms: 1000
EOF
}
{
  base_params
  cat <<EOF
    entity_freeze_frame:
      storage:
        path: ""
    triggers:
      storage:
        path: "${STATE_DIR}/triggers/triggers.db"
EOF
} >"${CONFIG_DIR}/params_next_to_triggers.yaml"
{
  base_params
  cat <<EOF
    entity_freeze_frame:
      storage:
        path: ""
EOF
} >"${CONFIG_DIR}/params_no_store.yaml"
{
  base_params
  cat <<EOF
    entity_freeze_frame:
      storage:
        path: "${STATE_DIR}/no-such-dir/entity_freeze_frames.db"
EOF
} >"${CONFIG_DIR}/params_unopenable.yaml"

echo "[3/5] Restart scenarios on the demo parameters (explicit store path)"
gw_container_start
fm_start "${STATE_DIR}/faults.db"
first_life_with_frame explicit /config/gateway_params.yaml 111
captured_1="${CAPTURED}"
assert_logged "Entity freeze-frames persisted in ${EXPLICIT_STORE}"
# A second fault, confirmed at 121 and cleared while the gateway runs.
server set FaultCode 121
wait_value fault_code 121
server set StatusWord $((BIT_FAULT + BIT_CLEARED))
wait_status PLC_VALVE_STUCK CONFIRMED
frame="$(wait_frame PLC_VALVE_STUCK '.["x-medkit"].full_data.fault_code == 121')"
check_frame "${frame}" '.["x-medkit"] | has("capture_origin") | not' "PLC_VALVE_STUCK: frame of 121 captured on the confirm edge"
captured_cleared="$(jq -r '.["x-medkit"].captured_at' <<<"${frame}")"
server set StatusWord "${BIT_FAULT}"
wait_status PLC_VALVE_STUCK CLEARED
server set FaultCode 112
wait_value fault_code 112

echo "  [scenario 1] the gateway container is replaced"
docker rm -f "${GATEWAY_NAME}" >/dev/null
gw_container_start
fm_start "${STATE_DIR}/faults.db"
gw_start explicit-2 /config/gateway_params.yaml
catch_up_done $((BIT_FAULT + BIT_SENTINEL_A)) PLC_SENTINEL_A
frame="$(frame_of "${FAULT}")"
check_frame "${frame}" '.["x-medkit"].full_data.fault_code == 111' "the frame holds the value from the confirmation (111)"
check_frame "${frame}" ".[\"x-medkit\"].captured_at == \"${captured_1}\"" "the frame keeps captured_at ${captured_1}"
check_frame "${frame}" '.["x-medkit"] | has("capture_origin") | not' "the frame has no capture_origin"
frame="$(frame_of PLC_VALVE_STUCK)"
check_frame "${frame}" '.["x-medkit"].full_data.fault_code == 121' "the cleared fault still serves its frame (121)"
check_frame "${frame}" ".[\"x-medkit\"].captured_at == \"${captured_cleared}\"" "the cleared fault's frame keeps captured_at ${captured_cleared}"
assert_logged "Entity freeze-frame: reloaded frames for 2 fault(s) from the store"
[[ "$(stored_codes "${EXPLICIT_STORE}")" == "${FAULT} PLC_SENTINEL_A PLC_VALVE_STUCK" ]] ||
  fail "store rows: $(stored_codes "${EXPLICIT_STORE}")"
ok "the store holds the frames of ${FAULT}, PLC_SENTINEL_A and PLC_VALVE_STUCK"

echo "  [scenario 2] cleared and reported again while gateway_node was down"
gw_stop
fm_clear "${FAULT}"
fm_report "${FAULT}"
server set FaultCode 333
gw_start reoccurred /config/gateway_params.yaml
wait_log "Entity freeze-frame for fault '${FAULT}': the stored frame of entity '${ENTITY}' is from an earlier occurrence and was discarded (stored occurrence 1, the fault manager lists occurrence 2, CONFIRMED)"
frame="$(wait_frame "${FAULT}" '.["x-medkit"].capture_origin == "startup"')"
check_frame "${frame}" '.["x-medkit"].full_data.fault_code == 333' "the frame was read again on start (333)"
check_frame "${frame}" ".[\"x-medkit\"].captured_at > \"${captured_1}\"" "the frame has a newer captured_at"
ok "the frame is marked capture_origin: startup"

echo "  [scenario 3] cleared, confirmed again and cleared again while gateway_node was down"
gw_stop
fm_report PLC_VALVE_STUCK
fm_clear PLC_VALVE_STUCK
gw_start reoccurred-and-cleared /config/gateway_params.yaml
wait_log "Entity freeze-frame for fault 'PLC_VALVE_STUCK': the stored frame of entity '${ENTITY}' is from an earlier occurrence and was discarded (stored occurrence 1, the fault manager lists occurrence 2, CLEARED)"
frame="$(frame_of PLC_VALVE_STUCK)"
if [[ -n "${frame}" ]]; then
  check_frame "${frame}" ".[\"x-medkit\"].captured_at != \"${captured_cleared}\" and .[\"x-medkit\"].full_data.fault_code != 121" \
    "the detail does not serve the first occurrence's frame"
else
  ok "the detail does not serve the first occurrence's frame (no frame)"
fi

echo "  [scenario 4] the same as scenario 2 with the OPC-UA server unreachable"
gw_stop
fm_clear "${FAULT}"
fm_report "${FAULT}"
docker network disconnect "${NET_NAME}" "${SERVER_NAME}"
gw_start unreachable /config/gateway_params.yaml
wait_log "Entity freeze-frame for fault '${FAULT}': the stored frame of entity '${ENTITY}' is from an earlier occurrence and was discarded (stored occurrence 2, the fault manager lists occurrence 3, CONFIRMED)"
# The catch-up read of the entity, which is the sign that the catch-up has run.
wait_log "Entity freeze-frame for fault '${FAULT}': entity '${ENTITY}' values are all null"
curl -sf "${API}/apps/${ENTITY}/faults/${FAULT}" >/dev/null || fail "no fault detail for ${FAULT}"
[[ -z "$(frame_of "${FAULT}")" ]] || fail "the fault detail still serves a frame: $(frame_of "${FAULT}")"
ok "the fault detail serves no frame"
[[ "$(stored_codes "${EXPLICIT_STORE}")" == "PLC_SENTINEL_A" ]] || fail "store rows: $(stored_codes "${EXPLICIT_STORE}")"
ok "the stored row of ${FAULT} is gone"
docker network connect "${NET_NAME}" "${SERVER_NAME}"

echo "  [scenario 5] the fault manager restarts with empty storage"
gw_stop
server set StatusWord 0
fm_stop
fm_start "${STATE_DIR}/faults-empty.db"
gw_start empty-fault-manager /config/gateway_params.yaml
wait_log "Entity freeze-frame: the fault list is empty or could not be read, so no reloaded frame is dropped"
[[ "$(stored_codes "${EXPLICIT_STORE}")" == "PLC_SENTINEL_A" ]] || fail "store rows: $(stored_codes "${EXPLICIT_STORE}")"
ok "an empty fault list drops nothing: the store still holds PLC_SENTINEL_A"
gw_stop
fm_report PLC_UNRELATED
gw_start one-unrelated-fault /config/gateway_params.yaml
wait_log "Entity freeze-frame: dropped the reloaded frames of 1 fault(s) the fault manager no longer holds: PLC_SENTINEL_A"
rows="$(stored_codes "${EXPLICIT_STORE}")"
[[ -z "${rows}" || "${rows}" == "PLC_UNRELATED" ]] || fail "store rows: ${rows}"
ok "the frame of PLC_SENTINEL_A, which the fault manager does not hold, is dropped (rows: ${rows:-none})"

echo "[4/5] entity_freeze_frame.storage.path cases (scenario 6)"
storage_case() {
  local name="$1" params="$2" marker="$3" logged="$4" survives="$5"
  echo "  [case ${name}]"
  gw_stop
  fm_stop
  fm_start "${STATE_DIR}/faults-${name}.db"
  first_life_with_frame "${name}" "${params}" "${marker}"
  local captured="${CAPTURED}"
  assert_logged "${logged}"
  gw_stop
  gw_start "${name}-2" "${params}"
  local frame
  if [[ "${survives}" == yes ]]; then
    catch_up_done $((BIT_FAULT + BIT_SENTINEL_B)) PLC_SENTINEL_B
    frame="$(frame_of "${FAULT}")"
    check_frame "${frame}" ".[\"x-medkit\"].full_data.fault_code == ${marker}" "${name}: the frame survives the restart"
    check_frame "${frame}" ".[\"x-medkit\"].captured_at == \"${captured}\"" "${name}: captured_at is kept"
  else
    frame="$(wait_frame "${FAULT}" '.["x-medkit"].capture_origin == "startup"')"
    check_frame "${frame}" ".[\"x-medkit\"].full_data.fault_code == $((marker + 1))" "${name}: the frame is read again after the restart"
    check_frame "${frame}" ".[\"x-medkit\"].captured_at > \"${captured}\"" "${name}: with a newer captured_at"
  fi
}
in_gw "mkdir -p ${STATE_DIR}/triggers"
storage_case next-to-triggers /config/params_next_to_triggers.yaml 400 \
  "Entity freeze-frames persisted in ${STATE_DIR}/triggers/entity_freeze_frames.db" yes
storage_case no-store /config/params_no_store.yaml 500 \
  "Entity freeze-frames are not persisted (no entity_freeze_frame.storage.path and no triggers.storage.path); they are lost on restart" no
storage_case unopenable /config/params_unopenable.yaml 600 \
  "Entity freeze-frame store '${STATE_DIR}/no-such-dir/entity_freeze_frames.db' could not be opened, frames stay in memory" no

echo "[5/5] Subscription mode with a publishing interval of 60 s (scenario 7)"
# A subscription publishes its first values one interval after it is created.
# GET /data must serve values within 15 s, and the one catch-up read must find them.
SUB_STORE="${STATE_DIR}/sub/entity_freeze_frames.db"
sub_params() {
  base_params | sed 's/plugins.opcua.poll_interval_ms: 1000/plugins.opcua.poll_interval_ms: 60000/'
  cat <<EOF
    plugins.opcua.prefer_subscriptions: true
    plugins.opcua.subscription_interval_ms: 60000
    entity_freeze_frame:
      storage:
        path: "${SUB_STORE}"
EOF
}
{
  base_params
  cat <<EOF
    entity_freeze_frame:
      storage:
        path: "${SUB_STORE}"
EOF
} >"${CONFIG_DIR}/params_sub_first.yaml"
sub_params >"${CONFIG_DIR}/params_sub60.yaml"
# A 1 s poll, so the gateway sees a dropped link within a second. Values still
# arrive only through the subscription and the read before each subscribe.
{
  base_params
  cat <<EOF
    plugins.opcua.prefer_subscriptions: true
    plugins.opcua.subscription_interval_ms: 60000
EOF
} >"${CONFIG_DIR}/params_sub60_reconnect.yaml"
docker cp "${CONFIG_DIR}/." "${GATEWAY_NAME}:/config/"
gw_stop
fm_stop
in_gw "mkdir -p ${STATE_DIR}/sub"
fm_start "${STATE_DIR}/faults-sub.db"
first_life_with_frame sub /config/params_sub_first.yaml 900
gw_stop
fm_clear "${FAULT}"
fm_report "${FAULT}"
server set FaultCode 950
gw_start sub-60 /config/params_sub60.yaml
wait_for "${API}/apps/${ENTITY}/data" '.items[] | select(.id == "fault_code") | .value == 950' 15
ok "GET /data serves the current value (950) within 15 s in subscription mode"
wait_log "Entity freeze-frame for fault '${FAULT}': the stored frame of entity '${ENTITY}' is from an earlier occurrence and was discarded (stored occurrence 1, the fault manager lists occurrence 2, CONFIRMED)" 15
frame="$(wait_frame "${FAULT}" '.["x-medkit"].capture_origin == "startup"')"
check_frame "${frame}" '.["x-medkit"].full_data.fault_code == 950' "subscription mode: the startup frame holds the current value (950)"

echo "  [scenario 7] the server is down when the gateway starts"
STATUS_URL="${API}/components/ff_restart_runtime/x-plc-status"
gw_stop
docker network disconnect "${NET_NAME}" "${SERVER_NAME}"
server set FaultCode 960
wait_server_log "OK FaultCode=960"
gw_launch sub-60-server-down /config/params_sub60_reconnect.yaml
# The poller tries at once, then waits 5 s and 10 s: bring the server up early.
wait_log "OPC-UA poller started (mode: poll)" 90
docker network connect "${NET_NAME}" "${SERVER_NAME}"
wait_for "${API}/apps/${ENTITY}/data" '.items[] | select(.id == "fault_code") | .value == 960' 15
ok "GET /data serves 960 within 15 s of the server coming up"
wait_for "${STATUS_URL}" '.mode == "subscription"' 15
ok "the plugin runs in subscription mode"

echo "  [scenario 7] the server restarts with a new value"
docker rm -f "${SERVER_NAME}" >/dev/null
# The old docker client must let go of the FIFO before the new server reads it.
wait "${SERVER_DOCKER_PID}" 2>/dev/null || true
wait_for "${STATUS_URL}" '.connected == false' 30
ok "the gateway saw the server go"
server_run
server set FaultCode 970
wait_server_log "OK FaultCode=970"
docker network connect "${NET_NAME}" "${SERVER_NAME}"
wait_for "${API}/apps/${ENTITY}/data" '.items[] | select(.id == "fault_code") | .value == 970' 15
ok "GET /data serves the new value (970) within 15 s of the server coming back"
wait_for "${STATUS_URL}" '.mode == "subscription"' 15
ok "the plugin runs in subscription mode"

echo "All freeze-frame restart scenarios passed."
exec 3>&-
