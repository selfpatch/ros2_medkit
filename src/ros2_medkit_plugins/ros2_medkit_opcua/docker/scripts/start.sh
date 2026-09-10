#!/usr/bin/env bash
# Start OpenPLC + medkit gateway for manual testing.
# Usage: from the ros2_medkit repo root, run
#     bash src/ros2_medkit_plugins/ros2_medkit_opcua/docker/scripts/start.sh
#
# The gateway container also runs a fault_manager_node, so an alarm the PLC
# raises becomes a fault the SOVD API serves instead of a 503.
#
# The gateway's state (entity freeze frames, faults.db, rosbags) is kept on the
# named volume below, so it survives stop.sh and a later start.sh. Purge it with
#     docker volume rm ros2-medkit-opcua-state
set -eo pipefail

STATE_VOLUME="${OPCUA_DEMO_STATE_VOLUME:-ros2-medkit-opcua-state}"

# Printed by the gateway container once fault_manager_node has advertised its
# services, and grepped for below. The container is where the wait happens, this
# is how the host learns the outcome.
FM_READY_MARKER="fault_manager ready: /fault_manager/report_fault"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOCKER_DIR="$(dirname "$SCRIPT_DIR")"
PLUGIN_DIR="$(dirname "$DOCKER_DIR")"
# Repo root = 4 levels up from script (scripts -> docker -> ros2_medkit_opcua
# -> ros2_medkit_plugins -> src -> repo root)
REPO_ROOT="$(cd "$PLUGIN_DIR/../../.." && pwd)"

echo "=== Building OpenPLC ==="
docker build -t openplc-tank "$DOCKER_DIR/openplc" 2>&1 | tail -3

echo ""
echo "=== Building gateway + OPC-UA plugin ==="
cd "$REPO_ROOT"
docker build -f "$DOCKER_DIR/Dockerfile.gateway" -t gateway-opcua . 2>&1 | tail -5

echo ""
echo "=== Starting containers ==="
docker rm -f openplc gateway 2>/dev/null || true
docker network create plc-demo 2>/dev/null || true
# The gateway's state goes on a named volume rather than the container's
# writable layer, because stop.sh removes the container. Without this a
# freeze-frame captured when the alarm confirmed would be destroyed by the only
# stop procedure the demo ships, and the next start would re-read the PLC as it
# is then instead of serving the values frozen at fault time.
docker volume create "$STATE_VOLUME" >/dev/null

docker run -d --name openplc --network plc-demo -p 4840:4840 openplc-tank
echo "OpenPLC starting..."
for _ in $(seq 1 45); do
    if docker logs openplc 2>&1 | grep -q "PLC State: RUNNING"; then
        echo "  OpenPLC running"
        break
    fi
    sleep 2
done

docker run -d --name gateway --network plc-demo -p 8080:8080 \
    -v "$STATE_VOLUME":/var/lib/ros2_medkit \
    -e ROS_DOMAIN_ID=60 \
    -e OPCUA_ENDPOINT_URL="opc.tcp://openplc:4840/openplc/opcua" \
    -e OPCUA_NODE_MAP_PATH="/config/tank_nodes.yaml" \
    gateway-opcua \
    bash -c "
      mkdir -p /var/lib/ros2_medkit/rosbags /config
      echo 'manifest_version: \"1.0\"' > /config/manifest.yaml
      source /opt/ros/jazzy/setup.bash && source /root/ws/install/setup.bash
      # A fault manager runs beside the gateway, in the same container and on
      # the same ROS domain. Without one the alarms the OPC-UA plugin detects
      # have nowhere to go: /api/v1/faults answers 503 and the demo can show
      # live PLC values but never a fault. It is started before gateway_node so
      # its services are advertised before the plugin calls
      # /fault_manager/report_fault.
      #
      # database_path is passed explicitly rather than left to the default: it
      # has to land on /var/lib/ros2_medkit, the named volume, so faults.db
      # outlives the container the way the entity freeze frames already do.
      # Rosbag capture is opt-in and stays off here. storage_path names the
      # same volume, so a recording lands there whenever it is switched on.
      ros2 run ros2_medkit_fault_manager fault_manager_node --ros-args \
        -p database_path:=/var/lib/ros2_medkit/faults.db \
        -p snapshots.rosbag.storage_path:=/var/lib/ros2_medkit/rosbags \
        > /var/lib/ros2_medkit/fault_manager.log 2>&1 &
      # Poll for the service instead of sleeping a fixed time: 'ros2 service
      # list' is the cheapest ROS-native availability signal, and a fixed sleep
      # is either too short on a loaded machine or wasted time on a fast one.
      # Running the poll before gateway_node is what makes it mean anything.
      # 'ros2 service list' also reports a name that only a client has opened,
      # and the gateway opens clients for exactly these services, so the same
      # check made after the gateway is up would pass with nothing serving it.
      for _ in \$(seq 1 50); do
        if ros2 service list 2>/dev/null | grep -q '/fault_manager/report_fault'; then
          break
        fi
        sleep 0.2
      done
      if ! ros2 service list 2>/dev/null | grep -q '/fault_manager/report_fault'; then
        echo 'ERROR: fault_manager_node did not advertise /fault_manager/report_fault within 10s.' >&2
        echo 'Last lines of /var/lib/ros2_medkit/fault_manager.log:' >&2
        tail -n 20 /var/lib/ros2_medkit/fault_manager.log >&2 || true
        exit 1
      fi
      echo '$FM_READY_MARKER'
      PLUGIN_PATH=\$(find /root/ws/install -name 'libros2_medkit_opcua_plugin.so' | head -1)
      ros2 run ros2_medkit_gateway gateway_node \
        --ros-args --params-file /config/gateway_params.yaml \
        -p plugins.opcua.path:=\$PLUGIN_PATH \
        -p discovery.mode:=hybrid \
        -p discovery.manifest_path:=/config/manifest.yaml \
        -p discovery.manifest_strict_validation:=false"

echo "Fault manager starting..."
fm_ready=0
for _ in $(seq 1 60); do
    # Substring match rather than a pipe into grep -q: grep -q closes the pipe
    # on the first hit, and under `set -o pipefail` the SIGPIPE'd `docker logs`
    # would turn a found marker into a failed test.
    if [[ "$(docker logs gateway 2>&1)" == *"$FM_READY_MARKER"* ]]; then
        fm_ready=1
        echo "  Fault manager ready (log: /var/lib/ros2_medkit/fault_manager.log)"
        break
    fi
    # The container exits when the wait above timed out. Stop polling for a
    # marker that can no longer arrive and report it below.
    if [ -z "$(docker ps -q --filter 'name=^gateway$')" ]; then
        break
    fi
    sleep 1
done
if [ "$fm_ready" -eq 0 ]; then
    echo "ERROR: the fault manager never advertised /fault_manager/report_fault." >&2
    echo "The demo needs it: without it the alarms the PLC raises are dropped and" >&2
    echo "/api/v1/faults answers 503. Gateway container log:" >&2
    docker logs gateway 2>&1 | tail -20 >&2
    exit 1
fi

echo "Gateway starting..."

for _ in $(seq 1 30); do
    if curl -sf http://localhost:8080/api/v1/apps 2>/dev/null | jq -e '.items | map(.id) | contains(["tank_process"])' >/dev/null 2>&1; then
        echo ""
        echo "============================================"
        echo "  Ready! Gateway on http://localhost:8080"
        echo "============================================"
        echo ""
        echo "Stop:  bash scripts/stop.sh"
        echo "Tests: bash scripts/run_integration_tests.sh"
        echo "State: volume '$STATE_VOLUME' (kept across stop/start)"
        echo "Faults: fault_manager_node runs in the gateway container"
        echo "        (log: docker exec gateway cat /var/lib/ros2_medkit/fault_manager.log)"
        exit 0
    fi
    sleep 2
done

echo "WARNING: PLC entities not yet discovered"
