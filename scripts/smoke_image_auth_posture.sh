#!/bin/bash
# Copyright 2026 bburda
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Checks what a built image serves: the entrypoint, the config layers and the
# MEDKIT_* variables together. colcon tests run the node directly and reach none
# of that.
#
# Usage: scripts/smoke_image_auth_posture.sh <image reference>
#        SMOKE_PREFIX=<container name prefix> (default medkit-smoke-<pid>-<random>,
#        unique per run: the run removes only the containers it created)
#
# Two instruments, both read from the running gateway and not from the command
# given to it: the HTTP answer, fetched by a curl container on the run's own
# network, and the startup line in `docker logs`:
#
#   Configuration: REST API at 0.0.0.0:8080, backstop refresh interval: 2000ms
#
# Every refusal is paired with an answer in the same run (the open container,
# the token, an allowed origin), so "nothing answered" cannot pass for "refused".
#
# Probes run inside the network because neither a published port nor a bridge
# address is reachable from every shell that has a docker client. The network is
# private to the run and so is the ROS domain, so no other stack sees these
# gateways. Files go in with `docker cp` because a bind-mount source is resolved
# on the daemon's host.

set -euo pipefail

IMAGE="${1:?usage: $0 <image reference>}"
PREFIX="${SMOKE_PREFIX:-medkit-smoke-$$-$RANDOM}"
SECRET="smoke_image_posture_secret_of_at_least_32_chars"
CLIENT_ID="smoke"
CLIENT_SECRET="smoke_client_secret"
CLIENTS="${CLIENT_ID}:${CLIENT_SECRET}:admin"
WORKDIR="$(mktemp -d)"
DEADLINE_S=120
NET="${PREFIX}-net"
CURL="${PREFIX}-curl"
export ROS_DOMAIN_ID=$((RANDOM % 100 + 1))

# Names of the containers this run created. Appended to in the main shell only
# (never inside a command substitution), so the trap sees all of them.
CREATED=()

cleanup() {
  for name in "${CREATED[@]}"; do
    docker rm -f "$name" >/dev/null 2>&1 || true
  done
  docker network rm "$NET" >/dev/null 2>&1 || true
  rm -rf "$WORKDIR"
}
trap cleanup EXIT

passed=0
failures=0

ok() { echo "ok   $1"; passed=$((passed + 1)); }
fail() { echo "FAIL $1" >&2; failures=$((failures + 1)); }

expect() {
  local label=$1 want=$2 got=$3
  if [ "$got" = "$want" ]; then ok "$label -> $got"; else fail "$label -> $got, expected $want"; fi
}

expect_contains() {
  local label=$1 want=$2 got=$3
  if [ "${got#*"$want"}" != "$got" ]; then
    ok "$label -> $want"
  else
    fail "$label -> ${got:-<nothing>}, expected to contain $want"
  fi
}

# Runs curl in the probe container, with bounds so that a gateway that accepts
# and never answers cannot hang the run. The exec is detached and its answer goes through a file: a foreground `docker exec`
# ends early and loses stdout that is written more than about a second after it starts,
# and a slow gateway answer is one. The poll ends at the curl bound plus a margin.
curl_in() {
  local rc end=$((SECONDS + 60))
  docker exec "$CURL" rm -f /tmp/out /tmp/rc
  docker exec -d "$CURL" sh -c 'curl -s --connect-timeout 2 --max-time 5 "$@" >/tmp/out 2>/dev/null; echo $? >/tmp/rc' \
    sh "$@"
  until docker exec "$CURL" test -s /tmp/rc; do
    [ "$SECONDS" -lt "$end" ] || return 1
    sleep 0.1
  done
  docker exec "$CURL" cat /tmp/out
  rc=$(docker exec "$CURL" cat /tmp/rc)
  return "${rc:-1}"
}

# Starts a container. start_container <name> [-e VAR=val ...] [-- <args>]
# COPY_FILE/COPY_DEST: a file copied in before the container starts.
start_container() {
  local name=$1
  shift
  local flags=()
  while [ $# -gt 0 ] && [ "$1" != "--" ]; do
    flags+=("$1")
    shift
  done
  [ $# -gt 0 ] && shift
  if docker inspect "$name" >/dev/null 2>&1; then
    echo "container name $name is already in use; set SMOKE_PREFIX" >&2
    exit 2
  fi
  docker create --name "$name" --network "$NET" -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
    "${flags[@]}" "$IMAGE" "$@" >/dev/null
  CREATED+=("$name")
  if [ -n "${COPY_FILE:-}" ]; then
    docker cp "$COPY_FILE" "$name:${COPY_DEST:?COPY_DEST required with COPY_FILE}"
  fi
  docker start "$name" >/dev/null
}

# Polls until the gateway answers anything on <port>; prints the status code of
# GET <path>, or 000 after the deadline.
wait_code() {
  local name=$1 port=$2 path=$3
  local code="000" end=$((SECONDS + DEADLINE_S))
  while [ "$SECONDS" -lt "$end" ]; do
    code=$(curl_in -o /dev/null -w '%{http_code}' "http://$name:$port$path") || true
    [ -n "$code" ] && [ "$code" != "000" ] && break
    sleep 1
  done
  if [ "$code" = "000" ]; then
    echo "$name: never answered on $name:$port" >&2
    docker logs "$name" >&2 2>&1 || true
  fi
  echo "$code"
}

# Polls `docker logs` for the gateway's startup line and prints it.
startup_line() {
  local name=$1 line="" end=$((SECONDS + DEADLINE_S))
  while [ "$SECONDS" -lt "$end" ]; do
    line=$(docker logs "$name" 2>&1 | grep -m1 'Configuration: REST API at' || true)
    [ -n "$line" ] && break
    sleep 1
  done
  echo "$line"
}

# Polls `docker logs` for the gateway's ready line and prints it, or nothing after the deadline.
ready_line() {
  local name=$1 line="" end=$((SECONDS + DEADLINE_S))
  while [ "$SECONDS" -lt "$end" ]; do
    line=$(docker logs "$name" 2>&1 | grep -m1 'Medkit Gateway ready on' || true)
    [ -n "$line" ] && break
    sleep 1
  done
  echo "$line"
}

# Prints the Access-Control-Allow-Origin value the gateway returns for a request
# carrying the given Origin, or "<none>" when the header is absent. A 000 answer
# is reported as "<no answer>" so it never reads as "<none>".
acao_for() {
  local name=$1 origin=$2 headers
  headers=$(curl_in -D - -o /dev/null -H "Origin: $origin" "http://$name:8080/api/v1/health") || true
  if [ -z "$headers" ]; then echo "<no answer>"; return; fi
  echo "$headers" | tr -d '\r' | sed -n 's/^[Aa]ccess-[Cc]ontrol-[Aa]llow-[Oo]rigin:[[:space:]]*//p' | head -1 \
    | { read -r v || true; echo "${v:-<none>}"; }
}

token_for() {
  curl_in -X POST "http://$1:8080/api/v1/auth/authorize" \
    -H 'Content-Type: application/json' \
    -d "{\"grant_type\":\"client_credentials\",\"client_id\":\"$CLIENT_ID\",\"client_secret\":\"$CLIENT_SECRET\"}" \
    | sed -n 's/.*"access_token"[[:space:]]*:[[:space:]]*"\([^"]*\)".*/\1/p'
}

# GET /areas with the token from MEDKIT_CLIENTS: proves the credential reached
# the gateway, which a container closed to everyone, its operator included,
# does not.
expect_token_works() {
  local label=$1 name=$2 token code
  token=$(token_for "$name")
  if [ -z "$token" ]; then
    fail "$label: POST /auth/authorize issued no token"
    return
  fi
  code=$(curl_in -o /dev/null -w '%{http_code}' -H "Authorization: Bearer $token" "http://$name:8080/api/v1/areas") || true
  expect "$label: GET /areas with the token" 200 "$code"
}

# Checks beyond GET /areas for a closed container: the health probe is refused,
# and so is a write. DELETE /api/v1/faults is the write because it exists on a
# gateway with nothing in it (a fixed route, no entity needed) and needs the
# OPERATOR role, which the admin client has. With the token it must answer
# something other than 401.
expect_closed_routes() {
  local label=$1 name=$2 token code
  code=$(curl_in -o /dev/null -w '%{http_code}' "http://$name:8080/api/v1/health") || true
  expect "$label: anonymous GET /health" 401 "$code"
  code=$(curl_in -o /dev/null -w '%{http_code}' -X DELETE "http://$name:8080/api/v1/faults") || true
  expect "$label: anonymous DELETE /faults" 401 "$code"
  token=$(token_for "$name")
  # A gateway with no fault manager answers 503 after about 5 s, so this call gets a longer bound.
  code=$(curl_in --max-time 20 -o /dev/null -w '%{http_code}' -X DELETE -H "Authorization: Bearer $token" \
    "http://$name:8080/api/v1/faults") || true
  if [ -n "$token" ] && [[ "$code" =~ ^[1-5][0-9][0-9]$ ]] && [ "$code" != "401" ]; then
    ok "$label: DELETE /faults with the token -> $code (not 401)"
  else
    fail "$label: DELETE /faults with the token -> ${code:-<none>}, token '${token:+issued}'"
  fi
}

# A file that turns authentication off and names a secret of its own.
cat > "$WORKDIR/auth-off.yaml" <<'YAML'
ros2_medkit_gateway:
  ros__parameters:
    auth:
      enabled: false
      require_auth_for: "write"
      jwt_secret: "a_file_supplied_secret_of_at_least_32_characters"
YAML
cat > "$WORKDIR/refresh.yaml" <<'YAML'
ros2_medkit_gateway:
  ros__parameters:
    refresh_interval_ms: 4321
YAML
cat > "$WORKDIR/host.yaml" <<'YAML'
ros2_medkit_gateway:
  ros__parameters:
    server:
      host: "127.0.0.1"
YAML
cat > "$WORKDIR/cors.yaml" <<'YAML'
ros2_medkit_gateway:
  ros__parameters:
    cors:
      allowed_origins: ["https://my-dashboard.example.com"]
YAML
# Layer merge: a wildcard default with a node override after it, a node section
# only, and a wildcard section only. Each sets a value the container layer sets too.
cat > "$WORKDIR/wild-and-node.yaml" <<'YAML'
/**:
  ros__parameters:
    server:
      host: "0.0.0.0"
    refresh_interval_ms: 3333
/ros2_medkit_gateway:
  ros__parameters:
    server:
      host: "127.0.0.1"
YAML
sed 's|^/ros2_medkit_gateway:|ros2_medkit_gateway:|' "$WORKDIR/wild-and-node.yaml" > "$WORKDIR/wild-and-bare-node.yaml"
cat > "$WORKDIR/node-only.yaml" <<'YAML'
/ros2_medkit_gateway:
  ros__parameters:
    refresh_interval_ms: 1111
YAML
cat > "$WORKDIR/wild-only.yaml" <<'YAML'
/**:
  ros__parameters:
    refresh_interval_ms: 2222
YAML
cat > "$WORKDIR/renamed.yaml" <<'YAML'
/**:
  ros__parameters:
    server:
      port: 8081
/renamed:
  ros__parameters:
    refresh_interval_ms: 5555
YAML
chmod 755 "$WORKDIR"
chmod a+r "$WORKDIR"/*.yaml

docker network create "$NET" >/dev/null
docker create --name "$CURL" --network "$NET" --entrypoint sleep "$IMAGE" infinity >/dev/null
CREATED+=("$CURL")
docker start "$CURL" >/dev/null

# The image's own configuration, read out of the image under test. Config.Cmd is
# empty and the entrypoint carries the layers; container.yaml lists exactly the
# two default web UI origins, so an added origin fails here.
cmd_json=$(docker image inspect -f '{{json .Config.Cmd}}' "$IMAGE")
case "$cmd_json" in
  null|'[]') ok "image Config.Cmd is empty -> $cmd_json" ;;
  *) fail "image Config.Cmd -> $cmd_json, expected empty" ;;
esac
expect "image Config.Entrypoint" '["/entrypoint.sh"]' "$(docker image inspect -f '{{json .Config.Entrypoint}}' "$IMAGE")"
docker create --name "${PREFIX}-config" "$IMAGE" >/dev/null
CREATED+=("${PREFIX}-config")
docker cp "${PREFIX}-config:/etc/ros2_medkit/container.yaml" "$WORKDIR/container.yaml"
origins=$(sed -n '/allowed_origins:/,$p' "$WORKDIR/container.yaml" | sed -n 's/^[[:space:]]*-[[:space:]]*//p' | tr -d '"' | tr '\n' ' ')
expect "image container.yaml cors.allowed_origins" "http://localhost:3000 http://localhost:5173 " "$origins"

# C2: no environment, the image answers anonymous requests.
start_container "${PREFIX}-open" --
expect "no environment: anonymous GET /areas" 200 "$(wait_code "${PREFIX}-open" 8080 /api/v1/areas)"
expect_contains "no environment: startup line" "REST API at 0.0.0.0:8080, backstop refresh interval: 2000ms" \
  "$(startup_line "${PREFIX}-open")"

# C3: secret and clients close the image; the credential still gets in.
start_container "${PREFIX}-closed" -e MEDKIT_JWT_SECRET="$SECRET" -e MEDKIT_CLIENTS="$CLIENTS" --
expect "MEDKIT_JWT_SECRET: anonymous GET /areas" 401 "$(wait_code "${PREFIX}-closed" 8080 /api/v1/areas)"
expect_token_works "MEDKIT_CLIENTS" "${PREFIX}-closed"
expect_closed_routes "MEDKIT_JWT_SECRET" "${PREFIX}-closed"

# C3: the environment beats a mounted file that says auth off.
COPY_FILE="$WORKDIR/auth-off.yaml" COPY_DEST=/etc/ros2_medkit/params.yaml \
  start_container "${PREFIX}-override" -e MEDKIT_JWT_SECRET="$SECRET" -e MEDKIT_CLIENTS="$CLIENTS" --
expect "environment over a mounted file with auth off: anonymous GET /areas" 401 \
  "$(wait_code "${PREFIX}-override" 8080 /api/v1/areas)"
expect_token_works "environment over a mounted file" "${PREFIX}-override"
expect_closed_routes "environment over a mounted file" "${PREFIX}-override"

# C4: MEDKIT_AUTH_DISABLED=1 opens the image even with a secret.
start_container "${PREFIX}-disabled" -e MEDKIT_JWT_SECRET="$SECRET" -e MEDKIT_CLIENTS="$CLIENTS" \
  -e MEDKIT_AUTH_DISABLED=1 --
expect "MEDKIT_AUTH_DISABLED=1 with a secret: anonymous GET /areas" 200 \
  "$(wait_code "${PREFIX}-disabled" 8080 /api/v1/areas)"

# C1: a mounted file wins over the container layer (refresh_interval_ms), keeps
# the keys it does not name (server.host, CORS origins).
COPY_FILE="$WORKDIR/refresh.yaml" COPY_DEST=/etc/ros2_medkit/params.yaml start_container "${PREFIX}-mount" --
expect "mounted file: anonymous GET /areas" 200 "$(wait_code "${PREFIX}-mount" 8080 /api/v1/areas)"
expect_contains "mounted file: refresh_interval_ms and untouched server.host" \
  "REST API at 0.0.0.0:8080, backstop refresh interval: 4321ms" "$(startup_line "${PREFIX}-mount")"
expect "mounted file without cors: Origin http://localhost:3000" "http://localhost:3000" \
  "$(acao_for "${PREFIX}-mount" http://localhost:3000)"

# C1: a mounted file can set server.host again. The startup line is the proof;
# a gateway bound to loopback cannot be reached from the bridge.
COPY_FILE="$WORKDIR/host.yaml" COPY_DEST=/etc/ros2_medkit/params.yaml start_container "${PREFIX}-hostmount" --
expect_contains "mounted file: server.host" "REST API at 127.0.0.1:8080" "$(startup_line "${PREFIX}-hostmount")"

# Layer merge: a node section after a wildcard section wins key by key, and the
# wildcard keys it does not name still apply.
COPY_FILE="$WORKDIR/wild-and-node.yaml" COPY_DEST=/etc/ros2_medkit/params.yaml \
  start_container "${PREFIX}-wildnode" -e MEDKIT_AUTH_DISABLED=1 --
expect_contains "mounted wildcard plus node: node host wins, wildcard refresh applies" \
  "REST API at 127.0.0.1:8080, backstop refresh interval: 3333ms" "$(startup_line "${PREFIX}-wildnode")"
expect_contains "mounted wildcard plus node: gateway reaches the ready line" "ready on" "$(ready_line "${PREFIX}-wildnode")"

COPY_FILE="$WORKDIR/wild-and-bare-node.yaml" COPY_DEST=/etc/ros2_medkit/params.yaml \
  start_container "${PREFIX}-wildbare" -e MEDKIT_AUTH_DISABLED=1 --
expect_contains "mounted wildcard plus node (name without slash): node host wins" \
  "REST API at 127.0.0.1:8080, backstop refresh interval: 3333ms" "$(startup_line "${PREFIX}-wildbare")"
expect_contains "mounted wildcard plus node (name without slash): ready line" "ready on" "$(ready_line "${PREFIX}-wildbare")"

# Layer merge: a node-only file and a wildcard-only file each override the container layer.
COPY_FILE="$WORKDIR/node-only.yaml" COPY_DEST=/etc/ros2_medkit/params.yaml \
  start_container "${PREFIX}-nodeonly" -e MEDKIT_AUTH_DISABLED=1 --
expect_contains "mounted node section only" \
  "REST API at 0.0.0.0:8080, backstop refresh interval: 1111ms" "$(startup_line "${PREFIX}-nodeonly")"
expect_contains "mounted node section only: ready line" "ready on" "$(ready_line "${PREFIX}-nodeonly")"
COPY_FILE="$WORKDIR/wild-only.yaml" COPY_DEST=/etc/ros2_medkit/params.yaml \
  start_container "${PREFIX}-wildonly" -e MEDKIT_AUTH_DISABLED=1 --
expect_contains "mounted wildcard section only" \
  "REST API at 0.0.0.0:8080, backstop refresh interval: 2222ms" "$(startup_line "${PREFIX}-wildonly")"
expect_contains "mounted wildcard section only: ready line" "ready on" "$(ready_line "${PREFIX}-wildonly")"

# Layer merge: a section for another node name reaches a node renamed to it, merged with `/**`.
COPY_FILE="$WORKDIR/renamed.yaml" COPY_DEST=/etc/ros2_medkit/params.yaml \
  start_container "${PREFIX}-renamed" -e MEDKIT_AUTH_DISABLED=1 -- --ros-args -r __node:=renamed
expect_contains "renamed node: its own section and the wildcard reach it" \
  "backstop refresh interval: 5555ms" "$(startup_line "${PREFIX}-renamed")"
expect_contains "renamed node: wildcard port" "REST API at 127.0.0.1:8081" "$(startup_line "${PREFIX}-renamed")"
expect_contains "renamed node: ready line" "ready on" "$(ready_line "${PREFIX}-renamed")"

# A read-only root filesystem: only the directory ROS writes its logs to is writable.
GW_UID=$(docker run --rm --entrypoint id "$IMAGE" -u)
start_container "${PREFIX}-readonly" -e MEDKIT_AUTH_DISABLED=1 --read-only \
  --tmpfs "/home/medkit/.ros/log:uid=$GW_UID" --
expect_contains "read-only root filesystem: gateway reaches the ready line" "ready on" "$(ready_line "${PREFIX}-readonly")"
expect "read-only root filesystem: anonymous GET /areas" 200 "$(wait_code "${PREFIX}-readonly" 8080 /api/v1/areas)"

# C1: an argument-only override keeps the container layer.
start_container "${PREFIX}-argonly" -- --ros-args -p server.port:=9090
expect "arg-only override: anonymous GET /areas on 9090" 200 "$(wait_code "${PREFIX}-argonly" 9090 /api/v1/areas)"
expect_contains "arg-only override: bind address and port" "REST API at 0.0.0.0:9090" \
  "$(startup_line "${PREFIX}-argonly")"

# C5: default CORS origins.
start_container "${PREFIX}-cors" --
expect "default CORS: container answers" 200 "$(wait_code "${PREFIX}-cors" 8080 /api/v1/health)"
for origin in http://localhost:3000 http://localhost:5173; do
  expect "default CORS: Origin $origin" "$origin" "$(acao_for "${PREFIX}-cors" "$origin")"
done
expect "default CORS: Origin http://evil.example" "<none>" "$(acao_for "${PREFIX}-cors" http://evil.example)"

# C5: a mounted file that sets cors.allowed_origins replaces the list.
COPY_FILE="$WORKDIR/cors.yaml" COPY_DEST=/etc/ros2_medkit/params.yaml start_container "${PREFIX}-corsmount" --
expect "mounted CORS list: container answers" 200 "$(wait_code "${PREFIX}-corsmount" 8080 /api/v1/health)"
expect "mounted CORS list: its own origin" "https://my-dashboard.example.com" \
  "$(acao_for "${PREFIX}-corsmount" https://my-dashboard.example.com)"
expect "mounted CORS list: default origin is gone" "<none>" \
  "$(acao_for "${PREFIX}-corsmount" http://localhost:3000)"

# C9: a command that does not start with a flag runs as given, the way a
# consumer runs the image through `bash -c 'exec ros2 launch ...'`. The layers do
# not apply to it; the node still reads the MEDKIT_* variables itself.
LAUNCH_CMD='exec ros2 launch ros2_medkit_gateway bringup.launch.py enable_diagnostic_bridge:=true server_host:=0.0.0.0 cors_allowed_origins:=https://example.test'
start_container "${PREFIX}-launch" -- bash -c "$LAUNCH_CMD"
expect "launch command: anonymous GET /health" 200 "$(wait_code "${PREFIX}-launch" 8080 /api/v1/health)"
expect "launch command: its own origin" "https://example.test" "$(acao_for "${PREFIX}-launch" https://example.test)"
expect "launch command: layer origin is not applied" "<none>" "$(acao_for "${PREFIX}-launch" http://localhost:3000)"
# The open launch container runs its own /fault_manager; it goes before the next one starts.
docker stop -t 30 "${PREFIX}-launch" >/dev/null
start_container "${PREFIX}-launchclosed" -e MEDKIT_JWT_SECRET="$SECRET" -e MEDKIT_CLIENTS="$CLIENTS" -- bash -c "$LAUNCH_CMD"
expect "launch command with MEDKIT_JWT_SECRET: anonymous GET /areas" 401 "$(wait_code "${PREFIX}-launchclosed" 8080 /api/v1/areas)"
expect_token_works "launch command with MEDKIT_CLIENTS" "${PREFIX}-launchclosed"
expect_closed_routes "launch command with MEDKIT_JWT_SECRET" "${PREFIX}-launchclosed"

# The gateway gets the container's SIGTERM itself and shuts down cleanly: exit
# code 0, its own shutdown line in the log, and `docker stop` returns well before
# its timeout (a SIGKILL after the timeout exits 137).
start_container "${PREFIX}-stop" --
expect "stop: container answers" 200 "$(wait_code "${PREFIX}-stop" 8080 /api/v1/health)"
stop_timeout=30
t0=$(date +%s%N)
docker stop -t "$stop_timeout" "${PREFIX}-stop" >/dev/null
elapsed_ms=$(( ($(date +%s%N) - t0) / 1000000 ))
expect "stop: exit code" 0 "$(docker inspect -f '{{.State.ExitCode}}' "${PREFIX}-stop")"
expect_contains "stop: gateway's shutdown line" "Shutting down ROS 2 Medkit Gateway" "$(docker logs "${PREFIX}-stop" 2>&1)"
if [ "$elapsed_ms" -lt $((stop_timeout * 1000 / 2)) ]; then
  ok "stop: returned in ${elapsed_ms} ms, before the ${stop_timeout} s timeout"
else
  fail "stop: took ${elapsed_ms} ms of a ${stop_timeout} s timeout"
fi

echo "image posture: $passed passed, $failures failed"
[ "$failures" -eq 0 ]
