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
# Check what a BUILT IMAGE actually serves.
#
# Usage: scripts/smoke_image_auth_posture.sh <image reference>
#
# WHAT THIS DRIVES, and what it does not. The rule the gateway applies to the
# three MEDKIT_* variables is covered by colcon tests, which run the node
# directly. This drives the image around it: the entrypoint exporting the
# variables and prepending the three config layers, the layers themselves, and
# the packaged config they combine with. A colcon test reaches none of that.
#
# Two instruments. The posture cases read an HTTP status code from the
# container's bridge address. The layering cases read the gateway's own startup
# line out of `docker logs`, which carries the bind address, the port and the
# refresh interval in force:
#
#   Configuration: REST API at 0.0.0.0:8080, backstop refresh interval: 2000ms
#
#   1. no environment          -> anonymous GET /areas answers 200
#   2. MEDKIT_JWT_SECRET set   -> anonymous GET /areas answers 401,
#                                 and MEDKIT_CLIENTS obtains a working token
#   3. a mounted params file with auth off, plus MEDKIT_JWT_SECRET
#                              -> anonymous GET /areas answers 401
#   4. a file mounted at /etc/ros2_medkit/params.yaml changing
#      refresh_interval_ms -> the running gateway reports that value
#   5. an arg-only override (-p server.port:=9090)
#                              -> the gateway answers on 9090 and still binds
#                                 every interface
#
# Case 2 carries the token check because a container closed to its operator as
# well as to everyone else would pass the 401 on its own, and that is the
# failure an image ships silently: MEDKIT_CLIENTS never reaching the gateway
# looks identical from outside until somebody tries to log in.
#
# Case 3 is the one that says the environment beats the file. Without it, a
# gateway that consulted the file and deferred to it would pass cases 1 and 2
# while leaving a container its operator believes closed wide open.
#
# Cases 4 and 5 are the layering: the mounted file has to win, and an override
# has to change the key it names WITHOUT taking the layers with it.
#
# The gateway is reached at the container's bridge address, never at a
# published port on localhost: where the Docker daemon is not in this shell's
# network namespace, -p publishes somewhere this script cannot see and every
# request times out while the container is perfectly healthy.

set -euo pipefail

IMAGE="${1:?usage: $0 <image reference>}"
SECRET="smoke_image_posture_secret_of_at_least_32_chars"
CLIENT_ID="smoke"
CLIENT_SECRET="smoke_client_secret"
CLIENTS="${CLIENT_ID}:${CLIENT_SECRET}:admin"
WORKDIR="$(mktemp -d)"

# Fixed, not collected as the cases run: each case calls posture_of inside a
# command substitution to capture the status code, and a subshell cannot append
# to the parent's array - so a list built that way is empty when the trap fires
# and every container survives the run.
CONTAINERS=(medkit-smoke-open medkit-smoke-closed medkit-smoke-override
            medkit-smoke-mount medkit-smoke-argonly)

cleanup() {
  for name in "${CONTAINERS[@]}"; do
    docker rm -f "$name" >/dev/null 2>&1 || true
  done
  rm -rf "$WORKDIR"
}
trap cleanup EXIT

# The status code an anonymous GET /areas gets from a container, once it is
# answering at all. Prints the code on stdout; anything else goes to stderr so
# the caller can capture the one value.
#
# Called as: posture_of <name> <docker run flags...> -- <container args...>
# The container args replace the image CMD, which is how a case points the
# gateway at its own params file.
#
# COPY_FILE, when set, is copied into the container at COPY_DEST before it
# starts. A bind mount would be simpler and is wrong here: the daemon resolves
# a -v source path on ITS filesystem, so where the daemon is not in this
# shell's namespace the container gets nothing and the gateway dies parsing an
# absent file. `docker cp` streams the bytes through the API instead, which
# works either way.
posture_of() {
  local name=$1
  shift
  local flags=()
  while [ $# -gt 0 ] && [ "$1" != "--" ]; do
    flags+=("$1")
    shift
  done
  if [ $# -gt 0 ]; then
    shift  # drop the --
  fi
  docker rm -f "$name" >/dev/null 2>&1 || true
  docker create --name "$name" "${flags[@]}" "$IMAGE" "$@" >/dev/null
  if [ -n "${COPY_FILE:-}" ]; then
    docker cp "$COPY_FILE" "$name:${COPY_DEST:?COPY_DEST required with COPY_FILE}"
  fi
  docker start "$name" >/dev/null

  local ip
  ip=$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' "$name")
  if [ -z "$ip" ]; then
    echo "$name: container has no bridge address" >&2
    docker logs "$name" >&2 2>&1 || true
    echo "000"
    return 0
  fi

  # curl prints 000 itself when it cannot connect, and exits non-zero; the
  # `|| true` keeps that exit from ending the script and leaves the 000 for
  # the loop to retry on.
  # ARG_PORT lets a case that moved the port probe the port it moved it to.
  local port="${ARG_PORT:-8080}"
  local code="000"
  for _ in $(seq 1 60); do
    code=$(curl -s -o /dev/null -w '%{http_code}' "http://$ip:$port/api/v1/areas") || true
    [ -n "$code" ] && [ "$code" != "000" ] && break
    sleep 2
  done
  if [ "$code" = "000" ]; then
    echo "$name: never answered on http://$ip:$port" >&2
    docker logs "$name" >&2 2>&1 || true
  fi
  # Printed either way, 000 included. Returning non-zero here would abort the
  # whole script under `set -e` before `expect` could name the case, so a
  # container that never answered would end the run with no diagnostic at all.
  echo "$code"
}

# The label names the case AND the request, because not every case below is an
# anonymous GET /areas.
# Waits for the gateway's own startup line in `docker logs` and returns it.
#
# The startup line is the read-back instrument: the gateway prints it before it
# starts listening, so it is there once posture_of has had an answer, and it
# carries both values these cases are about:
#
#   Configuration: REST API at 0.0.0.0:8080, backstop refresh interval: 2000ms
startup_line() {
  local name=$1
  local line=""
  for _ in $(seq 1 60); do
    line=$(docker logs "$name" 2>&1 | grep -m1 'Configuration: REST API at' || true)
    [ -n "$line" ] && break
    sleep 2
  done
  echo "$line"
}

# Reports whether a startup line contains an expected fragment.
expect_in_startup() {
  local label=$1 want=$2 line=$3
  if [ "${line#*"$want"}" != "$line" ]; then
    echo "ok   $label -> $want"
  else
    echo "FAIL $label -> ${line:-<no startup line>}, expected to contain $want" >&2
    return 1
  fi
}

expect() {
  local label=$1 want=$2 got=$3
  if [ "$got" = "$want" ]; then
    echo "ok   $label -> $got"
  else
    echo "FAIL $label -> $got, expected $want" >&2
    return 1
  fi
}

# A params file that turns authentication OFF and names a secret of its own,
# which is exactly the file an entrypoint must not defer to when the
# environment says closed.
cat > "$WORKDIR/auth-off.yaml" <<'YAML'
ros2_medkit_gateway:
  ros__parameters:
    auth:
      enabled: false
      require_auth_for: "write"
      jwt_secret: "a_file_supplied_secret_of_at_least_32_characters"
YAML
# Readable and traversable by the image's non-root user.
chmod 755 "$WORKDIR"
chmod a+r "$WORKDIR/auth-off.yaml"

failures=0

open_code=$(posture_of medkit-smoke-open --)
expect "default, no environment: anonymous GET /areas" 200 "$open_code" \
  || failures=$((failures + 1))

closed_code=$(posture_of medkit-smoke-closed \
  -e MEDKIT_JWT_SECRET="$SECRET" -e MEDKIT_CLIENTS="$CLIENTS" --)
expect "MEDKIT_JWT_SECRET set: anonymous GET /areas" 401 "$closed_code" \
  || failures=$((failures + 1))

# The credential in MEDKIT_CLIENTS has to reach the gateway, or the container is
# closed to its operator as well as to everyone else - which looks identical
# from outside until somebody tries to log in. Run against the container the
# case above just measured, so the token is exchanged against that posture.
closed_ip=$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' \
  medkit-smoke-closed)
token=$(curl -s -X POST "http://$closed_ip:8080/api/v1/auth/authorize" \
  -H 'Content-Type: application/json' \
  -d "{\"grant_type\":\"client_credentials\",\"client_id\":\"$CLIENT_ID\",\"client_secret\":\"$CLIENT_SECRET\"}" \
  | sed -n 's/.*"access_token"[[:space:]]*:[[:space:]]*"\([^"]*\)".*/\1/p')
if [ -z "$token" ]; then
  echo "FAIL MEDKIT_CLIENTS: POST /auth/authorize issued no token" >&2
  docker logs medkit-smoke-closed >&2 2>&1 || true
  failures=$((failures + 1))
else
  authed_code=$(curl -s -o /dev/null -w '%{http_code}' \
    -H "Authorization: Bearer $token" "http://$closed_ip:8080/api/v1/areas")
  expect "MEDKIT_CLIENTS credential: GET /areas with its token" 200 "$authed_code" \
    || failures=$((failures + 1))
fi

# Arguments here are passed BEHIND the image's three config layers, which the
# entrypoint supplies, so server.host stays 0.0.0.0 without being repeated -
# and the case exercises that layering as well as the precedence it is about.
override_code=$(COPY_FILE="$WORKDIR/auth-off.yaml" COPY_DEST=/tmp/auth-off.yaml \
  posture_of medkit-smoke-override \
  -e MEDKIT_JWT_SECRET="$SECRET" -e MEDKIT_CLIENTS="$CLIENTS" \
  -- --ros-args --params-file /tmp/auth-off.yaml)
expect "environment over a params file with auth off: anonymous GET /areas" 401 \
  "$override_code" || failures=$((failures + 1))

# A mounted file at the documented mount point has to beat the image's own
# layers. refresh_interval_ms is the probe: the packaged config says 30000, the
# image's container layer says 2000, and a mount saying 4321 must win. Read back
# from the running gateway, because the file proves only what was written.
cat > "$WORKDIR/mounted.yaml" <<'YAML'
ros2_medkit_gateway:
  ros__parameters:
    refresh_interval_ms: 4321
YAML
chmod a+r "$WORKDIR/mounted.yaml"

mount_code=$(COPY_FILE="$WORKDIR/mounted.yaml" COPY_DEST=/etc/ros2_medkit/params.yaml \
  posture_of medkit-smoke-mount --)
expect "mounted params file: anonymous GET /areas" 200 "$mount_code" \
  || failures=$((failures + 1))

expect_in_startup "mounted params file: backstop refresh interval" \
  "backstop refresh interval: 4321ms" "$(startup_line medkit-smoke-mount)" \
  || failures=$((failures + 1))

# An arg-only override replaces the CMD. The three config layers live in the
# entrypoint precisely so that does not drop them: the port changes and the
# container still binds every interface, which is what makes it reachable.
argonly_code=$(ARG_PORT=9090 posture_of medkit-smoke-argonly -- --ros-args -p server.port:=9090)
expect "arg-only override: anonymous GET /areas on 9090" 200 "$argonly_code" \
  || failures=$((failures + 1))

expect_in_startup "arg-only override: bind address and port" \
  "REST API at 0.0.0.0:9090" "$(startup_line medkit-smoke-argonly)" \
  || failures=$((failures + 1))

if [ "$failures" -ne 0 ]; then
  echo "$failures image posture case(s) failed" >&2
  exit 1
fi
echo "image posture: every case as documented"
