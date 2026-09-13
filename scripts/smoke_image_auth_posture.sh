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
# Check what an image actually serves, in the three configurations its
# documentation names.
#
# Usage: scripts/smoke_image_auth_posture.sh <image reference>
#
# The entrypoint's auth rule lives in shell, so no colcon test reaches it, and
# both halves of it have been wrong in a way only a running container showed:
# once the variable was ignored entirely, once it turned authentication on and
# left require_auth_for at "write" so every read stayed open.
#
#   1. no environment          -> anonymous GET /areas answers 200
#   2. MEDKIT_JWT_SECRET set   -> anonymous GET /areas answers 401
#   3. a mounted params file with auth off, plus MEDKIT_JWT_SECRET
#                              -> anonymous GET /areas answers 401
#
# Case 3 is the one that says the environment beats the file. Without it, an
# entrypoint that consulted the file and deferred to it would pass cases 1
# and 2 while leaving a container its operator believes closed wide open.
#
# The gateway is reached at the container's bridge address, never at a
# published port on localhost: where the Docker daemon is not in this shell's
# network namespace, -p publishes somewhere this script cannot see and every
# request times out while the container is perfectly healthy.

set -euo pipefail

IMAGE="${1:?usage: $0 <image reference>}"
SECRET="smoke_image_posture_secret_of_at_least_32_chars"
CLIENTS="smoke:smoke_client_secret:admin"
WORKDIR="$(mktemp -d)"

# Fixed, not collected as the cases run: each case calls posture_of inside a
# command substitution to capture the status code, and a subshell cannot append
# to the parent's array - so a list built that way is empty when the trap fires
# and every container survives the run.
CONTAINERS=(medkit-smoke-open medkit-smoke-closed medkit-smoke-override)

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
    return 1
  fi

  # curl prints 000 itself when it cannot connect and exits non-zero; a
  # fallback `|| echo 000` here appended a SECOND 000, and "000000" is not
  # "000", so the wait loop finished on the first attempt with a value no
  # comparison below could match.
  local code="000"
  for _ in $(seq 1 60); do
    code=$(curl -s -o /dev/null -w '%{http_code}' "http://$ip:8080/api/v1/areas") || true
    [ -n "$code" ] && [ "$code" != "000" ] && break
    sleep 2
  done
  if [ "$code" = "000" ]; then
    echo "$name: never answered on http://$ip:8080" >&2
    docker logs "$name" >&2 2>&1 || true
    return 1
  fi
  echo "$code"
}

expect() {
  local label=$1 want=$2 got=$3
  if [ "$got" = "$want" ]; then
    echo "ok   $label: anonymous GET /areas -> $got"
  else
    echo "FAIL $label: anonymous GET /areas -> $got, expected $want" >&2
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
expect "default, no environment" 200 "$open_code" || failures=$((failures + 1))

closed_code=$(posture_of medkit-smoke-closed \
  -e MEDKIT_JWT_SECRET="$SECRET" -e MEDKIT_CLIENTS="$CLIENTS" --)
expect "MEDKIT_JWT_SECRET set" 401 "$closed_code" || failures=$((failures + 1))

# The container args replace the CMD, so server.host has to be repeated here:
# the packaged profile binds loopback and the image's own CMD is what opens it.
override_code=$(COPY_FILE="$WORKDIR/auth-off.yaml" COPY_DEST=/tmp/auth-off.yaml \
  posture_of medkit-smoke-override \
  -e MEDKIT_JWT_SECRET="$SECRET" -e MEDKIT_CLIENTS="$CLIENTS" \
  -- --ros-args --params-file /tmp/auth-off.yaml -p server.host:=0.0.0.0)
expect "environment over a params file with auth off" 401 "$override_code" \
  || failures=$((failures + 1))

if [ "$failures" -ne 0 ]; then
  echo "$failures image posture case(s) failed" >&2
  exit 1
fi
echo "image posture: all three cases as documented"
