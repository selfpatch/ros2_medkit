#!/bin/bash
# Copyright 2026 selfpatch contributors
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

# Install cpp-httplib v0.14.3 into the Pixi prefix ($CONDA_PREFIX).
# cpp-httplib is not available on conda-forge, so we build from source.
# This is the same approach used in CI for ROS 2 Humble.
set -euo pipefail

VERSION="v0.14.3"
# Pinned commit SHA for supply-chain safety (matches tag v0.14.3)
COMMIT_SHA="cbca63f091ef1147ff57e90eb1ee5e558aa05d2c"
INSTALL_PREFIX="${CONDA_PREFIX:?CONDA_PREFIX not set - run inside pixi environment}"

# Skip if already installed
if [ -f "$INSTALL_PREFIX/include/httplib.h" ]; then
  echo "cpp-httplib already installed in $INSTALL_PREFIX"
  exit 0
fi

TMPDIR="$(mktemp -d)"
trap 'rm -rf "$TMPDIR"' EXIT

echo "Installing cpp-httplib $VERSION into $INSTALL_PREFIX..."
git clone --depth 1 --branch "$VERSION" https://github.com/yhirose/cpp-httplib.git "$TMPDIR/cpp-httplib"

# Verify the cloned commit matches the pinned SHA
ACTUAL_SHA="$(git -C "$TMPDIR/cpp-httplib" rev-parse HEAD)"
if [ "$ACTUAL_SHA" != "$COMMIT_SHA" ]; then
  echo "ERROR: cpp-httplib SHA mismatch! Expected $COMMIT_SHA, got $ACTUAL_SHA"
  echo "The tag v0.14.3 may have been retargeted. Aborting for safety."
  exit 1
fi

cmake -S "$TMPDIR/cpp-httplib" -B "$TMPDIR/build" \
  -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" \
  -DHTTPLIB_REQUIRE_OPENSSL=ON
cmake --build "$TMPDIR/build" --target install

echo "cpp-httplib $VERSION installed successfully"
