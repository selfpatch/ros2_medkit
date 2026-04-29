#!/usr/bin/env bash
# Copyright 2026 bburda
#
# Regression gate for issue #375. Any direct call to rclcpp's subscription /
# callback-group creation APIs from gateway and fault_manager code outside
# `ros2_common/` is forbidden because it bypasses Ros2SubscriptionExecutor's
# serial worker (or fault_manager's LockedSubscriptionGuard) and reintroduces
# the rcl hash-map race that triggered SIGSEGV on Rolling.
#
# Callers in the gateway must go through:
#   ros2_medkit_gateway::ros2_common::Ros2SubscriptionSlot::create_typed / create_generic
#   ros2_medkit_gateway::ros2_common::Ros2SubscriptionExecutor::run_sync
# Callers in fault_manager must hold node_ops_mutex_ via LockedSubscriptionGuard.
#
# Known-legacy call sites that still use naked APIs are listed in
# ALLOWED_LEGACY_FILES below and are tracked for follow-up migration. Migrating
# a file means removing it from ALLOWED_LEGACY_FILES and routing through the
# appropriate guard.
#
# Multi-line aware: `auto sub = node->create_subscription\n    <T>(...);`
# slips past line-by-line `grep`, so the scan slurps each file with perl and
# matches across newlines.
#
# Exit code:
#   0  no violations outside ros2_common/, test/, or the allowlist
#   1  at least one forbidden call outside allowed locations
#   2  scan misconfiguration (missing dir, missing perl)

set -euo pipefail

if ! command -v perl >/dev/null 2>&1; then
  echo "FAIL: perl is required for multi-line scanning but was not found on PATH." >&2
  exit 2
fi

GATEWAY_ROOT="src/ros2_medkit_gateway"
FAULT_MANAGER_ROOT="src/ros2_medkit_fault_manager"

FORBIDDEN_PATTERN='(create_generic_subscription|create_subscription\s*<|create_callback_group)'

# Directories where the primitives may be called directly.
ALLOWED_DIRS_PATTERN="(${GATEWAY_ROOT}/src/ros2_common/|${GATEWAY_ROOT}/include/ros2_medkit_gateway/ros2_common/|${GATEWAY_ROOT}/test/|${FAULT_MANAGER_ROOT}/test/)"

# Explicit files that still use the naked APIs pending provider migration.
# Keep this list SHORT and add a TODO in the file referencing the follow-up.
ALLOWED_LEGACY_FILES=(
  "${GATEWAY_ROOT}/src/http/handlers/sse_fault_handler.cpp"        # faults provider follow-up
  "${GATEWAY_ROOT}/src/trigger_fault_subscriber.cpp"               # faults provider follow-up
  "${GATEWAY_ROOT}/src/trigger_topic_subscriber.cpp"               # data_stream provider follow-up
  "${GATEWAY_ROOT}/src/ros2/transports/ros2_action_transport.cpp"  # operations provider follow-up
  "${GATEWAY_ROOT}/src/log_manager.cpp"                            # logs provider follow-up
  "${FAULT_MANAGER_ROOT}/src/snapshot_capture.cpp"                 # uses LockedSubscriptionGuard (in-place serialisation)
  "${FAULT_MANAGER_ROOT}/include/ros2_medkit_fault_manager/snapshot_capture.hpp"  # comment references the guarded API
  "${FAULT_MANAGER_ROOT}/src/rosbag_capture.cpp"                   # bag-recorder spawns its own node + executor, no shared rcl hash map
)

LEGACY_PATTERN=""
for f in "${ALLOWED_LEGACY_FILES[@]}"; do
  if [[ -n "${LEGACY_PATTERN}" ]]; then
    LEGACY_PATTERN+="|"
  fi
  LEGACY_PATTERN+="${f}"
done
LEGACY_PATTERN="(${LEGACY_PATTERN})"

scan_dirs=(
  "${GATEWAY_ROOT}/src"
  "${GATEWAY_ROOT}/include"
  "${FAULT_MANAGER_ROOT}/src"
  "${FAULT_MANAGER_ROOT}/include"
)

# Fail loudly if a scan directory is missing. A silent miss converts the gate
# into a no-op (scan empty -> exit 0), which is exactly the fail-open we are
# trying to prevent.
for d in "${scan_dirs[@]}"; do
  if [[ ! -d "${d}" ]]; then
    echo "FAIL: scan directory '${d}' does not exist. The regression gate cannot run." >&2
    echo "This likely means a package was moved or renamed. Update scan_dirs in $(basename "$0")." >&2
    exit 2
  fi
done

# Multi-line scan via perl: for each *.cpp / *.hpp under scan_dirs, slurp the
# file (-0777), scan with the FORBIDDEN_PATTERN, print "path:LINE:matched"
# for every hit. Line numbers are computed from the byte offset of the match.
# Pipe `find -print0 | xargs -0` directly so NUL separators survive (bash
# variables strip NULs).
file_count=$(find "${scan_dirs[@]}" -type f \( -name '*.cpp' -o -name '*.hpp' \) -print | wc -l)
if [[ "${file_count}" -eq 0 ]]; then
  echo "FAIL: no source files found under scan_dirs - regression gate would silently pass." >&2
  exit 2
fi

# shellcheck disable=SC2016
# The perl script body intentionally contains $-prefixed perl variables that
# must NOT be expanded by bash. The single bash expansion ${FORBIDDEN_PATTERN}
# is interpolated explicitly via the '"..."' splice; everything else stays
# literal for perl.
raw_matches=$(find "${scan_dirs[@]}" -type f \( -name '*.cpp' -o -name '*.hpp' \) -print0 \
  | xargs -0 perl -0777 -ne '
      BEGIN { $pat = qr/'"${FORBIDDEN_PATTERN}"'/; }
      while ($_ =~ /$pat/g) {
        my $offset = pos($_) - length($&);
        my $line = 1 + (substr($_, 0, $offset) =~ tr/\n//);
        my $excerpt = $&;
        $excerpt =~ s/\s+/ /g;
        print "$ARGV:$line:$excerpt\n";
      }
    ')

violations="${raw_matches}"
if [[ -n "${violations}" ]]; then
  violations=$(printf '%s\n' "${violations}" | grep -vE "${ALLOWED_DIRS_PATTERN}" || true)
fi
if [[ -n "${violations}" ]]; then
  violations=$(printf '%s\n' "${violations}" | grep -vE "${LEGACY_PATTERN}" || true)
fi

if [[ -n "${violations}" ]]; then
  echo "FAIL: naked rclcpp subscription / callback-group creation outside ros2_common/:"
  echo "${violations}"
  echo
  echo "Route these through ros2_medkit_gateway::ros2_common::Ros2SubscriptionSlot"
  echo "so the rcl hash-map race fixed in issue #375 does not come back."
  echo "See src/ros2_medkit_gateway/design/ros2_subscription_architecture.rst for the allowed pattern."
  echo "To allowlist a legitimate new site, add the file path to"
  echo "ALLOWED_LEGACY_FILES in $(basename "$0") with a migration TODO."
  exit 1
fi

echo "OK: no new naked rclcpp subscription APIs outside ros2_common/ (gateway + fault_manager)."
