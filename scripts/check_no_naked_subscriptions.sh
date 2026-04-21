#!/usr/bin/env bash
# Copyright 2026 bburda
#
# Regression gate for issue #375. Any direct call to rclcpp's subscription /
# callback-group creation APIs from gateway code outside `ros2_common/` is
# forbidden because it bypasses Ros2SubscriptionExecutor's serial worker and
# reintroduces the rcl hash-map race that triggered SIGSEGV on Rolling.
#
# Callers in the gateway must go through:
#   ros2_medkit_gateway::ros2_common::Ros2SubscriptionSlot::create_typed / create_generic
#   ros2_medkit_gateway::ros2_common::Ros2SubscriptionExecutor::run_sync
#
# Known-legacy call sites that still use naked APIs are listed in
# ALLOWED_LEGACY_FILES below and are tracked for migration under Phase C
# (operations / faults / data_stream / logs providers). Migrating a file
# means removing it from ALLOWED_LEGACY_FILES and routing through
# Ros2SubscriptionSlot instead.
#
# Exit code:
#   0  no violations outside ros2_common/, test/, or the allowlist
#   1  at least one forbidden call outside allowed locations

set -euo pipefail

GATEWAY_ROOT="src/ros2_medkit_gateway"

FORBIDDEN_PATTERN='(create_generic_subscription|create_subscription<|create_callback_group)'

# Directories where the primitives may be called directly.
ALLOWED_DIRS_PATTERN="(${GATEWAY_ROOT}/src/ros2_common/|${GATEWAY_ROOT}/include/ros2_medkit_gateway/ros2_common/|${GATEWAY_ROOT}/test/)"

# Explicit files that still use the naked APIs pending provider migration.
# Keep this list SHORT and add a TODO in the file referencing the follow-up.
ALLOWED_LEGACY_FILES=(
  "${GATEWAY_ROOT}/src/http/handlers/sse_fault_handler.cpp"      # Phase C faults provider
  "${GATEWAY_ROOT}/src/trigger_fault_subscriber.cpp"             # Phase C faults provider
  "${GATEWAY_ROOT}/src/trigger_topic_subscriber.cpp"             # Phase C data_stream provider
  "${GATEWAY_ROOT}/src/operation_manager.cpp"                    # Phase C operations provider
  "${GATEWAY_ROOT}/src/log_manager.cpp"                          # Phase C logs provider
  "${GATEWAY_ROOT}/src/native_topic_sampler.cpp"                 # B9 cleanup (removal pending)
  "${GATEWAY_ROOT}/include/ros2_medkit_gateway/native_topic_sampler.hpp"  # B9 cleanup
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
)

violations=$(grep -rEn "${FORBIDDEN_PATTERN}" "${scan_dirs[@]}" \
    --include='*.cpp' --include='*.hpp' 2>/dev/null \
  | grep -vE "${ALLOWED_DIRS_PATTERN}" \
  | grep -vE "${LEGACY_PATTERN}" || true)

if [[ -n "${violations}" ]]; then
  echo "FAIL: naked rclcpp subscription / callback-group creation outside ros2_common/:"
  echo "${violations}"
  echo
  echo "Route these through ros2_medkit_gateway::ros2_common::Ros2SubscriptionSlot"
  echo "so the rcl hash-map race fixed in issue #375 does not come back."
  echo "See docs/design/ros2_subscription_architecture.rst once Phase D lands."
  exit 1
fi

echo "OK: no new naked rclcpp subscription APIs outside ros2_common/."
