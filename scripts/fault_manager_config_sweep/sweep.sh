#!/usr/bin/env bash
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

# Start the fault manager once per out-of-range or degenerate parameter value and capture its
# log lines, the status it reaches, the node exit status and the probe exit status.
# Usage: sweep.sh <workspace_root> <out_dir> [domain_id]
# Needs a sourced ROS environment. SWEEP_ONLY=<regex> limits the cases. Cases that depend on the
# rosbag recorder timing run SWEEP_RACE_RUNS times (default 5) into <case>.rN.txt.
# Run it against two builds and compare the out directories with compare.sh.
# Exit status: 1 when any probe failed or any node ended with a status other than 0 or 143.
set -o pipefail
: "${ROS_DISTRO:?source the ROS setup file first}"
ws="$(cd "$1" && pwd)"
out="$2"
export ROS_DOMAIN_ID="${3:-225}"
race_runs="${SWEEP_RACE_RUNS:-5}"
here="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "$ws/install/setup.bash"
set -u
failures=0
mkdir -p "$out"
node_bin="$ws/install/ros2_medkit_fault_manager/lib/ros2_medkit_fault_manager/fault_manager_node"
work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT

cat >"$work/entity_bad.yaml" <<'YAML'
/sweep_source:
  confirmation_threshold: 3
  healing_threshold: -4
YAML
cat >"$work/fault_bad.yaml" <<'YAML'
SWEEP_FAULT:
  confirmation_threshold: 2
  healing_threshold: -1
YAML
cat >"$work/corr_ok.yaml" <<'YAML'
correlation:
  enabled: true
  default_window_ms: 500
  patterns:
    sensor_errors:
      codes: ["SENSOR_*"]
  rules:
    - id: r1
      name: "R1"
      mode: hierarchical
      root_cause:
        codes: ["ESTOP_001"]
      symptoms:
        - pattern: sensor_errors
      window_ms: 2000
      mute_symptoms: true
YAML
cat >"$work/corr_bad.yaml" <<'YAML'
correlation:
  enabled: true
  rules:
    - id: r1
      mode: hierarchical
      root_cause:
        codes: []
      symptoms:
        - pattern: missing_pattern
YAML

# Drops timestamps, the temp path and run index, recording ids, bag sizes and durations, and attached-fault counts.
normalise() {
  sed -E -e 's/^\[([A-Z]+)\] \[[0-9.]+\] /[\1] /' -e "s#$work#WORK#g" -e 's#WORK/([a-z_]+)\.[0-9]+/#WORK/\1/#g' \
    -e 's/(fault_SWEEP_FAULT_)[0-9]+/\1MS/g' \
    -e 's/\([0-9.]+ MB(, [0-9.]+s, [0-9]+ attached fault\(s\))?\)/(SIZE)/' "$1"
}

# run_node <log> <probe_out> <failed> <passed> <settle> <node arg>...
# Sets node_status and probe_status. A node that is gone before the probe ends reports 'gone'.
run_node() {
  local log="$1" probe_out="$2" failed="$3" passed="$4" settle="$5"
  shift 5
  "$node_bin" --ros-args "$@" >"$log" 2>&1 &
  local pid=$!
  python3 "$here/probe.py" "$failed" "$passed" "$settle" >"$probe_out" 2>&1
  probe_status=$?
  if kill -0 "$pid" 2>/dev/null; then
    kill -TERM "$pid" 2>/dev/null
    wait "$pid" 2>/dev/null
    node_status=$?
  else
    wait "$pid" 2>/dev/null
    node_status="gone-$?"
  fi
  if [ "$probe_status" -ne 0 ] || { [ "$node_status" != 0 ] && [ "$node_status" != 143 ]; }; then
    failures=$((failures + 1))
    echo "UNEXPECTED: probe=$probe_status node=$node_status for $log" >&2
  fi
}

write_capture() {
  local log="$1" probe_out="$2" dest="$3"
  {
    normalise "$log"
    cat "$probe_out"
    echo "exit: node=$node_status probe=$probe_status"
  } >"$dest"
}

# run_case <name> <failed> <passed> <settle> <param>...   (db path is a fresh file per case)
run_case() {
  local name="$1" failed="$2" passed="$3" settle="$4"
  shift 4
  if [ -n "${SWEEP_ONLY:-}" ] && ! [[ "$name" =~ $SWEEP_ONLY ]]; then
    return
  fi
  local runs=1
  case "$name" in
    rosbag_all_bad | rosbag_neg_after | rosbag_cooldown_warn) runs="$race_runs" ;;
  esac
  local i dir dest
  for i in $(seq 1 "$runs"); do
    dir="$work/$name.$i"
    mkdir -p "$dir"
    run_node "$dir/node.log" "$dir/probe.txt" "$failed" "$passed" "$settle" \
      -p storage_type:=sqlite -p "database_path:=$dir/faults.db" -p snapshots.enabled:=false \
      -p snapshots.recapture_cooldown_sec:=0.0 "$@"
    dest="$out/$name.txt"
    [ "$runs" -gt 1 ] && dest="$out/$name.r$i.txt"
    write_capture "$dir/node.log" "$dir/probe.txt" "$dest"
  done
}

run_case default 2 1 0.2
run_case conf_positive 4 0 0.2 -p confirmation_threshold:=5
run_case conf_zero 3 0 0.2 -p confirmation_threshold:=0
run_case conf_wrap_neg 2 0 0.2 -p confirmation_threshold:=-4294967297
run_case heal_wrap_pos 2 3 0.2 -p healing_enabled:=true -p healing_threshold:=4294967296 -p confirmation_threshold:=-1
run_case heal_negative 3 4 0.2 -p healing_enabled:=true -p healing_threshold:=-2 -p confirmation_threshold:=-1
run_case heal_enabled_4294967296 2 3 0.2 -p healing_enabled:=true -p healing_threshold:=4294967296
run_case autoconfirm_nan 1 0 3.5 -p confirmation_threshold:=-3 -p auto_confirm_after_sec:=.nan
run_case autoconfirm_neg 1 0 3.5 -p confirmation_threshold:=-3 -p auto_confirm_after_sec:=-1.0
run_case autoconfirm_huge 1 0 3.5 -p confirmation_threshold:=-3 -p auto_confirm_after_sec:=1.0e10
run_case autoconfirm_valid_entity 1 0 3.5 -p confirmation_threshold:=-3 -p auto_confirm_after_sec:=1.5 \
  -p "entity_thresholds.config_file:=$work/entity_bad.yaml"
run_case cooldown_nan 2 0 0.2 -p snapshots.recapture_cooldown_sec:=.nan
run_case cooldown_neg 2 0 0.2 -p snapshots.recapture_cooldown_sec:=-5.0
run_case pool_zero 2 0 0.2 -p snapshots.capture_pool_size:=0
run_case pool_wrap 2 0 0.2 -p snapshots.capture_pool_size:=4294967296
run_case pool_max 2 0 0.2 -p snapshots.capture_pool_size:=64
run_case pool_over 2 0 0.2 -p snapshots.capture_pool_size:=65
run_case depth_zero 2 0 0.2 -p snapshots.capture_queue_depth:=0
run_case depth_wrap 2 0 0.2 -p snapshots.capture_queue_depth:=4294967296
run_case depth_negative 2 0 0.2 -p snapshots.capture_queue_depth:=-7
run_case depth_max 2 0 0.2 -p snapshots.capture_queue_depth:=65536
run_case depth_over 2 0 0.2 -p snapshots.capture_queue_depth:=65537
run_case policy_bogus 2 0 0.2 -p snapshots.capture_queue_full_policy:=bogus
run_case policy_drop_oldest 2 0 0.2 -p snapshots.capture_queue_full_policy:=drop_oldest
run_case policy_empty 2 0 0.2 -p "snapshots.capture_queue_full_policy:=''"
run_case maxsnap_neg 2 0 0.2 -p snapshots.max_per_fault:=-1
run_case maxsnap_zero 2 0 0.2 -p snapshots.max_per_fault:=0
run_case nearmiss_neg 2 0 0.2 -p near_miss.max_per_fault:=-1
run_case nearmiss_zero 2 0 0.2 -p near_miss.max_per_fault:=0
run_case audit_bad 2 0 0.2 -p audit_log.enabled:=true -p audit_log.transitions:=bogus \
  -p audit_log.retention_max_records:=-1
run_case audit_confirmed_only 2 0 0.2 -p audit_log.enabled:=true -p audit_log.transitions:=confirmed_only \
  -p audit_log.fail_closed:=true -p audit_log.retention_max_records:=5
run_case audit_off_bad_values 2 0 0.2 -p audit_log.transitions:=bogus -p audit_log.retention_max_records:=-3
run_case snap_timeout_nan 2 0 0.2 -p snapshots.timeout_sec:=.nan
run_case snap_timeout_zero 2 0 0.2 -p snapshots.timeout_sec:=0.0
run_case snap_msgsize_zero 2 0 0.2 -p snapshots.enabled:=true -p snapshots.max_message_size:=0
run_case snap_msgsize_neg 2 0 0.2 -p snapshots.enabled:=true -p snapshots.max_message_size:=-9
run_case rosbag_all_bad 2 0 0.2 -p snapshots.enabled:=true -p snapshots.rosbag.enabled:=true \
  -p snapshots.rosbag.duration_sec:=.nan -p snapshots.rosbag.duration_after_sec:=.nan \
  -p snapshots.rosbag.max_bag_size_mb:=0 -p snapshots.rosbag.max_total_storage_mb:=-1 \
  -p snapshots.rosbag.max_buffer_mb:=0 -p snapshots.rosbag.max_bags_per_fault:=-1 \
  -p "snapshots.rosbag.storage_path:=$work/bags"
run_case rosbag_neg_after 2 0 0.2 -p snapshots.enabled:=true -p snapshots.rosbag.enabled:=true \
  -p snapshots.rosbag.duration_sec:=-2.0 -p snapshots.rosbag.duration_after_sec:=-1.0 \
  -p "snapshots.rosbag.storage_path:=$work/bags"
run_case rosbag_cooldown_warn 2 0 0.2 -p snapshots.enabled:=true -p snapshots.rosbag.enabled:=true \
  -p snapshots.rosbag.max_bags_per_fault:=3 -p snapshots.recapture_cooldown_sec:=60.0 \
  -p "snapshots.rosbag.storage_path:=$work/bags"
run_case rosbag_off_bad_values 2 0 0.2 -p snapshots.rosbag.enabled:=false
run_case storage_memory 2 0 0.2 -p storage_type:=memory
run_case storage_bogus 2 0 0.2 -p storage_type:=bogus
run_case storage_postgres_unreachable 1 0 0.2 -p storage_type:=postgres \
  -p "database_url:=postgresql://nobody:nopass@127.0.0.1:1/none?connect_timeout=1"
run_case thresholds_bad_files 3 0 0.2 -p "entity_thresholds.config_file:=$work/entity_bad.yaml" \
  -p "fault_thresholds.config_file:=$work/fault_bad.yaml"
run_case thresholds_missing_files 2 0 0.2 -p "entity_thresholds.config_file:=$work/nope1.yaml" \
  -p "fault_thresholds.config_file:=$work/nope2.yaml"
run_case correlation_ok 2 0 0.2 -p "correlation.config_file:=$work/corr_ok.yaml"
run_case correlation_bad 2 0 0.2 -p "correlation.config_file:=$work/corr_bad.yaml"
run_case correlation_missing 2 0 0.2 -p "correlation.config_file:=$work/nope3.yaml"
run_case cleanup_interval_nan 2 0 0.2 -p "correlation.config_file:=$work/corr_ok.yaml" \
  -p correlation.cleanup_interval_sec:=.nan

# Two-phase: a HEALED row written by a healing run, reclassified by a run with healing off.
if [ -z "${SWEEP_ONLY:-}" ] || [[ "reclassify_healing_off" =~ $SWEEP_ONLY ]]; then
  phase_dir="$work/reclassify"
  mkdir -p "$phase_dir"
  run_node "$phase_dir/phase1.log" "$phase_dir/probe1.txt" 1 2 0.2 -p storage_type:=sqlite \
    -p "database_path:=$phase_dir/faults.db" -p snapshots.enabled:=false -p healing_enabled:=true \
    -p healing_threshold:=1
  run_node "$phase_dir/phase2.log" "$phase_dir/probe2.txt" 0 0 0.2 -p storage_type:=sqlite \
    -p "database_path:=$phase_dir/faults.db" -p snapshots.enabled:=false -p audit_log.enabled:=true \
    -p healing_enabled:=false
  write_capture "$phase_dir/phase2.log" "$phase_dir/probe2.txt" "$out/reclassify_healing_off.txt"
fi

echo "sweep done: $(find "$out" -name "*.txt" | wc -l) captures in $out, $failures unexpected"
[ "$failures" -eq 0 ]
