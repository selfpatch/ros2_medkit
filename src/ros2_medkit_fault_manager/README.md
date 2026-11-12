# ros2_medkit_fault_manager

Central fault manager node for the ros2_medkit fault management system.

## Overview

The FaultManager node provides a central point for fault aggregation and lifecycle management.
It receives fault reports from multiple sources, keeps them as fault records, and provides query
and clearing interfaces.

A record is identified by its `fault_code` and an **owner**. By default a code has **one record
shared by all its reporting sources**, owned by the reserved id `*`: a FAILED from any source moves
its one debounce counter, a PASSED from any source moves it back, `reporting_sources` lists every
source that reported it, and a clear ends the whole record. A code listed with `per_source: true` in
`fault_thresholds.config_file` instead has **one record per reporting source**, owned by the
`source_id` of its reports. Each record has its own status, debounce counter, occurrence count,
severity, timestamps, freeze frame, snapshots, near-miss series and rosbag links, and is cleared and
healed on its own. `ReportFault` refuses `*` as a `source_id`.

The services that act on a single record (`~/clear_fault`, `~/get_fault`, `~/get_snapshots`,
`~/get_rosbag`) carry a `source_id` request field. It names the record it owns (`*` for a shared
one). When no record has that owner, it names the record that source reports into, so a client that
knows only a reporting source still reaches the shared record of a default code. Leaving it empty is
unscoped and applies only when exactly one record carries the `fault_code`, otherwise the call fails
with a message beginning `ambiguous:` that lists the owners and changes nothing.

### Identity per fault code

```yaml
# fault_thresholds.config_file
BATTERY_LOW:
  per_source: true            # one record per reporting source
  confirmation_threshold: -3

ESTOP:
  confirmation_threshold: -1  # per_source absent: one record shared by all sources
```

`per_source` is a YAML boolean and defaults to `false`; a code with no entry is shared too. Any other
value (`yes`, `3`) rejects the whole file with an error, like every other malformed value in it.
Entity overrides do not carry the key. The file is read at startup.

Switching a code between the two modes keeps the records already stored as they were written, and
sends new reports to the configured identity. For each code that holds active records of the other
identity the node logs one warning at startup, naming the code, its configured mode and the owners
found. Nothing stored is rewritten.

A code reported by several sources and debounced under several policies (per-entity thresholds) has
one counter, so the node warns once per code when two sources resolve different policies for the
shared record. Set `per_source: true` or pin the policy in `fault_thresholds.config_file` to end it.

### Upgrading a database written before the owner column

A SQLite store written by an earlier release is keyed by the bare `fault_code` and is rebuilt on the
first open. That store holds one record per code, fed by every source, which is exactly what a
default code is now, so every row becomes the shared record of its code (owner `*`) and keeps its
`reporting_sources` as written. Its freeze frame, snapshots, recording links and near misses attach
to that record, the near misses of every source included. `confirmed_at_ns` is copied as it was,
except on a PREFAILED or PREPASSED row, where it becomes 0 (see Storage Backends below). All of it
runs in one transaction.

The rebuild is one way. A fault manager from an earlier release opened on a migrated store aborts on
the first report of a fault code the store does not hold yet (`NOT NULL constraint failed:
faults.owner`). If a rollback may be needed, keep a copy of `faults.db` from before the upgrade and
restore that copy instead of pointing the earlier release at the migrated file.

A PostgreSQL database written by an earlier release is migrated in place on the first connection, by
the same rules: the primary keys on the fault code become unique indexes over `(fault_code, owner)`
and every row becomes the shared record of its code. It runs in one transaction, so a failure leaves
the tables as they were.

The table names `faults_new` and `freeze_frames_new` are reserved for the rebuild. Anything found
under those names is debris from a rebuild that did not finish and is dropped on open, with a
warning naming the table.

## Quick Start

By default, faults are confirmed immediately when reported - no additional configuration needed.

```bash
# Start the fault manager
ros2 launch ros2_medkit_fault_manager fault_manager.launch.py

# Report a fault - it's immediately CONFIRMED
ros2 service call /fault_manager/report_fault ros2_medkit_msgs/srv/ReportFault \
  "{fault_code: 'MOTOR_OVERHEAT', event_type: 0, severity: 2, description: 'Motor temp exceeded', source_id: '/motor_node'}"

# Query faults
ros2 service call /fault_manager/list_faults ros2_medkit_msgs/srv/ListFaults \
  "{statuses: ['CONFIRMED']}"

# Clear a fault (cascade-clears correlated symptoms by default)
ros2 service call /fault_manager/clear_fault ros2_medkit_msgs/srv/ClearFault \
  "{fault_code: 'MOTOR_OVERHEAT', skip_correlation_auto_clear: false}"

# Clear without touching correlated symptoms
ros2 service call /fault_manager/clear_fault ros2_medkit_msgs/srv/ClearFault \
  "{fault_code: 'MOTOR_OVERHEAT', skip_correlation_auto_clear: true}"
```

> **Note:** The `skip_correlation_auto_clear` request field was added post-0.4.0. Adding a request field changes the service type hash, so callers built against `ros2_medkit_msgs` 0.4.0 or earlier must rebuild to keep talking to `fault_manager`.

## Services

| Service           | Type                                | Description                     |
| ----------------- | ----------------------------------- | ------------------------------- |
| `~/report_fault`  | `ros2_medkit_msgs/srv/ReportFault`  | Report a fault occurrence       |
| `~/list_faults`   | `ros2_medkit_msgs/srv/ListFaults`   | Query faults with filtering     |
| `~/clear_fault`   | `ros2_medkit_msgs/srv/ClearFault`   | Clear/acknowledge a fault       |
| `~/get_snapshots` | `ros2_medkit_msgs/srv/GetSnapshots` | Get topic snapshots for a fault |

## Features

- **Multi-source aggregation**: Same `fault_code` from different sources creates a single fault,
  unless the code is configured `per_source`, which gives each source its own record
- **Occurrence tracking**: Counts outages, not reports - the count starts at one and rises only
  when a cleared fault is raised again - and tracks all reporting sources
- **Severity escalation**: A record has one severity, the highest ever reported for it, kept
  within an occurrence and across occurrences. A report with a different severity is logged (see
  [Severity](#severity))
- **Persistent storage**: SQLite (default) or PostgreSQL backend ensures faults survive node restarts
- **Debounce filtering** (optional): AUTOSAR DEM-style counter-based fault confirmation with per-entity and per-fault_code threshold overrides
- **Snapshot capture**: Captures topic data when faults are confirmed for debugging (the value snapshots are deleted when the fault is cleared, unless `snapshots.retain_on_clear` is set)
- **Near-miss series**: Appends one entry per FAILED report that moved the debounce counter without confirming, bounded per record and retained when the record is cleared
- **Freeze-frame retention**: One compact JSON freeze-frame per record, retained across `clear_fault` (see below)
- **Fault correlation** (optional): Root cause analysis with symptom muting and auto-clear
- **Tamper-evident audit log** (optional): Append-only, hash-chained record of fault state transitions for verifiable history

## Parameters

| Parameter                       | Type   | Default                            | Description                                                                     |
| ------------------------------- | ------ | ---------------------------------- | ------------------------------------------------------------------------------- |
| `storage_type`                  | string | `"sqlite"`                         | Storage backend: `"sqlite"`, `"memory"` or `"postgres"`                         |
| `database_path`                 | string | `"/var/lib/ros2_medkit/faults.db"` | Path to SQLite database file                                                    |
| `database_url`                  | string | `""`                               | PostgreSQL connection string; empty = libpq environment variables               |
| `confirmation_threshold`        | int    | `-1`                               | Counter value at which faults are confirmed                                     |
| `healing_enabled`               | bool   | `false`                            | Enable automatic healing via PASSED events                                      |
| `healing_threshold`             | int    | `3`                                | Counter value at which faults are healed                                        |
| `auto_confirm_after_sec`        | double | `0.0`                              | Auto-confirm PREFAILED faults after timeout (0 = disabled)                      |
| `entity_thresholds.config_file` | string | `""`                               | Path to YAML file with per-entity debounce threshold overrides                  |
| `fault_thresholds.config_file`  | string | `""`                               | Path to YAML file with per-fault_code debounce threshold overrides, applied on top of the entity ones |
| `near_miss.max_per_fault`       | int    | `200`                              | Near-miss entries retained per fault record, oldest evicted first (0 = unlimited) |

### Snapshot Parameters

Snapshots capture topic data when faults are confirmed for post-mortem debugging.

Each confirm also writes a **freeze-frame**: a single compact JSON object mapping every captured topic to its value at confirmation time, keyed by the record. Which topics are captured is decided by the fault CODE (`fault_specific` and `patterns` are configuration about what a code means), while what is written belongs to the record, so two `per_source` records of one code capture the same topics into two separate frames. It differs from per-topic snapshots in two ways: snapshots are deleted when the record is cleared, while the freeze-frame is retained across `clear_fault` (once the snapshots are gone, `~/get_fault` serves the retained frame so the confirmed-state record stays available after acknowledgement), and a re-confirm that captures nothing (e.g. source publishers down) never overwrites an existing non-empty frame. A fault code with no configured capture set gets no freeze-frame row. A configured capture that samples nothing on its first run records an empty `{}` frame. Freeze-frame storage is bounded by the number of distinct records (one row per record, replaced in place) and rows are never evicted.

Under a fault storm, captures are bounded by a worker pool (`capture_pool_size`) draining a bounded queue (`capture_queue_depth`); excess captures are dropped per `capture_queue_full_policy` and logged (throttled). The pool is shared and is created when snapshots **or** rosbag is enabled, so these parameters bound both. `capture_pool_size` parallelizes freeze-frame snapshot capture only - rosbag stays single-writer regardless of pool size, and correlated faults confirming inside one post-roll window share a single recording.

That single-writer property also shapes what each fault of a burst gets. Nothing is buffered while a post-fault window is open (messages go straight into the open bag), and the flush that opened that bag already emptied the ring buffer, so a fault confirming right _after_ the window closes has no pre-fault history available. What a confirmation gets is decided by the buffer, so a fault arriving before any captured topic has published lands the same way. It gets a **post-fault-only bag**: its own recording holding just its `duration_after_sec` window, entered through the same post-roll state machine, so later faults of the burst attach to it normally. With `duration_after_sec: 0` there is no window to record into and such a fault gets no bag; if the bag cannot be written at all, no recording is opened and no metadata row is stored. The `duration_sec` on a stored bag is the span the recording was open rather than the configured windows, so a post-fault-only bag usually reports roughly `duration_after_sec` where a full one reports its buffered history too. It is a recording span, not a content span: a window during which nothing was published still reports the seconds it covered. It can also exceed `duration_sec + duration_after_sec`, because the ring buffer is pruned only when a message arrives - a topic that stops publishing keeps its last window buffered until the next confirmation flushes it, which is deliberate for a black box. See `docs/config/fault-manager.rst` for the full lifecycle.

| Parameter                             | Type     | Default         | Description                                                                                                                |
| ------------------------------------- | -------- | --------------- | -------------------------------------------------------------------------------------------------------------------------- |
| `snapshots.enabled`                   | bool     | `true`          | Enable/disable snapshot capture                                                                                            |
| `snapshots.background_capture`        | bool     | `false`         | Cache each configured topic's latest message vs on-demand capture; a topic absent at startup is cached once it appears     |
| `snapshots.timeout_sec`               | double   | `1.0`           | Timeout waiting for topic message (on-demand mode)                                                                         |
| `snapshots.max_message_size`          | int      | `65536`         | Maximum message size in bytes (larger messages skipped)                                                                    |
| `snapshots.default_topics`            | string[] | `[]`            | Topics to capture for all faults                                                                                           |
| `snapshots.config_file`               | string   | `""`            | Path to YAML config for `fault_specific` and `patterns`                                                                    |
| `snapshots.recapture_cooldown_sec`    | double   | `60.0`          | Min seconds between captures for the same fault record.                                                                   |
| `snapshots.max_per_fault`             | int      | `10`            | Max snapshots retained per fault.                                                                                          |
| `snapshots.capture_pool_size`         | int      | `2`             | Max concurrent capture threads under a fault storm (>= 1). Parallelizes snapshot capture only; rosbag stays single-writer. |
| `snapshots.capture_queue_depth`       | int      | `16`            | Max pending captures before the full-queue policy applies (>= 1).                                                          |
| `snapshots.capture_queue_full_policy` | string   | `reject_newest` | Policy when the queue is full: `reject_newest` or `drop_oldest`.                                                           |

**Topic Resolution Priority:**

1. `fault_specific` - Exact match for fault code (configured via YAML config file)
2. `patterns` - Regex pattern match (configured via YAML config file)
3. `default_topics` - Fallback for all faults

**Example YAML config file** (`snapshots.yaml`):

```yaml
fault_specific:
  MOTOR_OVERHEAT:
    - /joint_states
    - /motor/temperature
patterns:
  "MOTOR_.*":
    - /joint_states
    - /cmd_vel
```

### Rosbag Capture

Black-box rosbag capture (`snapshots.rosbag.*`) writes bags in `mcap` format by
default, which Foxglove and Lichtblick open directly with no conversion step.
`sqlite3` remains available through `snapshots.rosbag.format` and is also what an
unknown format string lands on. Neither backend is privileged over the other:
whichever is configured, if its plugin is unavailable at startup the FaultManager
logs a warning naming the missing package and falls back to the other one
automatically, disabling capture only if neither loads. Both `rosbag2_storage_mcap`
and `rosbag2_storage_default_plugins` (which carries the sqlite3 plugin) are runtime
dependencies of this package, so a normal install pulls in both backends. See
`docs/config/fault-manager.rst` and `docs/tutorials/snapshots.rst` for the full
parameter list and recording lifecycle.

### Storage Backends

This is the persisted fault store (`storage_type`), separate from the rosbag storage
format used by black-box capture (`snapshots.rosbag.format`, see Rosbag Capture above).

**SQLite (default)**: Faults are persisted to disk and survive node restarts. Uses WAL mode for optimal performance.
Each row also records when the current occurrence of its record confirmed (`confirmed_at_ns`, read from the
database file, not by any service). It is 0 until the occurrence confirms: a FAILED report on a CLEARED record
resets it, and the confirmation of that new occurrence sets it again. The owner migration keeps the value of a
CONFIRMED, HEALED or CLEARED row and writes 0 for a PREFAILED or PREPASSED one. A row that confirmed stays
CONFIRMED or HEALED until it is cleared, so a pending row with a value carries it from the occurrence before a
reactivation.

**Memory**: Faults are stored in memory only. Useful for testing or when persistence is not required.

**PostgreSQL**: Faults are stored in an external PostgreSQL server (14 or newer) and survive node restarts. The audit log stays a local SQLite file next to `database_path`. One database belongs to exactly one fault manager; give each fault manager its own database or schema.

PostgreSQL support is off by default. To build it, install `libpq-dev` and pass `-DPOSTGRES_SUPPORT=ON`:

```bash
colcon build --packages-select ros2_medkit_fault_manager --cmake-args -DPOSTGRES_SUPPORT=ON
```

The build downloads libpqxx 7.10.7 from GitHub at configure time and links it statically; rosdep installs nothing for it. Keep the password out of `database_url`: leave it empty and set `PGHOST`, `PGPORT`, `PGDATABASE`, `PGUSER` and `PGPASSWORD` (or use `~/.pgpass`). The node never logs the connection string, and it removes the password from error text it logs.

A wrong configuration stops the node at startup with exit code 1: a connection string libpq cannot parse, or a server that accepts the connection but does not allow creating the schema, or a table that lacks a column the node uses. A server that cannot be reached (including a wrong password or a missing database) does not stop it: the node runs without storage, services with an error field answer `Fault storage unavailable`, and `ReportFault` answers `accepted=false`. After a failed connection attempt, requests fail at once for 5 s and the next request tries again. An attempt waits at most 2 s per host address unless `connect_timeout` or `PGCONNECT_TIMEOUT` says otherwise, and a request on an open connection to a server that went off the network fails after `tcp_user_timeout` (5 s). With a libpq service, these values come from the service file. See `docs/config/fault-manager.rst` for the full table.

## Severity

A record has one severity. The stored severity is the highest any FAILED report for it has
carried, within an occurrence and across occurrences: `~/clear_fault` ends an occurrence but does
not lower the severity, and the SQLite store keeps it across restarts. Two reports of one code from
one source with different severities mean a misconfigured reporter, and keeping the maximum is the
conservative reading. Another record of the same code (a `per_source` code has one per source) keeps
its own severity: a CRITICAL report for one record neither escalates nor confirms another.

What each surface carries:

| Surface | Severity |
|---|---|
| `~/list_faults`, `~/get_fault`, the gateway's `GET /faults` and fault detail | stored |
| `FaultEvent` on `~/events`, the gateway's `/faults/stream` frames and fault trigger payloads | stored, the same value a read of the record returns after the report |
| Audit rows (`occurred`, `confirmed`, `healed`, `cleared`) | stored, at the moment of the transition |
| Near-miss entries | the severity that report carried |

The near-miss series is per-report history, so it keeps what the reporter sent. A lower report
stays visible there and in the mismatch warning below.

`critical_immediate_confirm` reads the stored severity of the record. A FAILED report on a record
stored as `CRITICAL` confirms it at once, whatever severity the report carries, from a cleared
record and from the HEALED latch alike. So no report leaves a record `CRITICAL` in `PREFAILED` or
`PREPASSED`. A row an earlier release stored that way keeps its status through the migration until
its next FAILED report.

When a FAILED report carries a severity other than the one stored for its record, the node logs a
warning naming the code, the reported and the stored severity and the source. It is logged once per
code, source and reported severity until the node restarts, so a reporter that repeats the same
report many times a second produces one line. The number of lines is bounded by the distinct
combinations, at most four per record, and not by the report rate. Each misconfigured source is
named on its own line.

## Near-Miss Series

A **near miss** is a FAILED report that moved the debounce counter without the fault ending up
CONFIRMED - the fault nearly happened. This is the debounce sense of the term and is unrelated
to any scoring sense used elsewhere. PASSED reports move the counter too, but in the healing
direction (the fault receding), so they are not near misses.

The series is **append-only**: one entry per qualifying report, holding the timestamp, the counter
value after the report, the confirmation threshold it was measured against, the severity the
report carried, the reporting source, and the fault status the report left behind. Every field describes that one report, so a series spanning a threshold change
or a new reporting source stays readable. Nothing is updated in place, so "this code approached
confirmation five times this hour" stays answerable.

Read `resulting_status` before counting entries. The HEALED latch holds the status the whole way
from the healing threshold down to the confirmation threshold, so every report on the way back
into a fault that **does** confirm also moves the counter without confirming, and lands in the
series. Entries with `PREFAILED` are approaches from a resting state; entries with `HEALED` are a
counter walking back down under the latch. Without the field the two cannot be told apart, and
"how often did this code approach confirmation without becoming a fault" is not answerable.
`resulting_status` is never `CONFIRMED` - that is what makes a report a near miss. It is empty for
rows written before the field existed.

With per-entity thresholds the recorded `confirmation_threshold` is the one belonging to the
**reporting source**, while the debounce counter is shared by every source of a shared record. It
is therefore not by itself the distance to confirmation for the record as a whole. An entry in
`fault_thresholds.config_file` that sets `confirmation_threshold` makes the two agree again: it
resolves the same for every source, so the recorded threshold is the record's own. An entry with
only healing fields keeps the per-source threshold. A `per_source` record has one source, so its
recorded threshold is already the record's own. The node warns once per code when two sources resolve different
policies for the shared record.

Entries are kept and evicted in **arrival order**, not by their timestamps. Reporters carry their
own clocks, so a report can arrive carrying a timestamp behind one already stored; ordering the
series by timestamp would let such a report evict itself on arrival.

It is **retained across `~/clear_fault`**. Clearing acknowledges one fault cycle; how often a code
approaches confirmation spans cycles, and once deleted it cannot be reconstructed. The same holds
for the startup reclassification of HEALED faults. Per-topic snapshots are dropped on clear by
default, because they belong to the one confirmed occurrence rather than to the series; set
`snapshots.retain_on_clear` to keep them as well.

With `snapshots.retain_on_clear` on, `~/get_fault` keeps serving the freeze-frame alongside the
retained snapshots, because it records the most recent confirmation while the snapshots may belong
to earlier ones. `~/get_snapshots` returns one entry per topic and serves the newest capture of
that topic, whichever storage backend is in use.

The bound is **per record, not per database**. Fault codes are unbounded in cardinality (and so is
the set of reporting sources of a `per_source` code), so a reporter emitting a stream of distinct
codes still grows the table. The bound caps what any single record costs, not the total.

Retention is **bounded per record** by `near_miss.max_per_fault` (default 200), evicting the
**oldest** entries first. That is the same direction as `snapshots.max_per_fault` and the rosbag
cap, and for the same reason: a series frozen at boot says nothing about whether the rate is
changing, and the evidence a technician wants is the evidence from the fault happening now. Set it
to 0 for unlimited, accepting that the database then grows with the reporting rate.

The series lives in the `near_misses` table of the fault database and is read through the storage
API (`FaultStorage::get_near_misses`). A database written by an earlier build gains the table on
first open. **There is no service or REST surface for it yet** - reading it means going through the
storage API or the database file.

## Advanced: Tamper-Evident Audit Log

An optional append-only, hash-chained audit log records every fault state transition (`occurred`, `confirmed`, `healed`, `cleared`) so the fault history is independently verifiable. Auto-recovery (a record reaching the healing threshold via PASSED events) is recorded as a distinct `healed` row, so the record's END is in the timeline and is not confused with a manual `cleared`. The `source` of a row a report drove (`occurred`, `confirmed`) is the reporting source. The automatic and clear transitions name the owner of the record that moved, which for a code shared by all its sources is `*`. The manager has no acknowledge action separate from clearing, so `~/clear_fault` is recorded as `cleared` (clear == ack), and there is no `ack` kind. The log also records its own lifecycle with `logging_activated` / `logging_deactivated` markers at start and stop. Every row records the record's stored severity at the transition (see [Severity](#severity)). It is **off by default** because it adds a write and storage cost per transition.

Each transition appends one immutable row holding `record_hash = sha256(prev_hash + canonical(event))` (OpenSSL EVP SHA-256), the `prev_hash` it links to, and a monotonic `seq`. The hash is computed once at insert and never recomputed. A persisted chain head lets the chain resume across restarts. The log is stored in its own SQLite database (separate from the fault store) and is treated as append-only: the manager only ever inserts rows, and `BEFORE UPDATE` / `BEFORE DELETE` triggers reject out-of-band edits (the guarded rotation prune excepted).

**Completeness is an integrity property.** `verify()` proves nothing was _deleted_ from the chain, but it cannot prove a transition that was _never appended_. So a silently dropped append is a hole `verify()` can never see. Every transition on the write path is therefore audited (occurred, timer/threshold confirmations, auto-heal, and clears), and an append failure is never swallowed silently: it increments a dropped-writes counter and clears an "audit healthy" flag. **These are in-process signals only** (C++ getters on the node). This revision exposes no service/REST/health endpoint that surfaces audit health or lets an operator run `verify()` at runtime, so the signals are **not operator-observable at runtime yet** - a runtime read/verify/health surface is future work. With `audit_log.fail_closed` set, an append failure is re-raised as a **fail-FAST** error so a compliance-strict deployment learns the audit broke. This does **not** roll back the fault-state change that already committed: the audit log and the fault store are **separate SQLite databases**, so there is no cross-DB atomicity, and `fail_closed` is a broken-audit alarm requiring operator action, not a rollback. The default (`fail_closed=false`) keeps fault processing running; either way the in-process signals record the gap.

`verify()` walks the persisted chain oldest-first and recomputes every link: editing a row breaks its `record_hash`, and deleting a row breaks the next row's `prev_hash` linkage. Deleting the newest row _while leaving the head untouched_ is caught by the persisted-head check (the head is read straight from the DB). However, deleting the newest row(s) **and** repointing the head with a single `UPDATE audit_chain_head SET seq=..., record_hash=...` to the prior row's values costs no more than any other casual edit - it is **not** the "recompute the entire chain" the threat model below might suggest - and the truncated chain still verifies. There is no external record that a later `seq` ever existed, so this tail-truncation is undetectable by design.

**Threat model (read this).** The chain is **unkeyed**, and the head and segment anchors live in the **same writable SQLite file** as the rows. `verify()` therefore catches edits or deletions that did **not** also recompute the chain - that is, casual or accidental tampering, and the bookkeeping bugs that would otherwise lose records. The append-only triggers are defense-in-depth: `audit_log` rejects out-of-band UPDATE/DELETE, `audit_anchors` carries the same guard-gated triggers so an out-of-band INSERT/UPDATE/DELETE of an anchor is rejected too, and the rotation-prune guard (`audit_prune_guard`) is itself protected by a trigger so an external writer cannot simply flip it open and then delete a prefix (or forge an anchor) - that flip is only permitted from the in-process connection that holds a per-connection temp marker. The single-row chain head (`audit_chain_head`) is intentionally **not** trigger-protected (a trigger there would block the legitimate head update inside the append transaction); a casual edit or delete of the head is instead caught by `verify()` via the seq/hash/head-mismatch checks. None of this stops an attacker with write access to the file: such an attacker can create the same temp marker or drop the triggers, and recompute the entire chain (head and anchors included) to forge a self-consistent history - and cheaper still, the tail-truncation above and the forged prefix-truncation below need no recompute at all. The triggers are **not** a security boundary - this is tamper-**evident**, not tamper-**proof**. True tamper-_proofing_ requires a key or signature over the head (so it cannot be recomputed without the key) or external anchoring of the head hash to an append-only store you do not control; both are out of scope here and belong to the audit-log exporter / signing follow-up.

**Retention/rotation**: when more than `audit_log.retention_max_records` rows are retained, the oldest segment is _sealed_ (its final `seq` + hash are persisted as an anchor) and then pruned. The surviving tail still verifies because the oldest retained row links back to the sealed anchor. Only the anchor at the current prune boundary is kept - the same rotation drops older anchors - so `audit_anchors` stays bounded (one row) instead of growing one row per rotation. Because `verify()` treats any matching sealed anchor as a valid tail root, a **forged** prefix-truncation (an out-of-band actor deletes a prefix and inserts a matching anchor) is **indistinguishable** from legitimate pruning: "the surviving tail still verifies" therefore covers a forged truncation exactly as well as a real one. The guard-gated `audit_anchors` triggers raise the bar for this (casual/accidental only) but, like every trigger here, a write-capable adversary can drop them - so this stays tamper-**evident**, not tamper-**proof**.

| Parameter                         | Type   | Default | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| --------------------------------- | ------ | ------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `audit_log.enabled`               | bool   | `false` | Enable the tamper-evident audit log                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| `audit_log.transitions`           | string | `"all"` | Which transitions to record: `"all"` (occurred/confirmed/healed/cleared) or `"confirmed_only"`. Lifecycle markers are always recorded.                                                                                                                                                                                                                                                                                                                                                   |
| `audit_log.database_path`         | string | `""`    | SQLite path. Empty => sibling `fault_audit.db` next to the fault DB (or `:memory:` for in-memory fault stores)                                                                                                                                                                                                                                                                                                                                                                           |
| `audit_log.retention_max_records` | int    | `0`     | Seal + prune the oldest segment beyond this many retained records (0 = unlimited)                                                                                                                                                                                                                                                                                                                                                                                                        |
| `audit_log.fail_closed`           | bool   | `false` | When `true`, an audit append failure is re-raised as a fail-FAST error signalling the audit chain is broken and needs operator action. It does **not** roll back the already-committed fault-state change (the fault store is a separate DB - no cross-DB atomicity). When `false`, the failure is logged and counted and fault processing continues. Either way the gap is recorded via the in-process dropped-writes / audit-healthy signals (not operator-observable at runtime yet). |

## Usage

### Launch

```bash
# Default (SQLite storage, immediate confirmation)
ros2 launch ros2_medkit_fault_manager fault_manager.launch.py

# With custom database path
ros2 run ros2_medkit_fault_manager fault_manager_node --ros-args \
  -p database_path:=/custom/path/faults.db

# With in-memory storage (no persistence)
ros2 run ros2_medkit_fault_manager fault_manager_node --ros-args \
  -p storage_type:=memory
```

## Advanced: Debounce Filtering

For systems that need to filter transient faults, enable debounce filtering by setting a lower `confirmation_threshold`.

### Configuration

```bash
# For a reporter that repeats its events while a condition holds.
# See "Choosing the right lever for your reporter" below before copying this.
ros2 run ros2_medkit_fault_manager fault_manager_node --ros-args \
  -p confirmation_threshold:=-3 \
  -p healing_enabled:=true \
  -p healing_threshold:=3
```

### How It Works

The fault manager uses an AUTOSAR DEM-style debounce model:

- **FAILED events** (fault detected): Decrement the internal counter
- **PASSED events** (fault cleared): Increment the internal counter
- Fault becomes **CONFIRMED** when counter reaches `confirmation_threshold`
- Fault becomes **HEALED** when counter reaches `healing_threshold` (if enabled)

The counter is always clamped to `[confirmation_threshold, healing_threshold]`, so a long run of
one-sided events cannot push it out to the integer limits and delay the opposite transition.
`confirmation_threshold < 0 <= healing_threshold` is required (`healing_threshold = 0` heals on a
single PASSED event). A positive confirmation threshold or a negative healing threshold is
sign-flipped with a warning, so `5` becomes `-5`; a confirmation threshold of `0` is then
rejected and falls back to `-1`.

`CONFIRMED` and `HEALED` are **latched** (hysteresis): once reached, the status holds until the
counter reaches the opposite threshold, so a single opposite-direction event cannot flip it. As a
result a fault that becomes active again can take up to `healing_threshold - confirmation_threshold`
events to return to the default (CONFIRMED-only) list. During that window `last_occurred` still
reflects the activity; `occurrence_count` does not, because it counts the edge that started the
occurrence, not every report within it.

### Choosing the right lever for your reporter

The counter only moves when an event arrives, so the count-based settings above work only for a
reporter that keeps sending FAILED while the condition is still there. A reporter that samples a
value on a timer and reports on every sample is of that kind.

Many reporters do not work that way. They send one FAILED when the condition appears and one clear
when it goes away, and nothing in between. For such a reporter the second FAILED never arrives, so
`confirmation_threshold: -3` means the fault stays PREFAILED and never confirms. The default fault
list returns CONFIRMED only, so the fault is invisible. Healing has the same problem in reverse:
`healing_threshold: 3` needs four consecutive PASSED events after a fault confirmed at `-1`, and
only one PASSED is ever sent, so the fault stays CONFIRMED until someone calls `~/clear_fault`.

Pick by how your reporter behaves:

| Reporter repeats FAILED while the condition holds | Reporter sends one event per transition |
|---|---|
| `confirmation_threshold: -N` confirms on the Nth FAILED, so it rides out N-1 noisy samples | `confirmation_threshold` cannot filter here; see below |
| `healing_threshold: N` needs `N - confirmation_threshold` clean samples | `healing_threshold: 0` heals on the single PASSED |

For the second column, `auto_confirm_after_sec` looks like it fills the gap: it promotes a fault
that has stayed PREFAILED for that long, without changing the counter. Pairing it with
`confirmation_threshold: -2` does keep a single FAILED out of CONFIRMED. It also has a trap. HEALED
is latched, and leaving that latch costs `healing_threshold - confirmation_threshold` FAILED
events, which at `-2` is two. A reporter that sends one means the SECOND occurrence of a fault code
never confirms again, and `occurrence_count` does not move either. Prefer leaving
`confirmation_threshold` at `-1` for such a reporter, and filter noisy samples in the reporter
itself, where the samples are.

Two things ignore the counter. A FAILED report on a record stored as `SEVERITY_CRITICAL` confirms
at once unless `critical_immediate_confirm` is turned off in the debounce config, and that field is
not exposed as a ROS parameter. `auto_confirm_after_sec` promotes on elapsed time since the last FAILED, which is
not the same as observing that the condition is still there.

### Fault Lifecycle with Debounce

```
     FAILED events          PASSED events
          |                      |
          v                      v
   [counter--]             [counter++]
          |                      |
          v                      v
PREFAILED -----> CONFIRMED -----> HEALED (retained)
  (counter     (counter <=     (counter >=
   < 0)         threshold)      healing)
                    |
                    v
                CLEARED (manual via ~/clear_fault)
```

### Status Reference

| Status | Description |
|--------|-------------|
| `PREFAILED` | Not yet confirmed. Usually a negative counter, but a fault that returns to 0 keeps the status it had |
| `CONFIRMED` | Fault is active and verified |
| `HEALED` | Resolved via PASSED events (if healing enabled) |
| `CLEARED` | Manually acknowledged via `~/clear_fault` |

### Testing Debounce

```bash
# Report FAILED events (need 3 to confirm with threshold=-3)
ros2 service call /fault_manager/report_fault ros2_medkit_msgs/srv/ReportFault \
  "{fault_code: 'SENSOR_FAIL', event_type: 0, severity: 2, description: 'Sensor timeout', source_id: '/sensor'}"

# Report PASSED event (fault condition cleared)
ros2 service call /fault_manager/report_fault ros2_medkit_msgs/srv/ReportFault \
  "{fault_code: 'SENSOR_FAIL', event_type: 1, severity: 0, description: '', source_id: '/sensor'}"

# Query all statuses including PREFAILED
ros2 service call /fault_manager/list_faults ros2_medkit_msgs/srv/ListFaults \
  "{statuses: ['PREFAILED', 'CONFIRMED', 'HEALED']}"
```

Event types: `0` = EVENT_FAILED, `1` = EVENT_PASSED

### Immediate Confirmation

A FAILED report on a record whose stored severity is CRITICAL confirms it at once, regardless of
threshold and of the HEALED latch. The stored severity decides, not the report: after a CRITICAL
occurrence is cleared, one WARN report for the same record confirms the next occurrence, which
shows CRITICAL. This holds under a per-entity threshold override too.

## Advanced: Fault Correlation

Fault correlation reduces noise by identifying relationships between faults. When enabled, symptom faults
(effects of a root cause) can be muted and auto-cleared when the root cause is resolved.

### Correlation Modes

**Hierarchical**: Defines explicit root cause → symptoms relationships. When a root cause fault occurs,
subsequent matching symptom faults within a time window are correlated and optionally muted.

**Auto-Cluster**: Automatically groups related faults that match a pattern within a time window.
Useful for detecting "storms" of related faults (e.g., communication errors).

### Configuration

Enable correlation by providing a YAML configuration file:

```bash
ros2 run ros2_medkit_fault_manager fault_manager_node --ros-args \
  -p correlation.config_file:=/path/to/correlation.yaml
```

### Correlation Parameters

| Parameter                          | Type   | Default | Description                                                     |
| ---------------------------------- | ------ | ------- | --------------------------------------------------------------- |
| `correlation.config_file`          | string | `""`    | Path to correlation YAML config (empty = disabled)              |
| `correlation.cleanup_interval_sec` | double | `5.0`   | Interval for cleaning up expired pending correlations (seconds) |

### Configuration File Format

```yaml
correlation:
  enabled: true
  default_window_ms: 500 # Default time window for symptom detection

  # Reusable fault patterns (supports wildcards with *)
  patterns:
    motor_errors:
      codes: ["MOTOR_COMM_*", "MOTOR_TIMEOUT_*"]
    drive_faults:
      codes: ["DRIVE_*"]
    comm_errors:
      codes: ["*_COMM_*", "*_TIMEOUT"]

  rules:
    # Hierarchical rule: E-Stop causes motor and drive faults
    - id: estop_cascade
      name: "E-Stop Cascade"
      mode: hierarchical
      root_cause:
        codes: ["ESTOP_001", "ESTOP_002"]
      symptoms:
        - pattern: motor_errors
        - pattern: drive_faults
      window_ms: 1000 # Symptoms within 1s of root cause
      mute_symptoms: true # Don't publish symptom events
      auto_clear_with_root: true # Clear symptoms when root cause clears

    # Auto-cluster rule: Group communication errors
    - id: comm_storm
      name: "Communication Storm"
      mode: auto_cluster
      match:
        - pattern: comm_errors
      min_count: 3 # Need 3 faults to form cluster
      window_ms: 500 # Within 500ms
      show_as_single: true # Only show representative fault
      representative: highest_severity # first | most_recent | highest_severity
```

### Pattern Wildcards

Patterns support `*` wildcard matching:

- `MOTOR_*` matches `MOTOR_COMM`, `MOTOR_TIMEOUT`, `MOTOR_DRIVE_FAULT`
- `*_COMM_*` matches `MOTOR_COMM_FL`, `SENSOR_COMM_TIMEOUT`
- `*_TIMEOUT` matches `MOTOR_TIMEOUT`, `SENSOR_TIMEOUT`

### Querying Correlation Data

Use `include_muted` and `include_clusters` to retrieve correlation information:

```bash
# Get faults with muted fault details
ros2 service call /fault_manager/list_faults ros2_medkit_msgs/srv/ListFaults \
  "{statuses: ['CONFIRMED'], include_muted: true, include_clusters: true}"
```

Response includes:

- `muted_count`: Number of muted symptom faults
- `cluster_count`: Number of active fault clusters
- `muted_faults[]`: Details of muted records (when `include_muted=true`), one entry per muted
  record. Muting is per record, so a symptom muted under one owner's root cause never hides
  another owner's record of the same code, and each entry names its owner in `source_id` (`*` for a
  shared record).
- `clusters[]`: Details of active clusters (when `include_clusters=true`)

### REST API (via Gateway)

Query parameters for GET `/api/v1/faults`:

- `include_muted=true`: Include muted fault details in response
- `include_clusters=true`: Include cluster details in response

Response fields:

```json
{
  "faults": [...],
  "count": 5,
  "muted_count": 2,
  "cluster_count": 1,
  "muted_faults": [
    {
      "fault_code": "MOTOR_COMM_FL",
      "root_cause_code": "ESTOP_001",
      "rule_id": "estop_cascade",
      "delay_ms": 50,
      "source_id": "*"
    }
  ],
  "clusters": [
    {
      "cluster_id": "comm_storm_1",
      "rule_id": "comm_storm",
      "rule_name": "Communication Storm",
      "representative_code": "SENSOR_TIMEOUT",
      "representative_severity": "CRITICAL",
      "fault_codes": ["MOTOR_COMM_FL", "SENSOR_TIMEOUT", "DRIVE_COMM_ERR"],
      "count": 3,
      "first_at": 1705678901.123,
      "last_at": 1705678901.456
    }
  ]
}
```

When clearing a root cause fault, `auto_cleared_codes` lists symptoms that were auto-cleared:

```json
{
  "status": "success",
  "fault_code": "ESTOP_001",
  "message": "Fault cleared",
  "auto_cleared_codes": ["MOTOR_COMM_FL", "MOTOR_COMM_FR", "DRIVE_FAULT"]
}
```

### Example: E-Stop Cascade

1. E-Stop is triggered → `ESTOP_001` fault reported
2. Motors lose power → `MOTOR_COMM_FL`, `MOTOR_COMM_FR` faults reported
3. Correlation engine detects motor faults are symptoms of E-Stop
4. Motor faults are muted (not published as events, but stored)
5. Dashboard shows only `ESTOP_001` (root cause)
6. When E-Stop is cleared → Motor faults are auto-cleared

## Namespaced Deployment

The fault manager can run in a custom ROS 2 namespace. The gateway resolves service and topic
names automatically via the `fault_manager.namespace` parameter:

```yaml
# gateway_params.yaml
ros2_medkit_gateway:
  ros__parameters:
    fault_manager.namespace: "robot1"      # -> /robot1/fault_manager/list_faults
    fault_manager.service_timeout_sec: 5.0
```

Launch the fault manager in a namespace:

```bash
ros2 launch ros2_medkit_fault_manager fault_manager.launch.py \
  namespace:=robot1
```

Leading slashes are optional - `"robot1"` and `"/robot1"` are equivalent.

## Building

```bash
colcon build --packages-select ros2_medkit_fault_manager
source install/setup.bash
```

### ROS-free core

The package has two parts. The core compiles without ROS and without DDS setup.
`scripts/check_cores_without_ros` builds it with plain CMake, runs a check program against it with no
ROS (a fault through `SqliteFaultStorage`, and `FaultManagerCore` with a clock the program sets), and
fails when the program links rclcpp. The unit test suites link `fault_manager_lib`, which uses rclcpp,
and some of them build an `rclcpp::Clock`.

**Core** (`fault_manager_core`, static library, no ROS):

- the storage backends (SQLite, in-memory, optional PostgreSQL), the threshold resolvers, the
  correlation engine and the audit log;
- `core::FaultManagerConfig`: every startup value, named after its parameter. `validate()` applies
  the clamps and fallbacks and returns the lines to log as `ConfigNotice` values;
- `core::build_core_parts()`: opens the storage and the audit log, loads the threshold and
  correlation files. It throws when the storage or the audit log cannot be opened.
  `core::reclassify_stale_healed()` is the startup pass for stale HEALED rows;
- `core::FaultManagerCore`: `report`, `clear`, `list`, `get`, `list_for_entity`, `record_transition`
  and `tick`. These calls run under one internal mutex.

The core reaches the outside through four interfaces. The ROS adapter implements them.

| Interface | Role |
| --- | --- |
| `core::EventSink` | `publish()` receives every fault event the core emits. |
| `core::CaptureHooks` | `capture_now()` queues the snapshot and rosbag capture of a fault code. The core has already applied the recapture cooldown. `on_prefailed()` and `on_cleared()` notify the capture side. |
| `core::WallClock` | `now_ns()` returns nanoseconds since the Unix epoch. |
| `core::LogSink` | `log()` receives the log lines. |

`EventSink` and `CaptureHooks` run inside the core mutex and must not call back into the core, or
the call deadlocks. `LogSink::log()` is called from several threads at once (core callers and
capture workers), so it must be thread-safe.

`tick(wall_now_ns)` runs the periodic jobs that are due: time-based auto-confirmation
every second and the correlation cleanup every `correlation.cleanup_interval_sec`. A job runs from half
a period before its due time. After a run its next due time stays on the job's own grid, so a late
tick does not delay the run after it. The core has no timer. The caller calls `tick` with the wall clock, at least as often as the shortest period it needs.

The core returns `ros2_medkit::fault_detection` values (`FaultRecord`, `FaultEventRecord`) with
nanosecond times.

**ROS adapter** (`fault_manager_lib` and `fault_manager_node`): the services, the `FaultEvent`
publisher, wall timers that call `tick`, the capture thread pool, snapshot and rosbag capture. It
implements the four interfaces: `ros2::RosEventSink`, `ros2::RosCaptureHooks`,
`ros2::SystemWallClock` and `ros2::RosLogSink`. It converts between the core values and the
messages in `ros2/fault_msg_conversions.hpp`.

### Plain-CMake build

`cmake/fault_manager_core.cmake` is plain CMake (3.16 or newer, C++17), no ament. The ament build
includes it by path, and so does `scripts/check_cores_without_ros`. The directories
`ros2_medkit_fault_detection` and `ros2_medkit_fault_manager` must stay side by side, because the file
finds the shared `fault_types.hpp` at `../ros2_medkit_fault_detection/include`. The file looks the
libraries up itself when the project has not.

| Dependency | When |
| --- | --- |
| SQLite3 | always |
| OpenSSL (`OpenSSL::Crypto`) | always |
| nlohmann_json | always |
| yaml-cpp | always |
| libpqxx >= 7.5 and PostgreSQL (libpq) | only with `-DPOSTGRES_SUPPORT=ON` |

The core is built from a `core::FaultManagerConfig`: call `validate()` and log the notices, then
`build_core_parts(config, sink)`, and pass the parts, the config and the four interface
implementations to `FaultManagerCore`. The fault reporter core is a separate target, see
`ros2_medkit_fault_reporter`. Both files can be included in one project and share the
`ros2_medkit_fault_detection` target. Neither target is installed or exported.
`scripts/check_cores_without_ros/run.sh` builds and runs both on Ubuntu 20.04 with gcc 9, the oldest
compiler the cores are checked with. Ubuntu 20.04 does not package libpqxx 7.5, so the PostgreSQL
backend is not built there. SQLite is the default.

## Testing

```bash
colcon test --packages-select ros2_medkit_fault_manager
colcon test-result --verbose
```

## License

Apache-2.0
