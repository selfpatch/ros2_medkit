# check_cores_without_ros

Checks that the ROS-free cores (fault reporter core, fault manager core, gateway core) build and
run without ROS. It builds them with plain CMake and gcc 9 on Ubuntu 20.04, the oldest compiler
the cores are checked with, then runs a small program that links them. The program also starts
the gateway REST server over minimal `GatewayServices` and reads `/api/v1/health`, `/api/v1/` and `/api/v1/version-info`. The check
fails if the binary links `rclcpp`.

```bash
scripts/check_cores_without_ros/run.sh
```

The CMake file includes `cmake/fault_reporter_core.cmake`, `cmake/fault_manager_core.cmake` and
`cmake/gateway_core.cmake` by path, as the ament builds do. The image installs only g++-9,
cmake, make and the dev packages of SQLite3, OpenSSL, nlohmann_json and yaml-cpp. libpqxx is
not installed, so the PostgreSQL backend is not built.

## What `main.cpp` checks

- Cyclic subscriptions: with `register_builtin_samplers()` and the SSE transport, a subscription on a data resource (served by
  a controlled `TopicDataProvider`), on a resource a plugin's `DataProvider` serves, and on faults each stream two events whose
  values differ.
- Freeze frames: a capture built by `make_freeze_frame_capture()` over a controlled `FaultEventSource` and a plugin's
  `DataProvider` freezes the entity's values on a fault confirmation. The fault detail over HTTP holds a `freeze_frame` with the
  value from the confirmation, not the one set after it. An entity no plugin serves gets none, the oldest fault's frames are
  evicted past `max_faults`, a re-confirmation replaces the frame, and a disabled configuration builds no capture.
- Fault event fan-out: one upstream source shared through `FaultEventFanOut` by the fault SSE stream (read over HTTP) and the
  capture: a confirmation shows up as an SSE event and as a freeze frame. Without the fan-out one source serves one consumer only.
- Triggers: a host built by `make_trigger_host()` over the fan-out of the single fault event source. A fault trigger created over
  HTTP fires on a confirmation, its event is read from the trigger's event stream, and the fault SSE stream still gets the
  confirmation. A data trigger on a plugin-served item does not fire while the value is below its range, fires once when the value
  enters it, and the item is read again and again by the core's polling source. A fourth trigger with `max_triggers = 3` answers
  503. A limit below 1 or past `INT_MAX` and a polling interval outside 100-60000 ms fall back with a notice.
- Plugin context: `context_plugin.cpp` builds as a shared object without ROS. The core `PluginManager` loads it, a
  `make_core_plugin_context()` context goes to it, and its routes and `DataProvider` answer over HTTP. The plugin gets a null
  `node()`, reads the host's entities, registers a capability, takes, checks and releases a lock, lists faults, reaches the
  notifier and the condition registry, and its entity change notification reaches the host's callback. A route on an unknown
  entity answers 404.
- Fault-trigger engine: a host built by `make_fault_trigger_host()` over that plugin. A rule created over HTTP reports nothing
  while the plugin's value is below the threshold. When the value crosses it, the fault goes through the host's fault manager
  into the host's single event source, and the fault SSE stream and the freeze-frame capture (which holds the value that crossed
  the threshold) both get the confirmation. When the value comes back the engine clears the fault. A rule on an unknown app
  answers 404, a disabled configuration or a host with no plugin builds no engine, and a polling interval below 50 ms is raised
  and one past `INT_MAX` is capped, each with a notice.
- Bulk data: a store built by `make_bulk_data_store()` takes an upload over HTTP, lists it, returns it byte-equal and deletes
  it; an upload past `max_upload_size` is refused; the categories do not contain `rosbags` when the fault manager serves none.
- `GET /` capabilities: `bulk_data` and `cyclic_subscriptions` are false for services without a store, rosbags, transport or
  sampler, and true as soon as a store or rosbags (bulk data), or a transport and a sampler (subscriptions), exist.
- Lock expiry: a lock with a one second expiry, taken over HTTP with a cyclic subscription in its scope, is gone after the
  expiry and the subscription is removed, with a `LockReaper` as the only cleanup.
- Scripts: `make_script_host()` over a scripts directory gives a manager with a backend (none without a directory); a script
  uploaded over HTTP runs, completes and its JSON output is readable.
- Aggregation: a second REST server in the same process is the peer of the first through `AggregationHost`; `merge_peers()`
  returns a routing table and the first server reads the peer's entity from the peer.
- Lifecycle without a reader: `/status` reports readiness from presence (also for an app with lifecycle services) and a
  transition answers 501.
- Gateway: with `server_name`, `vendor_name` and `server_version` set in `RestConfig`, `GET /api/v1/` returns the name and version, and `GET /api/v1/version-info` returns the vendor name and version.

- Reporter core: a report below the local filter threshold is filtered, the next one reaches the sink, a PASSED report is
  filtered by the same threshold, an empty fault code is not forwarded.
- Fault manager storage: a FAILED report creates a CONFIRMED fault, and `list_faults` returns it with its nanosecond time.
- `FaultManagerConfig::validate()`: a positive `confirmation_threshold` flips its sign, a NaN `auto_confirm_after_sec`
  falls back to 0, int64 thresholds outside the int32 range clamp to its ends, and each change has a notice.
- `build_core_parts()` opens a SQLite storage; audit and correlation stay off by default.
- `FaultManagerCore`: report, list and clear, with the `EventSink` seeing `fault_confirmed` and then `fault_cleared`.
- `FaultManagerCore::tick()`: auto-confirmation and the correlation cleanup run on their own periods, and not twice
  within one period. `CaptureHooks::capture_now` is called once on confirmation. A tick earlier than the previous one runs
  nothing, and the jobs then wait at most one period.
- A `correlation.cleanup_interval_sec` of 9e9 does not overflow the due time.

The Dockerfile also runs `ldd` on the binary and on the plugin and fails when `rclcpp` appears.

The script sends only the needed source directories to `docker build` as the build
context. The build runs with `-q`; when it fails, the script repeats it with the full log, so the compiler message is printed. CI runs the same CMake steps in an `ubuntu:20.04` container (job `cores-without-ros`).
`COLCON_IGNORE` keeps colcon from treating this directory as a package.
