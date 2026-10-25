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
- Bulk data: a store built by `make_bulk_data_store()` takes an upload over HTTP, lists it, returns it byte-equal and deletes
  it; an upload past `max_upload_size` is refused; the categories do not contain `rosbags` when the fault manager serves none.
- `GET /` capabilities: `bulk_data` and `cyclic_subscriptions` are false for services without a store, rosbags, transport or
  sampler, and true as soon as a store or rosbags (bulk data), or a transport and a sampler (subscriptions), exist.
- Lock expiry: a lock with a one second expiry, taken over HTTP with a cyclic subscription in its scope, is gone after the
  expiry and the subscription is removed, with a `LockReaper` as the only cleanup.
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

The Dockerfile also runs `ldd` on the binary and fails when `rclcpp` appears.

The script sends only the needed source directories to `docker build` as the build
context. CI runs the same CMake steps in an `ubuntu:20.04` container (job `cores-without-ros`).
`COLCON_IGNORE` keeps colcon from treating this directory as a package.
