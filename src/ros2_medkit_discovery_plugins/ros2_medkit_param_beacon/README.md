# ros2_medkit_param_beacon

Gateway discovery plugin that polls ROS 2 node parameters for entity metadata.
Enables pull-based beacon discovery where nodes advertise diagnostic hints
through standard ROS 2 parameters.

## How It Works

1. The plugin polls each node's parameter service for parameters under a configurable prefix (default: `ros2_medkit.discovery`), mapping individual parameters to `BeaconHint` fields
2. Hints are validated via `BeaconValidator` and stored in `BeaconHintStore` with TTL
3. Entity metadata is mapped into the SOVD hierarchy via `BeaconEntityMapper`
4. Results are exposed at the `x-medkit-param-beacon` vendor extension endpoint

In `runtime_only` and `manifest_only` discovery the plugin reads its poll targets from the
ROS 2 graph. In `hybrid` mode the merge pipeline passes it the discovered Apps, but the
gateway's refresh then calls it again with no Apps, which clears them, so a poll cycle mostly
reads the graph there too. A graph read skips hidden nodes (a name starting with `_`), the
gateway's own node and its helper nodes (`<gateway>_sub`, `<gateway>_fault_clients`,
`<gateway>_lifecycle_state_reader`): they carry no beacon. The plugin remembers what its own
reads saw: a node that ran on one of its reads is not polled once the graph lists only a
leftover of it, an entry with an empty enclave and no endpoints, which a node whose
participant has left can leave behind (see "How long a departed node keeps being listed" in
the gateway's `docs/config/server.rst`).

Each node keeps one parameter client across poll cycles. A node that is no longer a target,
also when no node is, loses its client. A parameter request that gets no answer within
`param_timeout_sec` - waiting for the service, listing parameters or getting their values -
is given up and removed from its client, and the node is skipped for the next 1, 2, 4 and
then 8 poll cycles while it keeps timing out, so a node whose parameter services never answer
does not stall polling. A node that answers is polled on every cycle, also when its answer
carries no values: rclpy answers so when one of the parameters asked for is declared with a
type and no value, and the plugin then stores no hint. None of this is logged.

Every duration below takes its minimum (0.1 s, `beacon_expiry_sec` 1.0 s) up to 2147483647 s,
the longest wait Fast DDS keeps without the poll thread spinning. A value above the maximum,
including `.inf` in a parameter file, becomes the maximum; NaN, `-.inf` and a value below the
minimum become the minimum. Each replacement is logged as a warning.

`max_hints` takes an integer from 1 to 2147483647. A 64-bit integer outside that range becomes
the nearer bound. The parameter parser reads an integer that does not fit in 64 bits as a
double. A double, such as `1.0e12` or `.nan`, is refused and the default 10000 is used. Both
are logged as warnings.

## Configuration

```yaml
plugins: ["param_beacon"]
plugins.param_beacon.path: "/path/to/libros2_medkit_param_beacon.so"
plugins.param_beacon.poll_interval_sec: 10.0
plugins.param_beacon.poll_budget_sec: 10.0
plugins.param_beacon.param_timeout_sec: 2.0
plugins.param_beacon.beacon_ttl_sec: 15.0
plugins.param_beacon.beacon_expiry_sec: 300.0
```

See [discovery options](https://selfpatch.github.io/ros2_medkit/config/discovery-options.html)
for full configuration reference.

## When to Use

Use the parameter beacon when entity metadata is **stable and infrequently updated** -
hardware descriptions, capabilities, firmware versions. For real-time metadata that
changes frequently, use the [topic beacon](../ros2_medkit_topic_beacon/) instead.

## License

Apache License 2.0
