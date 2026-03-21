# ros2_medkit_param_beacon

Gateway discovery plugin that polls ROS 2 node parameters for entity metadata.
Enables pull-based beacon discovery where nodes advertise diagnostic hints
through standard ROS 2 parameters.

## How It Works

1. The plugin queries nodes for a known beacon parameter (JSON-encoded `BeaconHint`)
2. Hints are validated via `BeaconValidator` and stored in `BeaconHintStore` with TTL
3. Entity metadata is mapped into the SOVD hierarchy via `BeaconEntityMapper`
4. Results are exposed at the `x-medkit-param-beacon` vendor extension endpoint

In non-hybrid discovery mode, the plugin discovers poll targets automatically from the
ROS 2 graph. In hybrid mode, targets come from the manifest.

## Configuration

```yaml
plugins: ["param_beacon"]
plugins.param_beacon.path: "/path/to/libros2_medkit_param_beacon.so"
plugins.param_beacon.poll_interval_sec: 10
plugins.param_beacon.poll_budget_ms: 500
plugins.param_beacon.timeout_ms: 1000
plugins.param_beacon.ttl_sec: 60
plugins.param_beacon.expiry_sec: 120
```

See [discovery options](https://selfpatch.github.io/ros2_medkit/config/discovery-options.html)
for full configuration reference.

## When to Use

Use the parameter beacon when entity metadata is **stable and infrequently updated** -
hardware descriptions, capabilities, firmware versions. For real-time metadata that
changes frequently, use the [topic beacon](../ros2_medkit_topic_beacon/) instead.

## License

Apache License 2.0
