# ros2_medkit_topic_beacon

Gateway discovery plugin that subscribes to ROS 2 topics for real-time entity metadata.
Enables push-based beacon discovery where nodes publish `MedkitDiscoveryHint` messages
to enrich the SOVD entity tree.

## How It Works

1. Nodes publish `ros2_medkit_msgs::msg::MedkitDiscoveryHint` messages on a beacon topic
2. The plugin subscribes, validates hints, and stores them with stamp-based TTL
3. A token bucket rate limiter prevents overload from high-frequency publishers
4. Results are exposed at the `x-medkit-topic-beacon` vendor extension endpoint

Hints transition through states: **active** (within TTL) -> **stale** (TTL expired,
data still served with stale marker) -> **expired** (removed from store).

`beacon_ttl_sec` takes 0.1 to 2147483647 s, `beacon_expiry_sec` 1.0 to 2147483647 s and
`max_messages_per_second` 1 to 10000. A value above its maximum, including `.inf` in a
parameter file, becomes the maximum; NaN, `-.inf` and a value below the minimum become the
minimum. Each replacement is logged as a warning. Messages over the rate limit are dropped
without a log.

`max_hints` takes an integer from 1 to 2147483647. A 64-bit integer outside that range becomes
the nearer bound. The parameter parser reads an integer that does not fit in 64 bits as a
double. A double, such as `1.0e12` or `.nan`, is refused and the default 10000 is used. Both
are logged as warnings.

## Configuration

```yaml
plugins: ["topic_beacon"]
plugins.topic_beacon.path: "/path/to/libros2_medkit_topic_beacon.so"
plugins.topic_beacon.beacon_ttl_sec: 10.0
plugins.topic_beacon.beacon_expiry_sec: 300.0
plugins.topic_beacon.allow_new_entities: true
plugins.topic_beacon.max_messages_per_second: 100.0
```

See [discovery options](https://selfpatch.github.io/ros2_medkit/config/discovery-options.html)
for full configuration reference.

## When to Use

Use the topic beacon when entity metadata **changes in real time** - sensor health,
connection quality, load metrics. For stable metadata that rarely changes, use the
[parameter beacon](../ros2_medkit_param_beacon/) instead.

## License

Apache License 2.0
