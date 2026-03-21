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

## Configuration

```yaml
plugins: ["topic_beacon"]
plugins.topic_beacon.path: "/path/to/libros2_medkit_topic_beacon.so"
plugins.topic_beacon.ttl_sec: 30
plugins.topic_beacon.expiry_sec: 90
plugins.topic_beacon.allow_new_entities: true
plugins.topic_beacon.rate_limit_hz: 10.0
```

See [discovery options](https://selfpatch.github.io/ros2_medkit/config/discovery-options.html)
for full configuration reference.

## When to Use

Use the topic beacon when entity metadata **changes in real time** - sensor health,
connection quality, load metrics. For stable metadata that rarely changes, use the
[parameter beacon](../ros2_medkit_param_beacon/) instead.

## License

Apache License 2.0
