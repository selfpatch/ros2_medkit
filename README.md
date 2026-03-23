# ros2_medkit_graph_provider

Gateway plugin that provides ROS 2 topic graph analysis with latency, frequency,
and drop-rate metrics. This is the default `IntrospectionProvider` - extracted
from `ros2_medkit_gateway` into a standalone plugin package in v0.4.0.

## What It Does

- Subscribes to `/diagnostics` for hardware-level metrics
- Builds a topic graph with per-topic metrics: frequency (Hz), latency (ms), drop rate (%)
- Detects stale topics with no recent data
- Tracks which nodes published to which topics
- Provides graph state snapshots via the `x-medkit-graph` vendor extension endpoint

## Configuration

Loaded as a gateway plugin via `gateway.launch.py` (configured by default):

```yaml
plugins: ["graph_provider"]
plugins.graph_provider.path: "/path/to/libros2_medkit_graph_provider.so"
plugins.graph_provider.expected_frequency_hz: 30.0
```

## Architecture

The plugin implements `GatewayPlugin` + `IntrospectionProvider`:

- `GraphProviderPlugin` - Main plugin class with ROS 2 subscriptions
- `GraphBuildState` - Internal state tracking topic metrics and staleness
- `GraphBuildConfig` - Configuration (expected frequency defaults)

Entity cache is populated from the ROS 2 graph during each discovery cycle
via the merge pipeline's `PluginLayer`.

## License

Apache License 2.0
