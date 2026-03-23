# ros2_medkit_beacon_common

Shared C++ library for beacon-based discovery plugins. Provides hint validation,
entity mapping, response building, and rate limiting used by both `ros2_medkit_topic_beacon`
and `ros2_medkit_param_beacon`.

## Components

| Class | Purpose |
|-------|---------|
| `BeaconHint` | Data struct for discovery hints (entity ID, topology, transport, process diagnostics, metadata) |
| `BeaconHintStore` | Thread-safe hint storage with TTL-based expiration and deduplication |
| `BeaconValidator` | Input validation for entity IDs, required fields, and hint consistency |
| `BeaconEntityMapper` | Maps beacon hints to SOVD entity hierarchy (Apps, Components, Functions, Areas) |
| `BeaconResponseBuilder` | Builds JSON responses for gateway vendor extension endpoints |
| `TokenBucket` | Thread-safe rate limiter for high-frequency beacon streams |

## Usage

This package is a dependency of the beacon discovery plugins. It is not used directly
by end users. Plugin developers building custom beacon types should link against this
library for validation and entity mapping.

```cmake
find_package(ros2_medkit_beacon_common REQUIRED)
target_link_libraries(my_beacon_plugin ros2_medkit_beacon_common::ros2_medkit_beacon_common)
```

## Related Packages

- [ros2_medkit_topic_beacon](../ros2_medkit_topic_beacon/) - Push-based beacon via ROS 2 topics
- [ros2_medkit_param_beacon](../ros2_medkit_param_beacon/) - Pull-based beacon via ROS 2 parameters

## License

Apache License 2.0
