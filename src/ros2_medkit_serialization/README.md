# ros2_medkit_serialization

Runtime JSON ↔ ROS 2 message serialization library for the ros2_medkit gateway.

## Overview

This package provides dynamic message serialization using [dynmsg](https://github.com/osrf/dynamic_message_introspection), enabling JSON ↔ ROS 2 message conversion at runtime without compile-time type dependencies. It eliminates CLI fallbacks in the gateway by providing native C++ APIs for all message operations.

## Features

- **JSON Serialization**: Convert ROS 2 messages to/from JSON (via YAML bridge)
- **CDR Serialization**: Serialize JSON to CDR format for `GenericPublisher` and `GenericClient`
- **CDR Deserialization**: Deserialize CDR data from `GenericSubscription` to JSON
- **Type Caching**: Thread-safe caching of type introspection data with LRU eviction
- **Service/Action Types**: Helper functions for request/response/goal type derivation
- **Schema Generation**: Generate JSON schemas for ROS 2 message types

## Dependencies

- `dynmsg` - OSRF dynamic message introspection library
- `nlohmann_json` - JSON library
- `yaml_cpp_vendor` - YAML library (for dynmsg compatibility)
- `rclcpp` - ROS 2 client library (for SerializedMessage)

## Usage

```cpp
#include "ros2_medkit_serialization/json_serializer.hpp"
#include "ros2_medkit_serialization/type_cache.hpp"

// Get type info (cached)
auto& cache = ros2_medkit_serialization::TypeCache::instance();
auto type_info = cache.get("std_msgs", "String");

// Create serializer
ros2_medkit_serialization::JsonSerializer serializer;

// Serialize JSON to CDR for GenericPublisher
nlohmann::json msg_json = {{"data", "Hello World"}};
auto serialized = serializer.serialize("std_msgs/msg/String", msg_json);
// Use with: generic_publisher->publish(serialized);

// Deserialize CDR to JSON from GenericSubscription
nlohmann::json result = serializer.deserialize("std_msgs/msg/String", serialized_msg);

// Convert in-memory message to JSON
nlohmann::json json = serializer.to_json(type_info, message_ptr);

// Generate schema for a type
nlohmann::json schema = serializer.get_schema("geometry_msgs/msg/Twist");
```

## API Reference

### JsonSerializer

| Method | Description |
|--------|-------------|
| `serialize(type, json)` | Serialize JSON to CDR `SerializedMessage` |
| `deserialize(type, msg)` | Deserialize CDR to JSON |
| `to_json(type_info, ptr)` | Convert in-memory message to JSON |
| `from_json(type, json)` | Create message from JSON |
| `get_schema(type)` | Get JSON schema for type |
| `get_defaults(type)` | Get default values as JSON |
| `yaml_to_json(yaml)` | Convert YAML node to JSON (static) |

### TypeCache

| Method | Description |
|--------|-------------|
| `instance()` | Get singleton instance |
| `get(pkg, name)` | Get cached type info |
| `get(type_string)` | Get cached type info from full type string |
| `clear()` | Clear all cached entries |

## Building

```bash
cd /path/to/ros2_medkit
colcon build --packages-select ros2_medkit_serialization
```

## Testing

```bash
colcon test --packages-select ros2_medkit_serialization
colcon test-result --verbose
```

## Design Documentation

See the [design documentation](../../docs/design/ros2_medkit_serialization/index.rst) for architecture details.

## License

Apache-2.0
