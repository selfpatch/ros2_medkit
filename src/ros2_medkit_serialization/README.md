# ros2_medkit_serialization

Runtime JSON to ROS 2 message serialization library for the ros2_medkit gateway.

## Overview

This package provides a thin wrapper around [dynmsg](https://github.com/osrf/dynamic_message_introspection) to enable JSON ↔ ROS 2 message conversion at runtime. It is designed to eliminate CLI dependencies in the gateway.

## Features

- **JSON Serialization**: Convert ROS 2 messages to/from JSON (via YAML bridge)
- **Type Caching**: Thread-safe caching of type introspection data
- **Service/Action Types**: Helper functions for request/response/goal type handling

## Dependencies

- `dynmsg` - OSRF dynamic message introspection library
- `nlohmann_json` - JSON library
- `yaml_cpp_vendor` - YAML library (for dynmsg compatibility)

## Usage

```cpp
#include "ros2_medkit_serialization/json_serializer.hpp"
#include "ros2_medkit_serialization/type_cache.hpp"

// Get type info (cached)
auto& cache = ros2_medkit_serialization::TypeCache::instance();
auto type_info = cache.get("std_msgs", "String");

// Serialize message to JSON
ros2_medkit_serialization::JsonSerializer serializer;
nlohmann::json json = serializer.to_json(ros_message);

// Deserialize JSON to message
auto ros_msg = serializer.from_json("std_msgs/msg/String", json);
```

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

## License

Apache-2.0
