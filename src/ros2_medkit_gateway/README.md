# ros2_medkit_gateway

HTTP gateway node for the ros2_medkit diagnostics system.

## Overview

This package provides an HTTP server that exposes diagnostic information from the ROS 2 system through REST APIs.

## Endpoints

- `GET /health` - Health check endpoint returning node status
- `GET /` - Service information and available endpoints

## Building

```bash
colcon build --packages-select ros2_medkit_gateway
```

## Running

```bash
ros2 run ros2_medkit_gateway gateway_node
```

### Parameters

- `port` (int, default: 8080) - HTTP server port
- `host` (string, default: "0.0.0.0") - HTTP server host address

### Example with custom port

```bash
ros2 run ros2_medkit_gateway gateway_node --ros-args -p port:=9090
```

## Dependencies

- ROS 2 (rclcpp)
- cpp-httplib (automatically fetched during build)

## License

Apache-2.0
