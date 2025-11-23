# ros2_medkit_gateway

HTTP gateway node for the ros2_medkit diagnostics system.

## Overview

The ROS 2 Medkit Gateway exposes ROS 2 system information and data through a RESTful HTTP API. It automatically discovers nodes in the ROS 2 system, organizes them into areas based on their namespaces, and provides endpoints to query and interact with them.

**Key Features:**
- **Auto-discovery**: Automatically detects ROS 2 nodes and topics
- **Area-based organization**: Groups nodes by namespace (e.g., `/powertrain`, `/chassis`, `/body`)
- **REST API**: Standard HTTP/JSON interface
- **Real-time updates**: Configurable cache refresh for up-to-date system state

## Endpoints

### Discovery Endpoints

- `GET /health` - Health check endpoint (returns healthy status)
- `GET /` - Gateway status and version information
- `GET /areas` - List all discovered areas (powertrain, chassis, body, root)
- `GET /components` - List all discovered components across all areas

### API Reference

#### GET /areas

Lists all discovered areas in the system.

**Example:**
```bash
curl http://localhost:8080/areas
```

**Response:**
```json
[
  {
    "id": "powertrain",
    "namespace": "/powertrain",
    "type": "Area"
  },
  {
    "id": "chassis",
    "namespace": "/chassis",
    "type": "Area"
  }
]
```

#### GET /components

Lists all discovered components across all areas.

**Example:**
```bash
curl http://localhost:8080/components
```

**Response:**
```json
[
  {
    "id": "temp_sensor",
    "namespace": "/powertrain/engine",
    "fqn": "/powertrain/engine/temp_sensor",
    "type": "Component",
    "area": "powertrain"
  },
  {
    "id": "rpm_sensor",
    "namespace": "/powertrain/engine",
    "fqn": "/powertrain/engine/rpm_sensor",
    "type": "Component",
    "area": "powertrain"
  }
]
```

**Response Fields:**
- `id` - Component name (node name)
- `namespace` - ROS 2 namespace where the component is running
- `fqn` - Fully qualified name (namespace + node name)
- `type` - Always "Component"
- `area` - Parent area this component belongs to

## Quick Start

### Build

```bash
# From your ROS 2 workspace
colcon build --packages-select ros2_medkit_gateway

# Source the workspace
source install/setup.bash
```

### Run

**Start the gateway:**
```bash
ros2 run ros2_medkit_gateway gateway_node
```

**Or use the launch file:**
```bash
ros2 launch ros2_medkit_gateway gateway.launch.py
```

**Start demo nodes:**
```bash
ros2 launch ros2_medkit_gateway demo_nodes.launch.py
```

**Test the API:**
```bash
curl http://localhost:8080/areas
```

## Configuration

The gateway can be configured via parameters in `config/gateway_params.yaml` or through command-line arguments.

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `server.host` | string | `127.0.0.1` | Host to bind the REST server (`127.0.0.1` for localhost, `0.0.0.0` for all interfaces) |
| `server.port` | int | `8080` | Port for the REST API (range: 1024-65535) |
| `refresh_interval_ms` | int | `2000` | Cache refresh interval in milliseconds (range: 100-60000) |

### Configuration Examples

**Change port via command line:**
```bash
ros2 run ros2_medkit_gateway gateway_node --ros-args -p server.port:=9090
```

**Change host via launch file:**
```bash
ros2 launch ros2_medkit_gateway gateway.launch.py server_host:=0.0.0.0
```

## Architecture

### Components

- **Gateway Node**: Main ROS 2 node that runs the REST server
- **Discovery Manager**: Discovers and caches ROS 2 nodes, organizing them into areas and components
- **REST Server**: HTTP server using cpp-httplib
- **Entity Cache**: In-memory cache of discovered areas and components, updated periodically

### Area Organization

The gateway organizes nodes into "areas" based on their namespace:

```
/powertrain/engine/temp_sensor  → Area: powertrain, Component: temp_sensor
/chassis/brakes/pressure_sensor → Area: chassis, Component: pressure_sensor
/body/lights/controller         → Area: body, Component: controller
/standalone_node                → Area: root, Component: standalone_node
```

## Demo Nodes

The package includes demo automotive nodes for testing:

| Node | Namespace | Description |
|------|-----------|-------------|
| `temp_sensor` | `/powertrain/engine` | Engine temperature sensor |
| `rpm_sensor` | `/powertrain/engine` | Engine RPM sensor |
| `pressure_sensor` | `/chassis/brakes` | Brake pressure sensor |
| `status_sensor` | `/body/door/front_left` | Door status sensor |
| `actuator` | `/chassis/brakes` | Brake actuator |
| `controller` | `/body/lights` | Light controller |
| `calibration` | `/powertrain/engine` | Calibration service |

**Launch all demo nodes:**
```bash
ros2 launch ros2_medkit_gateway demo_nodes.launch.py
```

## URL Encoding

Component and topic names in URLs use double underscore (`__`) as a namespace separator:

| ROS 2 Name | URL Encoding |
|------------|--------------|
| `/powertrain/engine` | `powertrain__engine` |
| `/chassis/brakes` | `chassis__brakes` |
| `/body/door/front_left` | `body__door__front_left` |

**Example:**
- Component FQN: `/powertrain/engine/temp_sensor`
- URL component ID: `powertrain__engine__temp_sensor`

## Manual API Testing with Postman

We provide a Postman collection for easy API testing:

1. **Import collection:** `postman/collections/ros2-medkit-gateway.postman_collection.json`
2. **Import environment:** `postman/environments/local.postman_environment.json`
3. **Activate environment:** Select "ROS 2 Medkit Gateway - Local" in Postman
4. **Start gateway:** `ros2 launch ros2_medkit_gateway gateway.launch.py`
5. **Start demo nodes:** `ros2 launch ros2_medkit_gateway demo_nodes.launch.py`
6. **Test:** Send requests from Postman!

See [postman/README.md](postman/README.md) for detailed instructions.

## Testing

### Run Integration Tests

```bash
# Build with tests
colcon build --packages-select ros2_medkit_gateway

# Run tests
colcon test --packages-select ros2_medkit_gateway --event-handlers console_direct+

# View test results
colcon test-result --verbose
```

## License
Apache-2.0
