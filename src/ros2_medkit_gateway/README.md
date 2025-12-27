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

All endpoints are prefixed with `/api/v1` for API versioning.

### Discovery Endpoints

- `GET /api/v1/health` - Health check endpoint (returns healthy status)
- `GET /api/v1/` - Gateway status and version information
- `GET /api/v1/areas` - List all discovered areas (powertrain, chassis, body, root)
- `GET /api/v1/components` - List all discovered components across all areas
- `GET /api/v1/areas/{area_id}/components` - List components within a specific area

### Component Data Endpoints

- `GET /api/v1/components/{component_id}/data` - Read all topic data from a component
- `GET /api/v1/components/{component_id}/data/{topic_name}` - Read specific topic data from a component
- `PUT /api/v1/components/{component_id}/data/{topic_name}` - Publish data to a topic

### Operations Endpoints (Services & Actions)

- `GET /api/v1/components/{component_id}/operations` - List all services and actions for a component
- `POST /api/v1/components/{component_id}/operations/{operation}` - Call service or send action goal
- `GET /api/v1/components/{component_id}/operations/{operation}/status` - Get action goal status
- `GET /api/v1/components/{component_id}/operations/{operation}/result` - Get action goal result
- `DELETE /api/v1/components/{component_id}/operations/{operation}` - Cancel action goal

### Configurations Endpoints (ROS 2 Parameters)

- `GET /api/v1/components/{component_id}/configurations` - List all parameters for a component
- `GET /api/v1/components/{component_id}/configurations/{param}` - Get parameter value
- `PUT /api/v1/components/{component_id}/configurations/{param}` - Set parameter value
- `DELETE /api/v1/components/{component_id}/configurations/{param}` - Reset parameter to default value
- `DELETE /api/v1/components/{component_id}/configurations` - Reset all parameters to default values

### API Reference

#### GET /api/v1/areas

Lists all discovered areas in the system.

**Example:**
```bash
curl http://localhost:8080/api/v1/areas
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

#### GET /api/v1/components

Lists all discovered components across all areas.

**Example:**
```bash
curl http://localhost:8080/api/v1/components
```

**Response:**
```json
[
  {
    "id": "temp_sensor",
    "namespace": "/powertrain/engine",
    "fqn": "/powertrain/engine/temp_sensor",
    "type": "Component",
    "area": "powertrain",
    "source": "node"
  },
  {
    "id": "carter1",
    "namespace": "/carter1",
    "fqn": "/carter1",
    "type": "Component",
    "area": "carter1",
    "source": "topic"
  }
]
```

**Response Fields:**
- `id` - Component name (node name or namespace for topic-based)
- `namespace` - ROS 2 namespace where the component is running
- `fqn` - Fully qualified name (namespace + node name)
- `type` - Always "Component"
- `source` - Discovery source: `"node"` (standard ROS 2 node) or `"topic"` (discovered from topic namespaces)
- `area` - Parent area this component belongs to

#### GET /api/v1/areas/{area_id}/components

Lists all components within a specific area.

**Example (Success):**
```bash
curl http://localhost:8080/api/v1/areas/powertrain/components
```

**Response (200 OK):**
```json
[
  {
    "id": "temp_sensor",
    "namespace": "/powertrain/engine",
    "fqn": "/powertrain/engine/temp_sensor",
    "type": "Component",
    "area": "powertrain",
    "source": "node"
  },
  {
    "id": "rpm_sensor",
    "namespace": "/powertrain/engine",
    "fqn": "/powertrain/engine/rpm_sensor",
    "type": "Component",
    "area": "powertrain",
    "source": "node"
  }
]
```

**Example (Error - Area Not Found):**
```bash
curl http://localhost:8080/api/v1/areas/nonexistent/components
```

**Response (404 Not Found):**
```json
{
  "error": "Area not found",
  "area_id": "nonexistent"
}
```

**URL Parameters:**
- `area_id` - Area identifier (e.g., `powertrain`, `chassis`, `body`)

**Use Cases:**
- Filter components by domain (only show powertrain components)
- Hierarchical navigation (select area → view its components)
- Area-specific health checks

### Component Data Read Endpoints

#### GET /api/v1/components/{component_id}/data

Read all topic data from a specific component.

**Example:**
```bash
curl http://localhost:8080/api/v1/components/temp_sensor/data
```

**Response (200 OK):**
```json
[
  {
    "topic": "/powertrain/engine/temperature",
    "timestamp": 1732377600000000000,
    "data": {
      "temperature": 85.5,
      "variance": 0.0
    }
  }
]
```

**Example (Error - Component Not Found):**
```bash
curl http://localhost:8080/api/v1/components/nonexistent/data
```

**Response (404 Not Found):**
```json
{
  "error": "Component not found",
  "component_id": "nonexistent"
}
```

**URL Parameters:**
- `component_id` - Component identifier (e.g., `temp_sensor`, `rpm_sensor`)

**Response Fields:**
- `topic` - Full topic path
- `timestamp` - Unix timestamp (nanoseconds since epoch) when data was sampled
- `data` - Topic message data as JSON object

**Behavior:**
- Returns array of all topics under the component's namespace
- Each topic is sampled once with `ros2 topic echo --once`
- Empty array `[]` returned if component has no topics
- 3-second timeout per topic to accommodate slow-publishing topics

**Use Cases:**
- Remote diagnostics - Read all sensor values from a component
- System monitoring - Get current state of all component topics
- Data logging - Periodic sampling of component data

**Performance Considerations:**
- Topic sampling uses **parallel execution** with configurable concurrency
- Default: up to 10 topics sampled in parallel (configurable via `max_parallel_topic_samples`)
- Response time scales with batch count: `ceil(topics / batch_size) × timeout`
- 3-second timeout per topic to accommodate slow-publishing topics

#### GET /api/v1/components/{component_id}/data/{topic_name}

Read data from a specific topic within a component.

**Example:**
```bash
curl http://localhost:8080/api/v1/components/temp_sensor/data/temperature
```

**Response (200 OK):**
```json
{
  "topic": "/powertrain/engine/temperature",
  "timestamp": 1732377600000000000,
  "data": {
    "temperature": 85.5,
    "variance": 0.0
  }
}
```

**Example (Error - Topic Not Found):**
```bash
curl http://localhost:8080/api/v1/components/temp_sensor/data/nonexistent
```

**Response (404 Not Found):**
```json
{
  "error": "Topic not found or not publishing",
  "component_id": "temp_sensor",
  "topic_name": "nonexistent"
}
```

**Example (Error - Component Not Found):**
```bash
curl http://localhost:8080/api/v1/components/nonexistent/data/temperature
```

**Response (404 Not Found):**
```json
{
  "error": "Component not found",
  "component_id": "nonexistent"
}
```

**Example (Error - Invalid Topic Name):**
```bash
curl http://localhost:8080/api/v1/components/temp_sensor/data/invalid-name
```

**Response (400 Bad Request):**
```json
{
  "error": "Invalid topic name",
  "details": "Entity ID contains invalid character: '-'. Only alphanumeric and underscore are allowed",
  "topic_name": "invalid-name"
}
```

**URL Parameters:**
- `component_id` - Component identifier (e.g., `temp_sensor`, `rpm_sensor`)
- `topic_name` - Topic name within the component (e.g., `temperature`, `rpm`)

**Response Fields:**
- `topic` - Full topic path (e.g., `/powertrain/engine/temperature`)
- `timestamp` - Unix timestamp (nanoseconds since epoch) when data was sampled
- `data` - Topic message data as JSON object

**Validation:**
- Both `component_id` and `topic_name` follow ROS 2 naming conventions
- Allowed characters: alphanumeric (a-z, A-Z, 0-9), underscore (_)
- Hyphens, special characters, and escape sequences are rejected

**Use Cases:**
- Read specific sensor value (e.g., just temperature, not all engine data)
- Lower latency than reading all component data
- Targeted monitoring of specific metrics

### Operations Endpoints

#### GET /api/v1/components/{component_id}/operations

List all operations (services and actions) available for a component.

**Example:**
```bash
curl http://localhost:8080/api/v1/components/calibration/operations
```

**Response (200 OK):**
```json
[
  {
    "name": "calibrate",
    "path": "/powertrain/engine/calibrate",
    "type": "std_srvs/srv/Trigger",
    "kind": "service",
    "type_info": {
      "schema": "...",
      "default_value": "..."
    }
  }
]
```

#### POST /api/v1/components/{component_id}/operations/{operation}

Call a service or send an action goal.

**Example (Service Call):**
```bash
curl -X POST http://localhost:8080/api/v1/components/calibration/operations/calibrate \
  -H "Content-Type: application/json" \
  -d '{}'
```

**Response (200 OK - Service):**
```json
{
  "status": "success",
  "kind": "service",
  "component_id": "calibration",
  "operation": "calibrate",
  "response": {
    "success": true,
    "message": "Calibration triggered"
  }
}
```

**Example (Action Goal):**
```bash
curl -X POST http://localhost:8080/api/v1/components/long_calibration/operations/long_calibration \
  -H "Content-Type: application/json" \
  -d '{"goal": {"order": 10}}'
```

**Response (202 Accepted - Action):**
```json
{
  "status": "accepted",
  "kind": "action",
  "component_id": "long_calibration",
  "operation": "long_calibration",
  "goal_id": "abc123def456...",
  "goal_status": "executing"
}
```

#### GET /api/v1/components/{component_id}/operations/{operation}/status

Get the status of an action goal.

**Example (Latest Goal):**
```bash
curl http://localhost:8080/api/v1/components/long_calibration/operations/long_calibration/status
```

**Example (Specific Goal):**
```bash
curl "http://localhost:8080/api/v1/components/long_calibration/operations/long_calibration/status?goal_id=abc123"
```

**Example (All Goals):**
```bash
curl "http://localhost:8080/api/v1/components/long_calibration/operations/long_calibration/status?all=true"
```

**Response (200 OK):**
```json
{
  "goal_id": "abc123def456...",
  "status": "succeeded",
  "action_path": "/long_calibration/long_calibration",
  "action_type": "example_interfaces/action/Fibonacci"
}
```

#### DELETE /api/v1/components/{component_id}/operations/{operation}

Cancel a running action goal.

**Example:**
```bash
curl -X DELETE "http://localhost:8080/api/v1/components/long_calibration/operations/long_calibration?goal_id=abc123"
```

**Response (200 OK):**
```json
{
  "status": "canceling",
  "goal_id": "abc123def456...",
  "message": "Cancel request sent"
}
```

### Configurations Endpoints

#### GET /api/v1/components/{component_id}/configurations

List all parameters for a component.

**Example:**
```bash
curl http://localhost:8080/api/v1/components/temp_sensor/configurations
```

**Response (200 OK):**
```json
{
  "component_id": "temp_sensor",
  "node_name": "/powertrain/engine/temp_sensor",
  "parameters": [
    {
      "name": "publish_rate",
      "value": 2.0,
      "type": "double"
    },
    {
      "name": "min_temp",
      "value": -40.0,
      "type": "double"
    }
  ]
}
```

#### GET /api/v1/components/{component_id}/configurations/{param}

Get a specific parameter value.

**Example:**
```bash
curl http://localhost:8080/api/v1/components/temp_sensor/configurations/publish_rate
```

**Response (200 OK):**
```json
{
  "component_id": "temp_sensor",
  "parameter": {
    "name": "publish_rate",
    "value": 2.0,
    "type": "double"
  }
}
```

#### PUT /api/v1/components/{component_id}/configurations/{param}

Set a parameter value.

**Example:**
```bash
curl -X PUT http://localhost:8080/api/v1/components/temp_sensor/configurations/publish_rate \
  -H "Content-Type: application/json" \
  -d '{"value": 5.0}'
```

**Response (200 OK):**
```json
{
  "status": "success",
  "component_id": "temp_sensor",
  "parameter": {
    "name": "publish_rate",
    "value": 5.0,
    "type": "double"
  }
}
```

#### DELETE /api/v1/components/{component_id}/configurations/{param}

Reset a parameter to its default (initial) value.

**Example:**
```bash
curl -X DELETE http://localhost:8080/api/v1/components/temp_sensor/configurations/min_temp
```

**Response (200 OK):**
```json
{
  "name": "min_temp",
  "value": -40.0,
  "type": "double",
  "reset_to_default": true
}
```

#### DELETE /api/v1/components/{component_id}/configurations

Reset all parameters to their default (initial) values.

**Example:**
```bash
curl -X DELETE http://localhost:8080/api/v1/components/temp_sensor/configurations
```

**Response (200 OK):**
```json
{
  "node_name": "/powertrain/engine/temp_sensor",
  "reset_count": 5,
  "failed_count": 0
}
```

**Response (207 Multi-Status - Partial Success):**
```json
{
  "node_name": "/powertrain/engine/temp_sensor",
  "reset_count": 3,
  "failed_count": 2,
  "failed_parameters": ["read_only_param1", "read_only_param2"]
}
```

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
curl http://localhost:8080/api/v1/areas
```

## Configuration

The gateway can be configured via parameters in `config/gateway_params.yaml` or through command-line arguments.

### Parameters

#### Server Configuration

| Parameter                    | Type   | Default     | Description                                                                            |
| ---------------------------- | ------ | ----------- | -------------------------------------------------------------------------------------- |
| `server.host`                | string | `127.0.0.1` | Host to bind the REST server (`127.0.0.1` for localhost, `0.0.0.0` for all interfaces) |
| `server.port`                | int    | `8080`      | Port for the REST API (range: 1024-65535)                                              |
| `refresh_interval_ms`        | int    | `2000`      | Cache refresh interval in milliseconds (range: 100-60000)                              |
| `max_parallel_topic_samples` | int    | `10`        | Max concurrent topic samples when fetching data (range: 1-50)                          |

#### Native Sampling

The gateway always uses native rclcpp APIs for topic discovery and sampling. This provides significant UX improvements:

- **Instant metadata for idle topics**: Topics without publishers return immediately with metadata (type, schema) instead of waiting for a 3-second timeout
- **Faster discovery**: Uses `node->get_topic_names_and_types()` instead of `ros2 topic list`
- **Reduced external dependencies**: Native APIs for discovery and idle topics. GNU `timeout` is still required for sampling topics with active publishers (install `coreutils` in containers if needed)
- **Better responsiveness**: Robots waiting for commands (many idle topics) respond instantly

CLI is only used for publishing (`ros2 topic pub`), as native publishing requires compile-time type knowledge.

#### CORS Configuration

Cross-Origin Resource Sharing (CORS) settings for browser-based clients. CORS is **enabled automatically** when `allowed_origins` is not empty.

| Parameter                | Type     | Default                      | Description                                                                         |
| ------------------------ | -------- | ---------------------------- | ----------------------------------------------------------------------------------- |
| `cors.allowed_origins`   | string[] | `[]`                         | List of allowed origins (e.g., `["http://localhost:5173"]`). Empty = CORS disabled. |
| `cors.allowed_methods`   | string[] | `["GET", "PUT", "OPTIONS"]`  | HTTP methods allowed for CORS requests                                              |
| `cors.allowed_headers`   | string[] | `["Content-Type", "Accept"]` | Headers allowed in CORS requests                                                    |
| `cors.allow_credentials` | bool     | `false`                      | Allow credentials (cookies, auth headers). Cannot be `true` with wildcard origin.   |
| `cors.max_age_seconds`   | int      | `86400`                      | How long browsers cache preflight response (24 hours default)                       |

### Configuration Examples

**Change port via command line:**
```bash
ros2 run ros2_medkit_gateway gateway_node --ros-args -p server.port:=9090
```

**Change host via launch file:**
```bash
ros2 launch ros2_medkit_gateway gateway.launch.py server_host:=0.0.0.0
```

**Enable CORS for local development:**
```yaml
cors:
  allowed_origins: ["http://localhost:5173", "http://localhost:3000"]
  allowed_methods: ["GET", "PUT", "OPTIONS"]
  allowed_headers: ["Content-Type", "Accept"]
  allow_credentials: false
  max_age_seconds: 86400
```

**Enable CORS for production (specific domain):**
```yaml
cors:
  allowed_origins: ["https://dashboard.example.com"]
  allowed_methods: ["GET", "OPTIONS"]
  allowed_headers: ["Content-Type", "Accept", "Authorization"]
  allow_credentials: true
  max_age_seconds: 86400
```

> ⚠️ **Security Note:** Using `["*"]` as `allowed_origins` is not recommended for production. When `allow_credentials` is `true`, wildcard origins will cause the application to fail to start with an exception.

## Architecture

### Components

- **Gateway Node**: Main ROS 2 node that runs the REST server
- **Discovery Manager**: Discovers and caches ROS 2 nodes, organizing them into areas and components
- **REST Server**: HTTP server using cpp-httplib
- **Entity Cache**: In-memory cache of discovered areas and components, updated periodically

### Topic-Based Discovery

In addition to standard ROS 2 node discovery, the gateway supports **topic-based discovery** for systems that publish topics without creating discoverable nodes (e.g., NVIDIA Isaac Sim, hardware bridges).

**How it works:**
1. Gateway scans all topics in the ROS 2 graph
2. Extracts unique namespace prefixes (e.g., `/carter1/odom` → `carter1`)
3. Creates virtual "components" for namespaces that have topics but no nodes
4. These components have `"source": "topic"` to distinguish them from node-based components

**Example:** Isaac Sim publishes topics like `/carter1/odom`, `/carter1/cmd_vel`, `/carter2/imu` without creating ROS 2 nodes. The gateway discovers:
- Component `carter1` with topics: `/carter1/odom`, `/carter1/cmd_vel`
- Component `carter2` with topics: `/carter2/imu`

**System topic filtering:** The following topics are filtered out during discovery:
- `/parameter_events`, `/rosout`, `/clock`
- Note: `/tf` and `/tf_static` are NOT filtered (useful for diagnostics)

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

| Node              | Namespace               | Description               |
| ----------------- | ----------------------- | ------------------------- |
| `temp_sensor`     | `/powertrain/engine`    | Engine temperature sensor |
| `rpm_sensor`      | `/powertrain/engine`    | Engine RPM sensor         |
| `pressure_sensor` | `/chassis/brakes`       | Brake pressure sensor     |
| `status_sensor`   | `/body/door/front_left` | Door status sensor        |
| `actuator`        | `/chassis/brakes`       | Brake actuator            |
| `controller`      | `/body/lights`          | Light controller          |
| `calibration`     | `/powertrain/engine`    | Calibration service       |

**Launch all demo nodes:**
```bash
ros2 launch ros2_medkit_gateway demo_nodes.launch.py
```

## URL Encoding

Topic names in URLs use standard percent-encoding (`%2F`) for the forward slash character:

| ROS 2 Topic Path                | URL Encoding                                |
| ------------------------------- | ------------------------------------------- |
| `/powertrain/engine/temperature`| `powertrain%2Fengine%2Ftemperature`         |
| `/chassis/brakes/command`       | `chassis%2Fbrakes%2Fcommand`                |
| `/body/door/front_left/status`  | `body%2Fdoor%2Ffront_left%2Fstatus`         |

**Example:**
- Topic: `/powertrain/engine/temperature`
- URL: `/api/v1/components/temp_sensor/data/powertrain%2Fengine%2Ftemperature`

> **Note:** Component IDs do not require encoding (they are simple names like `temp_sensor`).
> The leading slash is stripped before encoding.

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
