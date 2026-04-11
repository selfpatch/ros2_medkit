# ros2_medkit_opcua

Gateway plugin that bridges OPC-UA capable PLCs (OpenPLC, Siemens S7, Beckhoff, Allen-Bradley, etc.) into the SOVD entity tree. Enables unified diagnostics for mixed ROS 2 + industrial PLC deployments through a single REST API, with PLC alarms routed to `ros2_medkit_fault_manager` and numeric PLC values optionally bridged to ROS 2 `std_msgs/Float32` topics.

Follows the same plugin pattern as `ros2_medkit_graph_provider`: implements `GatewayPlugin` + `IntrospectionProvider` against the `get_routes()` plugin API, loaded at runtime by `ros2_medkit_gateway` via `dlopen`.

## What it does

- Connects to any OPC-UA capable PLC server over `opc.tcp`
- Emits SOVD entities (area, component, apps) from a YAML-driven node map
- Exposes PLC values as the `x-plc-data` vendor collection
- Allows writing setpoints via `x-plc-operations` with type-aware coercion and range validation
- Reports the connection state and poll metrics via `x-plc-status`
- Maps threshold-based PLC alarms to SOVD faults on the owning entity
- Optionally publishes numeric PLC values to ROS 2 `std_msgs/Float32` topics

## Architecture

```
                    OPC-UA (TCP :4840)
PLC Runtime  <─────────────────────────>  OPC-UA Plugin (.so)
  IEC 61131-3 program                      │
  Cyclic execution (100ms)                 │  Polls all configured nodes
  Variables exposed as OPC-UA nodes        │  Maps to SOVD entity tree
                                           │  Alarm thresholds -> fault reporting
                                           │
                                           ▼
                                    ros2_medkit Gateway
                                      REST API :8080
                                           │
                                    ┌──────┴──────┐
                                    │             │
                              SOVD REST      Fleet Gateway
                              (direct)       (aggregation)
                                    │             │
                                Dashboard    Multi-device view
```

The plugin connects to any PLC with an OPC-UA server over TCP. No ROS 2 dependency between the plugin and the PLC - communication is pure OPC-UA. The plugin is loaded by the gateway at runtime via `dlopen()` and registers vendor REST endpoints for PLC data access and control.

## SOVD Entity Model

The plugin creates a hierarchical entity tree from a YAML node map configuration:

```
Area: plc_systems
  └── Component: openplc_runtime
        ├── App: tank_process
        │     Data: tank_level (mm), tank_temperature (C), tank_pressure (bar)
        │     Faults: PLC_HIGH_TEMP, PLC_LOW_LEVEL, PLC_OVERPRESSURE
        ├── App: fill_pump
        │     Data: pump_speed (%)
        │     Operations: set_pump_speed
        └── App: drain_valve
              Data: valve_position (%)
              Operations: set_valve_position
```

## REST API

### Vendor Endpoints

| Method | Path | Description |
|--------|------|-------------|
| GET | `/apps/{id}/x-plc-data` | All OPC-UA values for entity (with units, types, timestamps) |
| GET | `/apps/{id}/x-plc-data/{name}` | Single data point value |
| POST | `/apps/{id}/x-plc-operations/set_{name}` | Write value to PLC (`{"value": 75.0}`) |
| GET | `/components/{id}/x-plc-status` | Connection state, poll stats, active alarms |

### Standard SOVD (provided by gateway)

| Method | Path | Description |
|--------|------|-------------|
| GET | `/api/v1/areas` | Lists `plc_systems` area |
| GET | `/api/v1/components` | Lists `openplc_runtime` component |
| GET | `/api/v1/apps` | Lists PLC applications (tank_process, fill_pump, etc.) |
| GET | `/api/v1/apps/{id}/faults` | Active PLC alarms mapped to SOVD faults |

### Example Responses

**Live PLC data:**
```json
GET /api/v1/apps/tank_process/x-plc-data

{
  "entity_id": "tank_process",
  "connected": true,
  "timestamp": 1774185903,
  "items": [
    {"name": "tank_level", "value": 742.5, "unit": "mm", "data_type": "float", "writable": true},
    {"name": "tank_temperature", "value": 31.8, "unit": "C", "data_type": "float", "writable": true},
    {"name": "tank_pressure", "value": 2.95, "unit": "bar", "data_type": "float", "writable": false}
  ]
}
```

**Write to PLC:**
```json
POST /api/v1/apps/fill_pump/x-plc-operations/set_pump_speed
{"value": 80.0}

{"status": "ok", "operation": "set_pump_speed", "node_id": "ns=2;i=4", "value_written": 80}
```

**PLC connection status:**
```json
GET /api/v1/components/openplc_runtime/x-plc-status

{
  "component_id": "openplc_runtime",
  "connected": true,
  "endpoint_url": "opc.tcp://openplc:4840/openplc/opcua",
  "server_description": "OpenPLC Runtime",
  "mode": "poll",
  "poll_count": 142,
  "error_count": 0,
  "node_count": 5,
  "active_alarms": []
}
```

## Finding Node IDs on your PLC

The plugin identifies PLC tags by OPC-UA node IDs in the canonical string
format (`ns=N;i=M` numeric, `ns=N;s=tag` string, `ns=N;g=...` GUID, or
`ns=N;b=...` opaque). To discover the correct node IDs for a real PLC
without guessing, use one of the standard OPC-UA browser tools:

- **UaExpert** - free GUI browser from Unified Automation. Download at
  <https://www.unified-automation.com/downloads/uaexpert.html>. Connect
  to your PLC's `opc.tcp://` endpoint, navigate the address space tree,
  right-click any Variable node, copy the NodeId property into the YAML
  map below.

- **`asyncua` command line** - `pip install asyncua` and then
  `python -m asyncua.tools.uals -u opc.tcp://your-plc:4840` walks the
  address space from a terminal, no GUI required.

- **Vendor toolchains** - Siemens TIA Portal's OPC-UA configuration
  exports DB/variable node IDs in the `ns=3;s="..."` format. Beckhoff
  TwinCAT 3 XAE displays them as `ns=4;s=MAIN.Tank.level`. Allen-Bradley
  users typically deploy Kepware or Ignition as an OPC-UA gateway which
  auto-maps tag names.

`config/tank_demo_nodes.yaml` ships a commented example with
ready-to-paste templates for OpenPLC, Siemens S7-1500, Beckhoff TwinCAT,
Allen-Bradley via Kepware and KUKA KR C5. Copy one of those blocks as a
starting point.

## Configuration

### Node Map (YAML)

Maps OPC-UA NodeIds to SOVD entities. One file per PLC setup.

```yaml
area_id: plc_systems
area_name: PLC Systems
component_id: openplc_runtime
component_name: OpenPLC Runtime

nodes:
  - node_id: "ns=2;i=1"          # OPC-UA NodeId (numeric or string)
    entity_id: tank_process       # SOVD app this belongs to
    data_name: tank_level         # Data point name in REST API
    display_name: Tank Level
    unit: mm
    data_type: float
    writable: true                # Allow writes via x-plc-operations
    min_value: 0.0                # Optional: range validation for writes
    max_value: 100.0
    alarm:                        # Optional: map to SOVD fault
      fault_code: PLC_LOW_LEVEL
      severity: WARNING
      message: Tank level below minimum
      threshold: 100.0
      above_threshold: false      # Alarm when value < threshold
```

### Gateway Parameters

```yaml
ros2_medkit_gateway:
  ros__parameters:
    plugins: ["opcua"]
    plugins.opcua.endpoint_url: "opc.tcp://plc-host:4840/path"
    plugins.opcua.node_map_path: "/path/to/nodes.yaml"
    plugins.opcua.poll_interval_ms: 1000
    plugins.opcua.prefer_subscriptions: false
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `endpoint_url` | `opc.tcp://localhost:4840` | OPC-UA server endpoint |
| `node_map_path` | (none) | Path to node map YAML (required) |
| `poll_interval_ms` | `1000` | Polling interval in milliseconds |
| `prefer_subscriptions` | `false` | Use OPC-UA subscriptions instead of polling |

### Operation Naming Convention

Write operations use the `set_` prefix convention:
- Node map defines `data_name: pump_speed`
- REST operation: `POST /apps/fill_pump/x-plc-operations/set_pump_speed`
- The `set_` prefix is stripped to find the matching node map entry
- Operations without `set_` prefix also work (matches `data_name` directly)

### Environment Variables (override YAML config)

| Variable | Description |
|----------|-------------|
| `OPCUA_ENDPOINT_URL` | OPC-UA server URL |
| `OPCUA_NODE_MAP_PATH` | Path to node map YAML |

## Hardware Deployment

### Robot + PLC on the same LAN

```
┌──────────────┐  WiFi/ETH  ┌──────────────┐  ETH  ┌──────────────┐
│  Robot       │◄───────────►│  Edge RPi    │◄─────►│  PLC         │
│  (Jetson)   │             │              │       │  (Siemens/   │
│              │             │  Fleet GW    │       │   Beckhoff/  │
│  ros2_medkit │             │  :9090       │       │   OpenPLC)   │
│  :8080       │             │              │       │              │
└──────────────┘             │  ros2_medkit │       │  OPC-UA      │
                             │  + OPC-UA    │       │  :4840       │
                             │  plugin      │       └──────────────┘
                             │  :8080       │
                             └──────────────┘
```

Fleet gateway aggregates robot and PLC diagnostics. Operator sees both in one dashboard.

### Robot directly connected to PLC

```
┌──────────────────────┐  ETH  ┌──────────────┐
│  Robot               │◄─────►│  PLC         │
│                      │       │  OPC-UA :4840│
│  ros2_medkit gateway │       └──────────────┘
│  + OPC-UA plugin     │
│  :8080               │
│                      │
│  ROS 2 nodes + PLC   │
│  in one entity tree  │
└──────────────────────┘
```

Plugin runs on the robot itself. ROS 2 faults and PLC alarms appear side by side.

### Compatible PLCs

Any PLC with an OPC-UA server works out of the box:

| PLC | OPC-UA Support | Notes |
|-----|---------------|-------|
| Siemens S7-1500 | Built-in | Most common in EU industry |
| Allen-Bradley CompactLogix | Built-in | Common in US |
| Beckhoff TwinCAT | Built-in | Popular in motion control |
| Schneider M340/M580 | Built-in | Process automation |
| OpenPLC v4 | Plugin (asyncua) | Open-source, used in our demo |
| Older PLCs (S7-300, etc.) | Via Modbus->OPC-UA bridge | Requires additional software |

### What you need

1. PLC with OPC-UA server enabled (most modern PLCs have this)
2. Network connectivity between robot/edge device and PLC (Ethernet, same subnet)
3. Node map YAML matching your PLC program variables
4. See Security Limitations below regarding OPC-UA auth

### What you DON'T need

- No ROS 2 on the PLC
- No modification to the PLC program
- No special hardware adapters
- No proprietary PLC software licenses

## Development

### Build

```bash
# From ros2_medkit repo root
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ros2_medkit_opcua
colcon test --packages-select ros2_medkit_opcua
```

### Docker Integration Tests

The plugin ships a self-contained OpenPLC tank demo in `docker/` that exercises the full stack end-to-end. CI runs this suite on every PR that touches the plugin; it is also runnable locally from any developer laptop.

```bash
cd src/ros2_medkit_plugins/ros2_medkit_opcua/docker

# Start OpenPLC + gateway (builds everything)
bash scripts/start.sh

# Manual testing
curl -s http://localhost:8080/api/v1/apps/tank_process/x-plc-data | jq .

# Automated tests (16 assertions)
bash scripts/run_integration_tests.sh

# Stop
bash scripts/stop.sh
```

### Test Coverage

| Category | Tests | What it validates |
|----------|-------|-------------------|
| Entity discovery | 5 | Areas, components, apps from PLC node map |
| PLC connection | 2 | OPC-UA connected, zero errors |
| Live data | 3 | Tank level, temperature, pressure have values |
| Write control | 2 | Pump speed, valve position written to PLC |
| Error handling | 3 | Unknown entity, unknown operation, invalid JSON |
| **Total** | **16** | |

## Security Limitations

**Current version uses Anonymous auth with SecurityPolicy=None.** All OPC-UA communication is unencrypted and unauthenticated. This is acceptable for isolated LANs and demo environments but NOT suitable for production networks exposed to untrusted traffic.

Planned for future versions:
- OPC-UA certificate-based authentication (Basic256Sha256)
- Username/password authentication
- Configurable security policy per connection

The plugin logs a startup message indicating the auth mode in use.

Write operations include configurable `min_value`/`max_value` range validation to prevent out-of-range values being sent to PLC actuators.

## Alarm-to-Fault Bridge

PLC alarms (threshold-based) are automatically mapped to SOVD faults:

```
PLC variable (e.g., TankTemperature = 95C)
    │
    ▼  threshold check (> 80C)
Alarm active: PLC_HIGH_TEMP
    │
    ▼  ROS 2 service call
ros2_medkit fault_manager
    │
    ▼  appears in SOVD API
GET /api/v1/apps/tank_process/faults
  -> [{fault_code: "PLC_HIGH_TEMP", severity: "ERROR", ...}]
```

When the value returns below threshold, the fault is automatically cleared.

## Key Design Decisions

- **Poll mode by default** - OPC-UA subscriptions require a client event loop thread. Polling at 1s interval is simpler and sufficient for diagnostics use cases.
- **Type-aware writes** - Plugin reads the OPC-UA node's data type before writing to avoid type mismatches (e.g., writing float32 to a REAL node, not float64).
- **Node map driven** - All entity mapping is in YAML config, not code. Same plugin binary works with any PLC by changing the config file.
- **Env var overrides** - `OPCUA_ENDPOINT_URL` and `OPCUA_NODE_MAP_PATH` override YAML config for Docker deployment flexibility.

## License

Apache License 2.0. See the `LICENSE` file at the repository root for the full text.

Copyright 2026 mfaferek93

## Third-party Dependencies

| Library | License | Notes |
|---------|---------|-------|
| [open62541pp](https://github.com/open62541pp/open62541pp) v0.16.0 | MPLv2 | OPC-UA C++ client, linked as library (no source modification) |
| [nlohmann/json](https://github.com/nlohmann/json) | MIT | JSON serialization |
| [yaml-cpp](https://github.com/jbeder/yaml-cpp) | MIT | Node map YAML parser |
| [OpenSSL](https://www.openssl.org/) | Apache-2.0 | TLS support for OPC-UA secure channels |

MPLv2 is a weak copyleft license that permits linking from Apache-2.0 code without triggering source disclosure obligations for the linking application, as long as the MPLv2-licensed library itself is not modified.

## Contributing

Issues and pull requests welcome through the main [ros2_medkit](https://github.com/selfpatch/ros2_medkit) repository. See the project-level `CONTRIBUTING.md` for coding style, test expectations, and review process.
