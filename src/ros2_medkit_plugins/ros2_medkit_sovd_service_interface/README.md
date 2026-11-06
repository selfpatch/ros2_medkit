# ros2_medkit_sovd_service_interface

Gateway plugin exposing medkit entity data via ROS 2 services. Enables ROS 2 nodes (e.g. VDA 5050 agent, BT.CPP, PlotJuggler) to access SOVD diagnostics without HTTP.

## Services

| Service | Type | Description |
|---------|------|-------------|
| `/medkit/list_entities` | `ros2_medkit_msgs/srv/ListEntities` | List all discovered entities (apps, components, areas) |
| `/medkit/list_entity_faults` | `ros2_medkit_msgs/srv/ListFaultsForEntity` | Get faults for a specific entity |
| `/medkit/get_entity_data` | `ros2_medkit_msgs/srv/GetEntityData` | Get entity topic data (not yet implemented) |
| `/medkit/get_capabilities` | `ros2_medkit_msgs/srv/GetCapabilities` | Get SOVD capabilities for an entity |

Service prefix is configurable via `plugins.sovd_service_interface.service_prefix` parameter (default: `/medkit`).

The entity-scoped requests name an entity by `entity_id` alone. An id is unique within its entity type only, so an
id held by more than one entity (an app and a component sharing it, say) is refused: `success` is false and
`error_message` names the types holding it. An id one entity holds is answered for that entity.

## Usage

Load as a gateway plugin:

```bash
ros2 run ros2_medkit_gateway gateway_node --ros-args \
    -p "plugins:=[\"sovd_service_interface\"]" \
    -p "plugins.sovd_service_interface.path:=/path/to/libsovd_service_interface.so" \
    -p "plugins.sovd_service_interface.service_prefix:=/medkit"
```

## Tests

```bash
colcon test --packages-select ros2_medkit_sovd_service_interface
```
