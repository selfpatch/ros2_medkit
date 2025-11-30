# ROS 2 Medkit Gateway Postman Collection

This folder contains Postman collections for testing the ROS 2 Medkit Gateway REST API.

## API Base Path

All endpoints are prefixed with `/api/v1` for API versioning.

## Current Collection:

**Collection:** `ros2-medkit-gateway.postman_collection.json`

Includes below endpoints:
- ✅ GET `/api/v1/` - Gateway info
- ✅ GET `/api/v1/areas` - List all areas
- ✅ GET `/api/v1/components` - List all components
- ✅ GET `/api/v1/areas/{area_id}/components` - List components in specific area
- ✅ GET `/api/v1/components/{component_id}/data` - Read all topic data from a component

## Quick Start

### 1. Import Collection

**In Postman Desktop:**
1. Click **Import** button (top-left)
2. Select: `postman/collections/ros2-medkit-gateway.postman_collection.json`
3. Click **Import**

### 2. Import Environment

1. Click **Environments** icon (left sidebar)
2. Click **Import**
3. Select: `postman/environments/local.postman_environment.json`
4. Activate environment: Select **"ROS 2 Medkit Gateway - Local"** from dropdown (top-right)

### 3. Start Gateway & Demo Nodes

**Terminal 1 - Gateway:**
```bash
ros2 launch ros2_medkit_gateway gateway.launch.py
```

**Terminal 2 - Demo Nodes:**
```bash
ros2 launch ros2_medkit_gateway demo_nodes.launch.py
```

### 4. Test

1. In Postman, expand **"Discovery"** folder
2. Click **"GET Gateway Info"**
3. Click **Send**
4. You should see: `{"status": "ROS 2 Medkit Gateway running", "version": "0.1.0", ...}`

5. Click **"GET List Areas"**
6. Click **Send**
7. You should see areas: `[{"id": "powertrain", ...}, {"id": "chassis", ...}, ...]`

8. Click **"GET List Components"**
9. Click **Send**
10. You should see components: `[{"id": "temp_sensor", "namespace": "/powertrain/engine", ...}, ...]`

11. Click **"GET Area Components"**
12. Click **Send**
13. You should see only powertrain components: `[{"id": "temp_sensor", "area": "powertrain", ...}, ...]`

14. Expand **"Component Data"** folder
15. Click **"GET Component Data"**
16. Click **Send**
17. You should see topic data: `[{"topic": "/powertrain/engine/temperature", "timestamp": ..., "data": {...}}, ...]`

## API Variables

The environment includes:
- `base_url`: `http://localhost:8080` (default gateway address)

You can change the port if your gateway runs on a different port.

---

**Happy Testing! 🚀**
