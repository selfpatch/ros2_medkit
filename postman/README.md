# ROS 2 Medkit Gateway Postman Collection

This folder contains Postman collections for testing the ROS 2 Medkit Gateway REST API.

## Current Collection:

**Collection:** `ros2-medkit-gateway.postman_collection.json`

Includes below endpoints:
- ✅ GET `/` - Gateway info
- ✅ GET `/areas` - List all areas
- ✅ GET `/components` - List all components

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

## API Variables

The environment includes:
- `base_url`: `http://localhost:8080` (default gateway address)

You can change the port if your gateway runs on a different port.

---

**Happy Testing! 🚀**
