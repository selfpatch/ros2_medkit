Manifest-Based Discovery
========================

This tutorial explains how to use manifest files to define your ROS 2 system
structure for SOVD discovery.

Introduction
------------

The ros2_medkit gateway supports three discovery modes:

1. **Runtime-Only** (default): Traditional ROS 2 graph introspection
2. **Hybrid** (recommended): Manifest as source of truth + runtime linking
3. **Manifest-Only**: Only expose manifest-declared entities

Hybrid mode is recommended because it:

- Provides **stable entity IDs** that don't change across restarts
- Enables **semantic groupings** (areas, functions) for better organization
- Preserves **runtime data** (topic values, service calls)
- Allows **documentation and metadata** in entity definitions
- Supports **offline detection** for apps not currently running

Discovery Modes Comparison
--------------------------

.. list-table::
   :header-rows: 1
   :widths: 20 30 30 20

   * - Feature
     - Runtime-Only
     - Hybrid
     - Manifest-Only
   * - Entity IDs
     - Derived from ROS graph
     - Stable (from manifest)
     - Stable (from manifest)
   * - Live topics/services
     - ✅ Yes
     - ✅ Yes
     - ❌ No
   * - Custom areas
     - ❌ No
     - ✅ Yes
     - ✅ Yes
   * - Apps entity type
     - ❌ No
     - ✅ Yes
     - ✅ Yes
   * - Functions entity type
     - ❌ No
     - ✅ Yes
     - ✅ Yes
   * - Offline detection
     - ❌ No
     - ✅ Yes
     - N/A

Writing Your First Manifest
---------------------------

Create a file named ``system_manifest.yaml``:

.. code-block:: yaml

   # SOVD System Manifest
   sovd_version: "1.0"

   metadata:
     name: "my-robot"
     version: "1.0.0"
     description: "My Robot System"

   # Define logical areas (subsystems)
   areas:
     - id: perception
       name: "Perception"
       category: "sensor-processing"
       description: "Sensor data processing and fusion"

   # Define hardware/virtual components
   components:
     - id: lidar-sensor
       name: "LiDAR Sensor"
       type: "sensor"
       area: perception

   # Define apps (map to ROS 2 nodes)
   apps:
     - id: lidar-driver
       name: "LiDAR Driver"
       is_located_on: lidar-sensor
       ros_binding:
         node_name: velodyne_driver
         namespace: /sensors

   # Define high-level functions
   functions:
     - id: environment-sensing
       name: "Environment Sensing"
       hosted_by:
         - lidar-driver

Enabling Manifest Mode
----------------------

Option 1: Using Launch File
~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   from launch import LaunchDescription
   from launch_ros.actions import Node

   def generate_launch_description():
       return LaunchDescription([
           Node(
               package='ros2_medkit_gateway',
               executable='gateway_node',
               parameters=[{
                   'manifest.enabled': True,
                   'manifest.file_path': '/path/to/system_manifest.yaml',
                   'manifest.mode': 'hybrid',
               }]
           )
       ])

Option 2: Using Parameter File
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Add to your ``gateway_params.yaml``:

.. code-block:: yaml

   ros2_medkit_gateway:
     ros__parameters:
       # ... existing parameters ...

       manifest:
         # Enable manifest-based discovery
         enabled: true

         # Path to manifest YAML file
         file_path: "/path/to/system_manifest.yaml"

         # Discovery mode: "runtime_only", "hybrid", or "manifest_only"
         mode: "hybrid"

         # Strict validation: fail on any validation error
         strict_validation: true

Then launch with:

.. code-block:: bash

   ros2 run ros2_medkit_gateway gateway_node \
       --ros-args --params-file /path/to/gateway_params.yaml

Option 3: Command Line Parameters
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   ros2 run ros2_medkit_gateway gateway_node --ros-args \
       -p manifest.enabled:=true \
       -p manifest.file_path:=/path/to/system_manifest.yaml \
       -p manifest.mode:=hybrid

Verifying the Configuration
---------------------------

Once the gateway is running with a manifest, you can verify the configuration
via the REST API.

Check manifest status:

.. code-block:: bash

   curl http://localhost:8080/api/v1/manifest/status

Expected response:

.. code-block:: json

   {
     "status": "active",
     "discovery_mode": "hybrid",
     "manifest_path": "/path/to/system_manifest.yaml",
     "statistics": {
       "areas_count": 1,
       "components_count": 1,
       "apps_count": 1,
       "functions_count": 1
     }
   }

List apps:

.. code-block:: bash

   curl http://localhost:8080/api/v1/apps

List functions:

.. code-block:: bash

   curl http://localhost:8080/api/v1/functions

Understanding Runtime Linking
-----------------------------

In hybrid mode, manifest apps are automatically linked to running ROS 2 nodes.
The linking process:

1. **Discovery**: Gateway discovers running ROS 2 nodes
2. **Matching**: For each manifest app, checks ``ros_binding`` configuration
3. **Linking**: If match found, copies runtime resources (topics, services, actions)
4. **Status**: Apps with matched nodes are marked ``is_online: true``

ROS Binding Configuration
~~~~~~~~~~~~~~~~~~~~~~~~~

The ``ros_binding`` section specifies how to match an app to a ROS 2 node:

.. code-block:: yaml

   ros_binding:
     # Match by node name
     node_name: "velodyne_driver"

     # Match within a specific namespace (optional)
     namespace: "/sensors"

     # Or match by topic namespace prefix (alternative)
     topic_namespace: "/sensors/lidar"

**Match Types:**

- **Exact match** (default): Node name and namespace must match exactly
- **Wildcard namespace**: Use ``namespace: "*"`` to match any namespace
- **Topic namespace**: Match nodes by their topic prefix

.. code-block:: yaml

   # Example: Match node in any namespace
   apps:
     - id: camera-driver
       name: "Camera Driver"
       ros_binding:
         node_name: usb_cam
         namespace: "*"

   # Example: Match by topic namespace
   apps:
     - id: lidar-processing
       name: "LiDAR Processing"
       ros_binding:
         topic_namespace: "/perception/lidar"

Checking Linking Status
~~~~~~~~~~~~~~~~~~~~~~~

Check which apps are online:

.. code-block:: bash

   curl http://localhost:8080/api/v1/apps | jq '.[] | {id, name, is_online}'

Example response:

.. code-block:: json

   {"id": "lidar-driver", "name": "LiDAR Driver", "is_online": true}
   {"id": "camera-driver", "name": "Camera Driver", "is_online": false}

Entity Hierarchy
----------------

The manifest defines a hierarchical structure:

.. code-block:: text

   Areas (logical/physical groupings)
   └── Components (hardware/virtual units)
       └── Apps (software applications)
           └── Data (topics)
           └── Operations (services/actions)
           └── Configurations (parameters)

   Functions (cross-cutting capabilities)
   └── Apps (hosted by)

**Areas** group related components by function or location:

.. code-block:: yaml

   areas:
     - id: perception
       name: "Perception Subsystem"
       subareas:
         - id: lidar-processing
           name: "LiDAR Processing"

**Components** represent hardware or virtual units:

.. code-block:: yaml

   components:
     - id: main-computer
       name: "Main Computer"
       type: "controller"
       area: perception
       subcomponents:
         - id: gpu-unit
           name: "GPU Processing Unit"

**Apps** are software applications (typically ROS 2 nodes):

.. code-block:: yaml

   apps:
     - id: slam-node
       name: "SLAM Node"
       is_located_on: main-computer
       depends_on:
         - lidar-driver
         - imu-driver
       ros_binding:
         node_name: slam_toolbox

**Functions** are high-level capabilities spanning multiple apps:

.. code-block:: yaml

   functions:
     - id: autonomous-navigation
       name: "Autonomous Navigation"
       hosted_by:
         - planner-node
         - controller-node
         - localization-node

Handling Unmanifested Nodes
---------------------------

In hybrid mode, the gateway may discover ROS 2 nodes that aren't declared
in the manifest. The ``config.unmanifested_nodes`` setting controls this:

.. code-block:: yaml

   config:
     # Options: ignore, warn, error, include_as_orphan
     unmanifested_nodes: warn

**Policies:**

- ``ignore``: Don't expose unmanifested nodes at all
- ``warn`` (default): Log warning, include nodes as orphans
- ``error``: Fail startup if orphan nodes detected
- ``include_as_orphan``: Include with ``source: "orphan"``

Hot Reloading
-------------

Reload the manifest without restarting the gateway:

.. code-block:: bash

   curl -X POST http://localhost:8080/api/v1/manifest/reload

This re-parses the manifest file and re-links apps to nodes.

REST API Endpoints
------------------

Manifest mode adds the following endpoints:

.. list-table::
   :header-rows: 1
   :widths: 30 50 20

   * - Endpoint
     - Description
     - Since
   * - ``GET /apps``
     - List all apps
     - 0.2.0
   * - ``GET /apps/{id}``
     - Get app capabilities
     - 0.2.0
   * - ``GET /apps/{id}/data``
     - Get app topic data
     - 0.2.0
   * - ``GET /apps/{id}/operations``
     - List app operations
     - 0.2.0
   * - ``GET /apps/{id}/configurations``
     - List app configurations
     - 0.2.0
   * - ``GET /functions``
     - List all functions
     - 0.2.0
   * - ``GET /functions/{id}``
     - Get function capabilities
     - 0.2.0
   * - ``GET /functions/{id}/hosts``
     - List function host apps
     - 0.2.0
   * - ``GET /functions/{id}/data``
     - Aggregated data from hosts
     - 0.2.0
   * - ``GET /functions/{id}/operations``
     - Aggregated operations
     - 0.2.0

Next Steps
----------

- :doc:`/config/manifest-schema` - Complete YAML schema reference
- :doc:`migration-to-manifest` - Migrate from runtime-only mode
- :doc:`/getting_started` - Basic gateway setup

.. seealso::

   - `ASAM SOVD Standard <https://www.asam.net/standards/detail/sovd/>`_
   - `Example Manifests <config/examples>`_ in the repository
