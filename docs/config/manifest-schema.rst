Manifest Schema Reference
=========================

This document describes the complete YAML schema for SOVD system manifests.

.. contents:: Table of Contents
   :local:
   :depth: 2

Top-Level Structure
-------------------

A manifest file has the following top-level structure:

.. code-block:: yaml

   manifest_version: "1.0"    # Required - manifest schema version

   metadata:              # Optional - document metadata
     name: string
     version: string
     description: string

   config:                # Optional - discovery behavior settings
     unmanifested_nodes: string
     inherit_runtime_resources: boolean
     allow_manifest_override: boolean

   areas: []              # Optional - area definitions
   components: []         # Optional - component definitions
   apps: []               # Optional - app definitions
   functions: []          # Optional - function definitions

manifest_version (Required)
~~~~~~~~~~~~~~~~~~~~~~~~~~~

The manifest schema version. Currently must be ``"1.0"``.

.. code-block:: yaml

   manifest_version: "1.0"

metadata (Optional)
~~~~~~~~~~~~~~~~~~~

Document metadata for identification and documentation.

.. list-table::
   :header-rows: 1
   :widths: 20 15 65

   * - Field
     - Type
     - Description
   * - ``name``
     - string
     - System/robot name (e.g., "turtlebot3-nav2")
   * - ``version``
     - string
     - Manifest version (e.g., "1.0.0")
   * - ``description``
     - string
     - Human-readable description

.. code-block:: yaml

   metadata:
     name: "my-robot"
     version: "2.0.0"
     description: "Mobile robot with Nav2 navigation stack"

config (Optional)
~~~~~~~~~~~~~~~~~

Discovery behavior configuration.

.. list-table::
   :header-rows: 1
   :widths: 25 15 60

   * - Field
     - Type
     - Description
   * - ``unmanifested_nodes``
     - string
     - Policy for ROS nodes not in manifest
   * - ``inherit_runtime_resources``
     - boolean
     - Copy topics/services from runtime nodes (default: true)
   * - ``allow_manifest_override``
     - boolean
     - Manifest values can override runtime (default: true)

**unmanifested_nodes options:**

- ``ignore`` - Don't expose unmanifested nodes
- ``warn`` - Log warning, include as orphans (default)
- ``error`` - Fail startup if orphan nodes detected
- ``include_as_orphan`` - Include with ``source: "orphan"``

.. code-block:: yaml

   config:
     unmanifested_nodes: warn
     inherit_runtime_resources: true
     allow_manifest_override: true

Areas
-----

Areas represent logical or physical groupings (subsystems, locations, etc.).
In runtime-only mode, areas are derived from ROS 2 namespaces.

Schema
~~~~~~

.. code-block:: yaml

   areas:
     - id: string              # Required - unique identifier
       name: string            # Required - human-readable name
       namespace: string       # Optional - ROS 2 namespace path
       category: string        # Optional - classification
       description: string     # Optional - detailed description
       tags: [string]          # Optional - tags for filtering
       translation_id: string  # Optional - i18n key
       parent_area_id: string  # Optional - parent area reference
       subareas: []            # Optional - nested area definitions

Fields
~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 20 15 10 55

   * - Field
     - Type
     - Required
     - Description
   * - ``id``
     - string
     - Yes
     - Unique identifier (alphanumeric, hyphens allowed)
   * - ``name``
     - string
     - Yes
     - Human-readable name
   * - ``namespace``
     - string
     - No
     - ROS 2 namespace path (e.g., "/perception")
   * - ``category``
     - string
     - No
     - Classification for filtering
   * - ``description``
     - string
     - No
     - Detailed description
   * - ``tags``
     - [string]
     - No
     - Tags for filtering and grouping
   * - ``translation_id``
     - string
     - No
     - Internationalization key
   * - ``parent_area_id``
     - string
     - No
     - Parent area ID (for flat hierarchy definition)
   * - ``subareas``
     - [Area]
     - No
     - Nested area definitions

Example
~~~~~~~

.. code-block:: yaml

   areas:
     - id: perception
       name: "Perception Subsystem"
       category: "sensor-processing"
       description: "Sensor data acquisition and processing"
       tags:
         - sensors
         - realtime
       subareas:
         - id: lidar-processing
           name: "LiDAR Processing"
           description: "Point cloud processing pipeline"

         - id: camera-processing
           name: "Camera Processing"
           description: "Image processing pipeline"

     - id: navigation
       name: "Navigation Subsystem"
       category: "motion-planning"

Components
----------

Components represent hardware or virtual entities (ECUs, sensors, controllers).
In runtime-only mode, synthetic components are created per namespace to group Apps (nodes).
In manifest mode, components are explicitly defined and Apps are linked to them.

Schema
~~~~~~

.. code-block:: yaml

   components:
     - id: string              # Required - unique identifier
       name: string            # Required - human-readable name
       type: string            # Optional - component type
       category: string        # Optional - classification
       area: string            # Optional - reference to area.id
       namespace: string       # Optional - ROS 2 namespace
       fqn: string             # Optional - fully qualified name
       variant: string         # Optional - hardware variant
       description: string     # Optional - detailed description
       tags: [string]          # Optional - tags for filtering
       translation_id: string  # Optional - i18n key
       parent_component_id: string  # Optional - parent component
       depends_on: [string]    # Optional - component IDs this depends on
       subcomponents: []       # Optional - nested definitions

Fields
~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 22 15 10 53

   * - Field
     - Type
     - Required
     - Description
   * - ``id``
     - string
     - Yes
     - Unique identifier
   * - ``name``
     - string
     - Yes
     - Human-readable name
   * - ``type``
     - string
     - No
     - Component type (sensor, actuator, controller, etc.)
   * - ``category``
     - string
     - No
     - Classification for filtering
   * - ``area``
     - string
     - No
     - Parent area ID
   * - ``namespace``
     - string
     - No
     - ROS 2 namespace path
   * - ``fqn``
     - string
     - No
     - Fully qualified name (namespace + id)
   * - ``variant``
     - string
     - No
     - Hardware variant identifier
   * - ``description``
     - string
     - No
     - Detailed description
   * - ``tags``
     - [string]
     - No
     - Tags for filtering
   * - ``translation_id``
     - string
     - No
     - Internationalization key
   * - ``parent_component_id``
     - string
     - No
     - Parent component ID
   * - ``depends_on``
     - [string]
     - No
     - List of component IDs this component depends on
   * - ``subcomponents``
     - [Component]
     - No
     - Nested component definitions

Common Component Types
~~~~~~~~~~~~~~~~~~~~~~

- ``sensor`` - Sensors (LiDAR, camera, IMU)
- ``actuator`` - Actuators (motors, grippers)
- ``controller`` - Controllers (main computer, ECU)
- ``accelerator`` - Compute accelerators (GPU, TPU)
- ``communication`` - Communication interfaces
- ``power`` - Power management

Example
~~~~~~~

.. code-block:: yaml

   components:
     - id: main-computer
       name: "Main Computer"
       type: "controller"
       area: control
       description: "Raspberry Pi 4 running ROS 2"
       variant: "rpi4-8gb"
       subcomponents:
         - id: gpu-unit
           name: "GPU Processing Unit"
           type: "accelerator"

     - id: lidar-sensor
       name: "LiDAR Sensor"
       type: "sensor"
       area: perception
       description: "360° laser range finder"
       tags:
         - safety-critical
         - realtime

     - id: imu-sensor
       name: "IMU Sensor"
       type: "sensor"
       area: perception

Apps
----

Apps represent software applications, typically mapping 1:1 to ROS 2 nodes.
Apps exist only in manifest and hybrid modes.

Schema
~~~~~~

.. code-block:: yaml

   apps:
     - id: string              # Required - unique identifier
       name: string            # Required - human-readable name
       category: string        # Optional - classification
       is_located_on: string   # Optional - component ID
       depends_on: [string]    # Optional - app IDs this app depends on
       description: string     # Optional - detailed description
       tags: [string]          # Optional - tags for filtering
       translation_id: string  # Optional - i18n key
       external: boolean       # Optional - not a ROS node (default: false)

       ros_binding:            # Required for hybrid mode linking
         node_name: string     # Required - ROS node name
         namespace: string     # Optional - namespace (default: /)
         topic_namespace: string  # Optional - match by topic prefix

Fields
~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 20 15 10 55

   * - Field
     - Type
     - Required
     - Description
   * - ``id``
     - string
     - Yes
     - Unique identifier
   * - ``name``
     - string
     - Yes
     - Human-readable name
   * - ``category``
     - string
     - No
     - Classification for filtering
   * - ``is_located_on``
     - string
     - No
     - Component ID where app runs
   * - ``depends_on``
     - [string]
     - No
     - List of app IDs this app depends on
   * - ``description``
     - string
     - No
     - Detailed description
   * - ``tags``
     - [string]
     - No
     - Tags for filtering
   * - ``translation_id``
     - string
     - No
     - Internationalization key
   * - ``external``
     - boolean
     - No
     - True if not a ROS node (default: false)

ros_binding Fields
~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 22 15 10 53

   * - Field
     - Type
     - Required
     - Description
   * - ``node_name``
     - string
     - Yes*
     - ROS 2 node name to bind to
   * - ``namespace``
     - string
     - No
     - Namespace ("*" for wildcard)
   * - ``topic_namespace``
     - string
     - Yes*
     - Alternative: match by topic prefix

\* Either ``node_name`` or ``topic_namespace`` is required.

**Matching behavior:**

1. **Exact match** (default): ``node_name`` and ``namespace`` must match exactly
2. **Wildcard namespace**: Set ``namespace: "*"`` to match node in any namespace
3. **Topic namespace**: Match nodes by their published topic prefix

Example
~~~~~~~

.. code-block:: yaml

   apps:
     # Match by exact node name and namespace
     - id: lidar-driver
       name: "LiDAR Driver"
       is_located_on: lidar-sensor
       ros_binding:
         node_name: velodyne_driver
         namespace: /sensors

     # Match node in any namespace
     - id: camera-driver
       name: "Camera Driver"
       ros_binding:
         node_name: usb_cam
         namespace: "*"

     # Match by topic namespace
     - id: perception-pipeline
       name: "Perception Pipeline"
       ros_binding:
         topic_namespace: /perception

     # App with dependencies
     - id: slam-node
       name: "SLAM Node"
       category: "localization"
       is_located_on: main-computer
       depends_on:
         - lidar-driver
         - imu-driver
       ros_binding:
         node_name: slam_toolbox
         namespace: /mapping

     # External app (not a ROS node)
     - id: cloud-connector
       name: "Cloud Connector"
       external: true
       description: "External cloud service integration"

Functions
---------

Functions represent high-level capabilities spanning multiple apps.
Functions are always manifest-defined and aggregate data from their host apps.

Schema
~~~~~~

.. code-block:: yaml

   functions:
     - id: string              # Required - unique identifier
       name: string            # Required - human-readable name
       category: string        # Optional - classification
       hosted_by: [string]     # Required - list of app IDs
       depends_on: [string]    # Optional - function IDs
       description: string     # Optional - detailed description
       tags: [string]          # Optional - tags for filtering
       translation_id: string  # Optional - i18n key

Fields
~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 20 15 10 55

   * - Field
     - Type
     - Required
     - Description
   * - ``id``
     - string
     - Yes
     - Unique identifier
   * - ``name``
     - string
     - Yes
     - Human-readable name
   * - ``hosted_by``
     - [string]
     - Yes
     - List of app IDs that implement this function
   * - ``category``
     - string
     - No
     - Classification for filtering
   * - ``depends_on``
     - [string]
     - No
     - List of function IDs this function depends on
   * - ``description``
     - string
     - No
     - Detailed description
   * - ``tags``
     - [string]
     - No
     - Tags for filtering
   * - ``translation_id``
     - string
     - No
     - Internationalization key

Function Capabilities
~~~~~~~~~~~~~~~~~~~~~

Functions aggregate capabilities from their host apps:

- **Data**: Combined topics from all host apps
- **Operations**: Combined services/actions from all host apps
- **Faults**: Faults from all host apps

Example
~~~~~~~

.. code-block:: yaml

   functions:
     - id: autonomous-navigation
       name: "Autonomous Navigation"
       category: "mobility"
       description: "Complete autonomous navigation capability"
       tags:
         - safety-critical
         - autonomous
       hosted_by:
         - amcl-node
         - planner-server
         - controller-server
         - bt-navigator

     - id: localization
       name: "Localization"
       category: "state-estimation"
       hosted_by:
         - amcl-node
         - map-server

     - id: perception
       name: "Environment Perception"
       category: "sensing"
       hosted_by:
         - lidar-driver
         - camera-driver
         - point-cloud-processor

Complete Example
----------------

Here's a complete manifest for a TurtleBot3 robot:

.. code-block:: yaml

   manifest_version: "1.0"

   metadata:
     name: "turtlebot3-nav2"
     version: "2.0.0"
     description: "TurtleBot3 with Nav2 navigation stack"

   config:
     unmanifested_nodes: warn
     inherit_runtime_resources: true

   areas:
     - id: perception
       name: "Perception"
       category: "sensor-processing"

     - id: navigation
       name: "Navigation"
       category: "motion-planning"

     - id: control
       name: "Control"
       category: "motion-control"

   components:
     - id: lidar-sensor
       name: "LiDAR Sensor"
       type: "sensor"
       area: perception

     - id: main-computer
       name: "Main Computer"
       type: "controller"
       area: control

   apps:
     - id: lidar-driver
       name: "LiDAR Driver"
       is_located_on: lidar-sensor
       ros_binding:
         node_name: ld08_driver

     - id: amcl-node
       name: "AMCL Localization"
       category: "localization"
       is_located_on: main-computer
       ros_binding:
         node_name: amcl

     - id: planner-server
       name: "Planner Server"
       category: "navigation"
       is_located_on: main-computer
       depends_on:
         - amcl-node
       ros_binding:
         node_name: planner_server

   functions:
     - id: autonomous-navigation
       name: "Autonomous Navigation"
       category: "mobility"
       hosted_by:
         - amcl-node
         - planner-server

Validation
----------

Manifests are validated during loading. The validator checks:

**Required fields:**

- ``manifest_version`` must be present and equal to "1.0"
- All entities must have ``id`` and ``name``
- Apps with ``ros_binding`` must have ``node_name`` or ``topic_namespace``
- Functions must have at least one entry in ``hosted_by``

**References:**

- ``area`` references must point to valid area IDs
- ``is_located_on`` must point to valid component IDs
- ``depends_on`` must point to valid app/function IDs
- ``hosted_by`` must point to valid app IDs

**Uniqueness:**

- All entity IDs must be unique within their type
- IDs should be unique across all types (warning)

**Format:**

- IDs should contain only alphanumeric characters and hyphens
- IDs should not start with numbers

Validation errors are reported with the path to the invalid field:

.. code-block:: text

   Validation error at apps[2].ros_binding: 'node_name' or 'topic_namespace' required
   Validation error at functions[0].hosted_by[1]: App 'unknown-app' not found

.. seealso::

   - :doc:`/tutorials/manifest-discovery` - User guide for manifest-based discovery
   - :doc:`/tutorials/migration-to-manifest` - Migration guide
