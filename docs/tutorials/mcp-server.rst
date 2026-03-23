MCP Server for LLM Integration
==============================

ros2_medkit_mcp provides MCP (Model Context Protocol) tools for connecting
Large Language Models to your ROS 2 system via the ros2_medkit gateway.

.. contents:: Table of Contents
   :local:
   :depth: 2

Overview
--------

The MCP server enables:

- **Natural language queries** about robot state
- **LLM-assisted diagnostics** and troubleshooting
- **Autonomous agents** for robot monitoring and control

The server wraps the gateway's HTTP API, making it accessible to LLMs through
standardized MCP tools.

Quick Start
-----------

Prerequisites
~~~~~~~~~~~~~

- Python 3.11+
- Poetry
- A running ros2_medkit gateway

Installation
~~~~~~~~~~~~

.. code-block:: bash

   git clone https://github.com/selfpatch/ros2_medkit_mcp.git
   cd ros2_medkit_mcp
   poetry install

Running the Server
~~~~~~~~~~~~~~~~~~

**stdio transport** (for Claude Desktop, VS Code):

.. code-block:: bash

   poetry run ros2-medkit-mcp-stdio

**HTTP transport** (for remote access):

.. code-block:: bash

   poetry run ros2-medkit-mcp-http --host 0.0.0.0 --port 8765

Configuration
-------------

The server is configured via environment variables:

.. list-table::
   :header-rows: 1
   :widths: 30 30 40

   * - Variable
     - Default
     - Description
   * - ``ROS2_MEDKIT_BASE_URL``
     - ``http://localhost:8080/api/v1``
     - Base URL of the gateway API
   * - ``ROS2_MEDKIT_BEARER_TOKEN``
     - *(none)*
     - Optional JWT token for authentication
   * - ``ROS2_MEDKIT_TIMEOUT_S``
     - ``30``
     - HTTP request timeout in seconds

Claude Desktop Integration
--------------------------

Add to your ``claude_desktop_config.json``:

.. code-block:: json

   {
     "mcpServers": {
       "ros2_medkit": {
         "command": "poetry",
         "args": ["run", "ros2-medkit-mcp-stdio"],
         "cwd": "/path/to/ros2_medkit_mcp",
         "env": {
           "ROS2_MEDKIT_BASE_URL": "http://localhost:8080/api/v1"
         }
       }
     }
   }

VS Code Integration
-------------------

Create ``.vscode/mcp.json`` in your workspace:

**Option 1: stdio transport (local)**

.. code-block:: json

   {
     "servers": {
       "ros2_medkit": {
         "type": "stdio",
         "command": "poetry",
         "args": ["run", "ros2-medkit-mcp-stdio"],
         "cwd": "/path/to/ros2_medkit_mcp",
         "env": {
           "ROS2_MEDKIT_BASE_URL": "http://localhost:8080/api/v1"
         }
       }
     }
   }

**Option 2: HTTP transport (Docker/remote)**

.. code-block:: json

   {
     "servers": {
       "ros2_medkit": {
         "type": "sse",
         "url": "http://localhost:8765/mcp"
       }
     }
   }

Docker Usage
------------

.. code-block:: bash

   # Run HTTP server
   docker run -p 8765:8765 \
     -e ROS2_MEDKIT_BASE_URL=http://host.docker.internal:8080/api/v1 \
     ghcr.io/selfpatch/ros2_medkit_mcp:latest

   # Run with stdio transport
   docker run -i ghcr.io/selfpatch/ros2_medkit_mcp:latest stdio

MCP Tools Reference
-------------------

Discovery Tools
~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Tool
     - Description
   * - ``sovd_version``
     - Get SOVD API version information
   * - ``sovd_entities_list``
     - List all entities (areas, components, apps, functions)
   * - ``sovd_entities_get``
     - Get a specific entity by ID
   * - ``sovd_area_components``
     - List components within an area

Faults Tools
~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Tool
     - Description
   * - ``sovd_faults_list``
     - List faults for a component
   * - ``sovd_faults_get``
     - Get specific fault details
   * - ``sovd_faults_clear``
     - Clear/acknowledge a fault

Data Tools
~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Tool
     - Description
   * - ``sovd_entity_data``
     - Read all topic data from an entity
   * - ``sovd_entity_topic_data``
     - Read data from a specific topic
   * - ``sovd_publish_topic``
     - Publish data to a topic

Operations Tools
~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Tool
     - Description
   * - ``sovd_list_operations``
     - List operations (services/actions) for a component
   * - ``sovd_create_execution``
     - Call a service or send an action goal
   * - ``sovd_get_execution``
     - Get action execution status
   * - ``sovd_list_executions``
     - List all executions for an operation
   * - ``sovd_cancel_execution``
     - Cancel a running action

Configurations Tools
~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Tool
     - Description
   * - ``sovd_list_configurations``
     - List parameters for a component
   * - ``sovd_get_configuration``
     - Get a parameter value
   * - ``sovd_set_configuration``
     - Set a parameter value
   * - ``sovd_delete_configuration``
     - Reset parameter to default
   * - ``sovd_delete_all_configurations``
     - Reset all parameters

Example Prompts
---------------

Once configured, try these prompts with your LLM:

- *"List all components in the system"*
- *"What faults are reported by the lidar sensor?"*
- *"Show me the current temperature reading"*
- *"Call the calibrate service on the camera component"*
- *"What parameters does the sensor node have?"*

Repository
----------

https://github.com/selfpatch/ros2_medkit_mcp

See Also
--------

- :doc:`/getting_started` — Basic gateway setup
- :doc:`/tutorials/authentication` — Configure JWT authentication
- :doc:`/api/rest` — REST API reference
