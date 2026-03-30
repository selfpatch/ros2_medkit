Tutorials
=========

Step-by-step guides for common use cases with ros2_medkit.

.. toctree::
   :maxdepth: 1

   demos/index
   heuristic-apps
   manifest-discovery
   migration-to-manifest
   authentication
   https
   locking
   scripts
   snapshots
   fault-correlation
   docker
   devcontainer
   integration
   custom_areas
   web-ui
   mcp-server
   openapi
   beacon-discovery
   plugin-system
   linux-introspection
   triggers-use-cases
   multi-instance

Demos
-----

:doc:`demos/index`
   Walkthroughs for running ros2_medkit with demo systems including
   the built-in sensor demo and TurtleBot3 simulation.

Discovery Tutorials
-------------------

:doc:`heuristic-apps`
   Use heuristic discovery to automatically map ROS 2 nodes to SOVD Apps
   without any configuration.

Manifest Discovery
------------------

:doc:`manifest-discovery`
   Use YAML manifests to define your ROS 2 system structure with stable IDs,
   semantic groupings, and offline detection.

:doc:`migration-to-manifest`
   Migrate from runtime-only discovery to hybrid mode for better control
   over entity organization.

Basic Tutorials
---------------

:doc:`authentication`
   Configure JWT-based authentication with role-based access control.

:doc:`https`
   Enable TLS/HTTPS for secure communication.

:doc:`locking`
   Use SOVD resource locking to prevent concurrent modification of entity resources.

:doc:`scripts`
   Upload, execute, and manage diagnostic scripts on entities.

:doc:`snapshots`
   Configure snapshot capture for fault debugging.

:doc:`fault-correlation`
   Configure fault correlation for root-cause analysis and noise reduction.

:doc:`docker`
   Deploy ros2_medkit in Docker containers.

:doc:`devcontainer`
   Set up a VS Code development container for ros2_medkit.

Companion Projects
------------------

:doc:`web-ui`
   ros2_medkit_web_ui — A web interface for browsing SOVD entity trees.

:doc:`mcp-server`
   ros2_medkit_mcp — Connect LLMs to your ROS 2 system via MCP protocol.

:doc:`openapi`
   Explore and interact with the gateway's self-describing OpenAPI spec and Swagger UI.

Advanced Tutorials
------------------

:doc:`integration`
   Integrate ros2_medkit with your existing ROS 2 system.

:doc:`custom_areas`
   Customize the entity hierarchy for your robot architecture.

:doc:`beacon-discovery`
   Use topic and parameter beacon plugins to enrich entities with runtime metadata from nodes.

:doc:`plugin-system`
   Extend the gateway with custom plugins for update backends, introspection, and REST endpoints.

:doc:`linux-introspection`
   Enrich discovery with Linux process, systemd, and container metadata.

:doc:`triggers-use-cases`
   Set up multi-trigger monitoring scenarios for OTA updates, thermal protection, and fleet diagnostics.

:doc:`multi-instance`
   Federate multiple gateway instances into a single API with peer aggregation.
