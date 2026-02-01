Tutorials
=========

Step-by-step guides for common use cases with ros2_medkit.

.. toctree::
   :maxdepth: 1

   heuristic-apps
   manifest-discovery
   migration-to-manifest
   authentication
   https
   snapshots
   fault-correlation
   docker
   devcontainer
   integration
   custom_areas
   web-ui
   mcp-server

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
   sovd_web_ui — A web interface for browsing SOVD entity trees.

:doc:`mcp-server`
   ros2_medkit_mcp — Connect LLMs to your ROS 2 system via MCP protocol.

Advanced Tutorials
------------------

:doc:`integration`
   Integrate ros2_medkit with your existing ROS 2 system.

:doc:`custom_areas`
   Customize the entity hierarchy for your robot architecture.
