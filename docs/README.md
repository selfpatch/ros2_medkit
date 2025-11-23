# ROS 2 Medkit Gateway - Architecture Documentation

This directory contains design documentation for the ros2_medkit_gateway project.

## Architecture Diagram

The `architecture.puml` file contains a PlantUML class diagram showing the relationships between the main components of the gateway.

### Main Components

1. **GatewayNode** - The main ROS 2 node that orchestrates the system
   - Extends `rclcpp::Node`
   - Manages periodic discovery and cache refresh
   - Runs the REST server in a separate thread
   - Provides thread-safe access to the entity cache

2. **DiscoveryManager** - Discovers ROS 2 entities and maps them to the SOVD hierarchy
   - Discovers Areas from node namespaces
   - Discovers Components from nodes, topics, and services
   - Extracts the entity hierarchy from the ROS 2 graph

3. **RESTServer** - Provides the HTTP/REST API
   - Serves endpoints: `/health`, `/`, `/areas`, `/components`, `/areas/{area_id}/components`
   - Retrieves cached entities from the GatewayNode
   - Runs on configurable host and port

4. **Data Models** - Entity representations
   - `Area` - Physical or logical domain
   - `Component` - Hardware or software component
   - `EntityCache` - Thread-safe cache of discovered entities
   