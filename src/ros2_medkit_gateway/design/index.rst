ros2_medkit_gateway
===================

This section contains design documentation for the ros2_medkit_gateway project.

Architecture
------------

The following diagram shows the relationships between the main components of the gateway.

.. plantuml::
   :caption: ROS 2 Medkit Gateway Class Architecture

   @startuml ros2_medkit_gateway_architecture

   skinparam linetype ortho
   skinparam classAttributeIconSize 0

   title ROS 2 Medkit Gateway - Class Architecture

   package "ROS 2 Framework" {
       class "rclcpp::Node" {
           +get_node_names()
           +get_topic_names_and_types()
           +get_service_names_and_types()
       }
   }

   package "ros2_medkit_gateway" {

       class GatewayNode {
           + get_entity_cache(): EntityCache
           + get_data_access_manager(): DataAccessManager*
       }

       class DiscoveryManager {
           + discover_areas(): vector<Area>
           + discover_components(): vector<Component>
       }

       class RESTServer {
           + start(): void
           + stop(): void
       }

       class DataAccessManager {
           + get_topic_sample(): json
           + get_component_data(): json
           - find_component_topics(): vector<string>
       }

       class ROS2CLIWrapper {
           + exec(): string
           + is_command_available(): bool
           + escape_shell_arg(): string
       }

       class OutputParser {
           + parse_yaml(): json
           - yaml_to_json(): json
       }

       class Area {
           + id: string
           + namespace_path: string
           + type: string
           + to_json(): json
       }

       class Component {
           + id: string
           + namespace_path: string
           + fqn: string
           + type: string
           + area: string
           + to_json(): json
       }

       class EntityCache {
           + areas: vector<Area>
           + components: vector<Component>
           + last_update: time_point
       }
   }

   package "External Libraries" {
       class "httplib::Server" as HTTPLibServer
       class "nlohmann::json" as JSON
   }

   ' Relationships

   ' Inheritance
   GatewayNode -up-|> "rclcpp::Node" : extends

   ' Composition (Gateway owns these)
   GatewayNode *-down-> DiscoveryManager : owns
   GatewayNode *-down-> RESTServer : owns
   GatewayNode *-down-> DataAccessManager : owns
   GatewayNode *-down-> EntityCache : owns

   ' Discovery Manager uses Node interface
   DiscoveryManager --> "rclcpp::Node" : uses

   ' REST Server references Gateway and DataAccessManager
   RESTServer --> GatewayNode : uses
   RESTServer --> DataAccessManager : uses

   ' DataAccessManager owns utility classes
   DataAccessManager *--> ROS2CLIWrapper : owns
   DataAccessManager *--> OutputParser : owns

   ' Entity Cache aggregates entities
   EntityCache o-right-> Area : contains many
   EntityCache o-right-> Component : contains many

   ' Discovery produces entities
   DiscoveryManager ..> Area : creates
   DiscoveryManager ..> Component : creates

   ' REST Server uses HTTP library
   RESTServer *--> HTTPLibServer : owns

   ' Models use JSON for serialization
   Area ..> JSON : serializes to
   Component ..> JSON : serializes to

   @enduml

Main Components
---------------

1. **GatewayNode** - The main ROS 2 node that orchestrates the system
   - Extends ``rclcpp::Node``
   - Manages periodic discovery and cache refresh
   - Runs the REST server in a separate thread
   - Provides thread-safe access to the entity cache

2. **DiscoveryManager** - Discovers ROS 2 entities and maps them to the SOVD hierarchy
   - Discovers Areas from node namespaces
   - Discovers Components from nodes, topics, and services
   - Extracts the entity hierarchy from the ROS 2 graph

3. **RESTServer** - Provides the HTTP/REST API
   - Serves endpoints: ``/health``, ``/``, ``/areas``, ``/components``, ``/areas/{area_id}/components``, ``/components/{component_id}/data``
   - Retrieves cached entities from the GatewayNode
   - Uses DataAccessManager for runtime topic data access
   - Runs on configurable host and port

4. **DataAccessManager** - Reads runtime data from ROS 2 topics
   - Samples topics using ROS 2 CLI (``ros2 topic echo``)
   - Handles timeout and error cases gracefully
   - Returns topic data as JSON with metadata (topic name, timestamp, data)
   - Configurable timeout per topic (default: 3 seconds for slow publishers)
   - Parallel topic sampling with configurable concurrency limit (``max_parallel_topic_samples``, default: 10)
   - Batched processing to bound resource usage while improving performance

5. **ROS2CLIWrapper** - Executes ROS 2 CLI commands safely
   - Wraps ``popen()`` with RAII for exception safety during command execution
   - Checks command exit status to detect failures
   - Prevents command injection with shell argument escaping
   - Validates command availability before execution

6. **OutputParser** - Converts ROS 2 CLI output to JSON
   - Parses YAML output from ``ros2 topic echo`` command
   - Preserves type information (bool → int → double → string precedence)
   - Handles multi-document YAML streams correctly
   - Converts ROS message structures to nested JSON objects

7. **Data Models** - Entity representations
   - ``Area`` - Physical or logical domain
   - ``Component`` - Hardware or software component
   - ``EntityCache`` - Thread-safe cache of discovered entities

