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
           + get_operation_manager(): OperationManager*
           + get_discovery_manager(): DiscoveryManager*
       }

       class DiscoveryManager {
           + discover_areas(): vector<Area>
           + discover_components(): vector<Component>
           + discover_services(): vector<ServiceInfo>
           + discover_actions(): vector<ActionInfo>
           + find_service_for_component(): optional<ServiceInfo>
           + find_action_for_component(): optional<ActionInfo>
       }

       class OperationManager {
           + call_service(): ServiceCallResult
           + call_component_service(): ServiceCallResult
           + send_action_goal(): ActionSendGoalResult
           + send_component_action_goal(): ActionSendGoalResult
           + cancel_action_goal(): ActionCancelResult
           + get_tracked_goal(): optional<ActionGoalInfo>
           + get_latest_goal_for_action(): optional<ActionGoalInfo>
           + cleanup_old_goals(): void
           + is_valid_uuid_hex(): bool {static}
           + is_service_type(): bool {static}
           + is_action_type(): bool {static}
       }

       class RESTServer {
           + start(): void
           + stop(): void
       }

       class DataAccessManager {
           + get_topic_sample_with_fallback(): json
           + get_component_data_with_fallback(): json
           + publish_to_topic(): json
           + get_topic_sample_native(): json
           + get_component_data_native(): json
       }

       class NativeTopicSampler {
           + discover_all_topics(): vector<TopicInfo>
           + discover_topics(): vector<TopicInfo>
           + sample_topic(): TopicSampleResult
           + sample_topics_parallel(): vector<TopicSampleResult>
       }

       class ROS2CLIWrapper {
           + exec(): string
           + is_command_available(): bool
           + escape_shell_arg(): string {static}
       }

       class OutputParser {
           + parse_yaml(): json
           + yaml_to_json(): json {static}
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
           + services: vector<ServiceInfo>
           + actions: vector<ActionInfo>
           + to_json(): json
       }

       class ServiceInfo {
           + full_path: string
           + name: string
           + type: string
       }

       class ActionInfo {
           + full_path: string
           + name: string
           + type: string
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
   GatewayNode *-down-> OperationManager : owns
   GatewayNode *-down-> EntityCache : owns

   ' Discovery Manager uses Node interface
   DiscoveryManager --> "rclcpp::Node" : uses

   ' REST Server references Gateway, DataAccessManager, and OperationManager
   RESTServer --> GatewayNode : uses
   RESTServer --> DataAccessManager : uses
   RESTServer --> OperationManager : uses

   ' OperationManager uses DiscoveryManager and CLI
   OperationManager --> DiscoveryManager : uses
   OperationManager *--> ROS2CLIWrapper : owns

   ' DataAccessManager owns utility classes
   DataAccessManager *--> ROS2CLIWrapper : owns (publishing)
   DataAccessManager *--> OutputParser : owns
   DataAccessManager *--> NativeTopicSampler : owns

   ' NativeTopicSampler uses Node interface
   NativeTopicSampler --> "rclcpp::Node" : uses

   ' Entity Cache aggregates entities
   EntityCache o-right-> Area : contains many
   EntityCache o-right-> Component : contains many

   ' Component contains operations
   Component o--> ServiceInfo : contains many
   Component o--> ActionInfo : contains many

   ' Discovery produces entities
   DiscoveryManager ..> Area : creates
   DiscoveryManager ..> Component : creates
   DiscoveryManager ..> ServiceInfo : creates
   DiscoveryManager ..> ActionInfo : creates

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
   - Manages periodic cleanup of old action goals (60s interval)

2. **DiscoveryManager** - Discovers ROS 2 entities and maps them to the SOVD hierarchy
   - Discovers Areas from node namespaces
   - Discovers Components from nodes, topics, and services
   - Discovers Services and Actions using native rclcpp APIs
   - Attaches operations (services/actions) to their parent components
   - Uses O(n+m) algorithm with hash maps for efficient service/action attachment

3. **OperationManager** - Executes ROS 2 operations (services and actions)
   - Calls ROS 2 services synchronously via ``ros2 service call`` CLI
   - Sends action goals via ``ros2 action send_goal`` CLI (3s timeout for acceptance)
   - Tracks active action goals with status, feedback, and timestamps
   - Subscribes to ``/_action/status`` topics for real-time goal status updates
   - Supports goal cancellation via ``ros2 action cancel`` CLI
   - Automatically cleans up completed goals older than 5 minutes

4. **RESTServer** - Provides the HTTP/REST API
   - Discovery endpoints: ``/health``, ``/``, ``/areas``, ``/components``, ``/areas/{area_id}/components``
   - Data endpoints: ``/components/{component_id}/data``, ``/components/{component_id}/data/{topic_name}``
   - Operations endpoints: ``POST .../operations/{op}`` (execute), ``GET .../operations/{op}/status`` (status), ``DELETE .../operations/{op}`` (cancel)
   - Retrieves cached entities from the GatewayNode
   - Uses DataAccessManager for runtime topic data access
   - Uses OperationManager for service/action execution
   - Runs on configurable host and port with CORS support

5. **DataAccessManager** - Reads runtime data from ROS 2 topics
   - Uses native rclcpp APIs for fast topic discovery and sampling
   - Checks publisher counts before sampling to skip idle topics instantly
   - Returns metadata (type, schema) for topics without publishers
   - Falls back to ROS 2 CLI only for publishing (``ros2 topic pub``)
   - Returns topic data as JSON with metadata (topic name, timestamp, type info)
   - Parallel topic sampling with configurable concurrency limit (``max_parallel_topic_samples``, default: 10)

6. **NativeTopicSampler** - Fast topic sampling using native rclcpp APIs
   - Discovers topics via ``node->get_topic_names_and_types()``
   - Checks ``count_publishers()`` before sampling to skip idle topics
   - Returns metadata instantly for topics without publishers (no CLI timeout)
   - Significantly improves UX when robot has many idle topics

7. **ROS2CLIWrapper** - Executes ROS 2 CLI commands safely
   - Used for publishing (``ros2 topic pub``), service calls, and action operations
   - Wraps ``popen()`` with RAII for exception safety during command execution
   - Checks command exit status to detect failures
   - Prevents command injection with shell argument escaping
   - Validates command availability before execution

8. **OutputParser** - Converts ROS 2 CLI output to JSON
   - Parses YAML output from ``ros2 topic echo`` and ``ros2 service call``
   - Preserves type information (bool → int → double → string precedence)
   - Handles multi-document YAML streams correctly
   - Converts ROS message structures to nested JSON objects
   - Provides static ``yaml_to_json()`` utility for reuse

9. **Data Models** - Entity representations
   - ``Area`` - Physical or logical domain
   - ``Component`` - Hardware or software component with attached operations
   - ``ServiceInfo`` - Service metadata (path, name, type)
   - ``ActionInfo`` - Action metadata (path, name, type)
   - ``EntityCache`` - Thread-safe cache of discovered entities

