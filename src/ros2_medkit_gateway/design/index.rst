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
           + get_configuration_manager(): ConfigurationManager*
       }

       class DiscoveryManager {
           + discover_areas(): vector<Area>
           + discover_components(): vector<Component>
           + discover_apps(): vector<App>
           + discover_services(): vector<ServiceInfo>
           + discover_actions(): vector<ActionInfo>
           + find_service_for_component(): optional<ServiceInfo>
           + find_action_for_component(): optional<ActionInfo>
       }

       interface DiscoveryStrategy <<interface>> {
           + discover_areas(): vector<Area>
           + discover_components(): vector<Component>
           + discover_apps(): vector<App>
           + discover_functions(): vector<Function>
           + get_name(): string
       }

       class RuntimeDiscoveryStrategy {
           + discover_node_components(): vector<Component>
           + discover_synthetic_components(): vector<Component>
           + discover_topic_components(): vector<Component>
           - config_: RuntimeConfig
       }

       class ManifestDiscoveryStrategy {
           + load_manifest(): void
           - manifest_: Manifest
       }

       class HybridDiscoveryStrategy {
           - primary_: ManifestDiscoveryStrategy
           - runtime_: RuntimeDiscoveryStrategy
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

       class ConfigurationManager {
           + list_parameters(): ParameterResult
           + get_parameter(): ParameterResult
           + set_parameter(): ParameterResult
       }

       class NativeTopicSampler {
           + discover_all_topics(): vector<TopicInfo>
           + discover_topics(): vector<TopicInfo>
           + sample_topic(): TopicSampleResult
           + sample_topics_parallel(): vector<TopicSampleResult>
       }

       class JsonSerializer {
           + serialize(): SerializedMessage
           + deserialize(): json
           + to_json(): json
           + from_json(): RosMessage_Cpp
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
           + source: string
           + services: vector<ServiceInfo>
           + actions: vector<ActionInfo>
           + to_json(): json
       }

       class App {
           + id: string
           + name: string
           + namespace_path: string
           + area: string
           + component: string
           + source: string
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
           + apps: vector<App>
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
   GatewayNode *-down-> ConfigurationManager : owns
   GatewayNode *-down-> EntityCache : owns

   ' Discovery Manager uses Node interface
   DiscoveryManager --> "rclcpp::Node" : uses

   ' REST Server references Gateway, DataAccessManager, OperationManager, and ConfigurationManager
   RESTServer --> GatewayNode : uses
   RESTServer --> DataAccessManager : uses
   RESTServer --> OperationManager : uses
   RESTServer --> ConfigurationManager : uses

   ' OperationManager uses DiscoveryManager and native serialization
   OperationManager --> DiscoveryManager : uses
   OperationManager *--> JsonSerializer : owns

   ' DataAccessManager owns utility classes and uses native publishing
   DataAccessManager *--> JsonSerializer : owns (serialization)
   DataAccessManager *--> NativeTopicSampler : owns

   ' NativeTopicSampler uses Node interface
   NativeTopicSampler --> "rclcpp::Node" : uses

   ' ConfigurationManager uses Node interface for parameter clients
   ConfigurationManager --> "rclcpp::Node" : uses

   ' Entity Cache aggregates entities
   EntityCache o-right-> Area : contains many
   EntityCache o-right-> Component : contains many
   EntityCache o-right-> App : contains many

   ' Component contains operations
   Component o--> ServiceInfo : contains many
   Component o--> ActionInfo : contains many

   ' Discovery produces entities
   DiscoveryManager ..> Area : creates
   DiscoveryManager ..> Component : creates
   DiscoveryManager ..> App : creates
   DiscoveryManager ..> ServiceInfo : creates
   DiscoveryManager ..> ActionInfo : creates

   ' Discovery strategy hierarchy
   DiscoveryManager --> DiscoveryStrategy : uses
   RuntimeDiscoveryStrategy .up.|> DiscoveryStrategy : implements
   ManifestDiscoveryStrategy .up.|> DiscoveryStrategy : implements
   HybridDiscoveryStrategy .up.|> DiscoveryStrategy : implements
   HybridDiscoveryStrategy --> ManifestDiscoveryStrategy : delegates
   HybridDiscoveryStrategy --> RuntimeDiscoveryStrategy : delegates

   ' REST Server uses HTTP library
   RESTServer *--> HTTPLibServer : owns

   ' Models use JSON for serialization
   Area ..> JSON : serializes to
   Component ..> JSON : serializes to
   App ..> JSON : serializes to

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
   - Discovers Areas from node namespaces or manifest definitions
   - Discovers Components (synthetic groups from runtime, or explicit from manifest)
   - Discovers Apps from ROS 2 nodes (individual running processes)
   - Discovers Services and Actions using native rclcpp APIs
   - Attaches operations (services/actions) to their parent Apps and Components
   - Uses pluggable strategy pattern: Runtime, Manifest, or Hybrid
   - Uses O(n+m) algorithm with hash maps for efficient service/action attachment

   **Discovery Strategies:**

   - **RuntimeDiscoveryStrategy** - Heuristic discovery via ROS 2 graph introspection
     - Maps nodes to Apps with ``source: "heuristic"``
     - Creates synthetic Components grouped by namespace
     - Handles topic-only namespaces (Isaac Sim, bridges) via TopicOnlyPolicy
   - **ManifestDiscoveryStrategy** - Static discovery from YAML manifest
     - Provides stable, semantic entity IDs
     - Supports offline detection of failed components
   - **HybridDiscoveryStrategy** - Combines manifest + runtime
     - Manifest defines structure, runtime links to live nodes
     - Best for production systems requiring stability + live status

3. **OperationManager** - Executes ROS 2 operations (services and actions) using native APIs
   - Calls ROS 2 services via ``rclcpp::GenericClient`` with native serialization
   - Sends action goals via native action client interfaces
   - Tracks active action goals with status, feedback, and timestamps
   - Subscribes to ``/_action/status`` topics for real-time goal status updates
   - Supports goal cancellation via native cancel service calls
   - Supports SOVD capability-based control (stop maps to ROS 2 cancel)
   - Automatically cleans up completed goals older than 5 minutes
   - Uses ``ros2_medkit_serialization`` for JSON ↔ ROS 2 message conversion

4. **RESTServer** - Provides the HTTP/REST API
   - Discovery endpoints: ``/health``, ``/areas``, ``/components``
   - Data endpoints: ``/components/{id}/data``, ``/components/{id}/data/{topic}``
   - Operations endpoints: ``/apps/{id}/operations``, ``/apps/{id}/operations/{op}/executions``
   - Configurations endpoints: ``/apps/{id}/configurations``, ``/apps/{id}/configurations/{param}``
   - Retrieves cached entities from the GatewayNode
   - Uses DataAccessManager for runtime topic data access
   - Uses OperationManager for service/action execution
   - Uses ConfigurationManager for parameter CRUD operations
   - Runs on configurable host and port with CORS support

5. **ConfigurationManager** - Manages ROS 2 node parameters
   - Lists all parameters for a node via ``rclcpp::SyncParametersClient``
   - Gets/sets individual parameter values with type conversion
   - Provides parameter descriptors (description, constraints, read-only flag)
   - Caches parameter clients per node for efficiency
   - Converts between JSON and ROS 2 parameter types automatically

6. **DataAccessManager** - Reads and writes runtime data from/to ROS 2 topics
   - Uses native rclcpp APIs for fast topic discovery and sampling
   - Checks publisher counts before sampling to skip idle topics instantly
   - Returns metadata (type, schema) for topics without publishers
   - Uses native ``rclcpp::GenericPublisher`` for topic publishing with CDR serialization
   - Returns topic data as JSON with metadata (topic name, timestamp, type info)
   - Parallel topic sampling with configurable concurrency limit (``max_parallel_topic_samples``, default: 10)

7. **NativeTopicSampler** - Fast topic sampling using native rclcpp APIs
   - Discovers topics via ``node->get_topic_names_and_types()``
   - Uses ``rclcpp::GenericSubscription`` for type-agnostic message sampling
   - Checks ``count_publishers()`` before sampling to skip idle topics
   - Returns metadata instantly for topics without publishers (no timeout)
   - Significantly improves UX when robot has many idle topics

8. **JsonSerializer** (ros2_medkit_serialization) - Converts between JSON and ROS 2 messages
   - Uses ``dynmsg`` library for dynamic type introspection
   - Serializes JSON to CDR format for publishing via ``serialize()``
   - Deserializes CDR to JSON for subscriptions via ``deserialize()``
   - Converts between deserialized ROS 2 messages and JSON via ``to_json()`` / ``from_json()``
   - Provides static ``yaml_to_json()`` utility for YAML to JSON conversion
   - Thread-safe and stateless design

9. **Data Models** - Entity representations
    - ``Area`` - Physical or logical domain (namespace grouping)
    - ``Component`` - Logical grouping of Apps; can be ``synthetic`` (auto-created from namespace), ``topic`` (from topic-only namespace), or ``manifest`` (explicitly defined)
    - ``App`` - Software application (ROS 2 node); individual running process linked to parent Component
    - ``ServiceInfo`` - Service metadata (path, name, type)
    - ``ActionInfo`` - Action metadata (path, name, type)
    - ``EntityCache`` - Thread-safe cache of discovered entities (areas, components, apps)
