ros2_medkit_serialization
=========================

This section contains design documentation for the ros2_medkit_serialization package.

Overview
--------

The ``ros2_medkit_serialization`` package provides runtime JSON ↔ ROS 2 message conversion
using the `dynmsg <https://github.com/osrf/dynamic_message_introspection>`_ library.
It enables the gateway to work with any ROS 2 message type without compile-time dependencies.

Architecture
------------

The following diagram shows the relationships between the main components.

.. plantuml::
   :caption: ROS 2 Medkit Serialization Class Architecture

   @startuml ros2_medkit_serialization_architecture

   skinparam linetype ortho
   skinparam classAttributeIconSize 0

   title ROS 2 Medkit Serialization - Class Architecture

   package "External Libraries" {
       class "dynmsg" {
           +ros_message_init()
           +ros_message_set_from_yaml()
           +ros_message_to_yaml()
           +ros_message_serialize()
           +ros_message_deserialize()
       }

       class "nlohmann::json" as JSON

       class "YAML::Node" as YAML

       class "rclcpp::SerializedMessage" as SerializedMessage
   }

   package "ros2_medkit_serialization" {

       class JsonSerializer {
           + to_json(type_info, data): json
           + to_json(type_string, data): json
           + from_json(type_info, json): RosMessage_Cpp
           + from_json(type_string, json): RosMessage_Cpp
           + from_json_to_message(type_info, json, data): void
           + serialize(type_string, json): SerializedMessage
           + deserialize(type_string, msg): json
           + get_schema(type_string): json
           + get_defaults(type_string): json
           + {static} yaml_to_json(yaml): json
           + {static} json_to_yaml(json): YAML::Node
       }

       class TypeCache <<singleton>> {
           + {static} instance(): TypeCache&
           + get(pkg, name): TypeInfo_Cpp*
           + get(type_string): TypeInfo_Cpp*
           + clear(): void
           - cache_: map<string, TypeInfo_Cpp*>
           - mutex_: shared_mutex
       }

       class ServiceActionTypes <<utility>> {
           + {static} get_request_type(srv_type): string
           + {static} get_response_type(srv_type): string
           + {static} get_goal_type(action_type): string
           + {static} get_result_type(action_type): string
           + {static} get_feedback_type(action_type): string
           + {static} is_service_type(type): bool
           + {static} is_action_type(type): bool
       }

       class SerializationError <<exception>> {
           + what(): string
       }

       class TypeNotFoundError <<exception>> {
           + what(): string
       }

       class JsonConversionError <<exception>> {
           + what(): string
       }
   }

   ' Relationships

   ' JsonSerializer uses TypeCache
   JsonSerializer --> TypeCache : uses

   ' JsonSerializer uses dynmsg
   JsonSerializer --> "dynmsg" : uses

   ' JsonSerializer converts between formats
   JsonSerializer ..> JSON : produces/consumes
   JsonSerializer ..> YAML : internal conversion
   JsonSerializer ..> SerializedMessage : produces/consumes

   ' TypeCache caches dynmsg type info
   TypeCache --> "dynmsg" : caches TypeInfo_Cpp

   ' Exception hierarchy
   TypeNotFoundError -up-|> SerializationError : extends
   JsonConversionError -up-|> SerializationError : extends

   ' JsonSerializer throws exceptions
   JsonSerializer ..> SerializationError : throws
   JsonSerializer ..> TypeNotFoundError : throws
   JsonSerializer ..> JsonConversionError : throws

   @enduml

Main Components
---------------

1. **JsonSerializer** - The main API for JSON ↔ ROS 2 message conversion
   - Stateless and thread-safe design
   - Provides ``serialize()`` / ``deserialize()`` for CDR format (used with GenericPublisher/GenericSubscription)
   - Provides ``to_json()`` / ``from_json()`` for in-memory message conversion
   - Uses dynmsg YAML bridge internally (JSON ↔ YAML ↔ ROS 2 message)
   - Generates JSON schemas and default values for any message type
   - Static utilities for YAML ↔ JSON conversion

2. **TypeCache** - Thread-safe singleton cache for type introspection data
   - Caches ``TypeInfo_Cpp*`` pointers from dynmsg
   - Uses ``std::shared_mutex`` for concurrent read access
   - Automatic cache miss loading via dynmsg ``ros_type_info_get()``
   - Key format: ``"package/msg/Type"`` or ``"package/srv/Type"``

3. **ServiceActionTypes** - Utility functions for service/action type manipulation
   - Derives request/response types from service type (e.g., ``std_srvs/srv/SetBool`` → ``std_srvs/srv/SetBool_Request``)
   - Derives goal/result/feedback types from action type
   - Type validation helpers (``is_service_type()``, ``is_action_type()``)

4. **Exception Classes** - Typed exceptions for error handling
   - ``SerializationError`` - Base class for all serialization errors
   - ``TypeNotFoundError`` - Message type not found in ROS 2 type system
   - ``JsonConversionError`` - JSON/YAML conversion failed

Data Flow
---------

Publishing (JSON → CDR)
~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: text

   JSON → json_to_yaml() → YAML::Node → dynmsg (set_from_yaml + serialize) → SerializedMessage → GenericPublisher

Subscribing (CDR → JSON)
~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: text

   GenericSubscription → SerializedMessage → dynmsg (deserialize + to_yaml) → YAML::Node → yaml_to_json() → JSON

Service Calls (JSON → CDR → JSON)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: text

   Request:  JSON → serialize() → SerializedMessage → GenericClient
   Response: GenericClient → SerializedMessage → deserialize() → JSON

Design Decisions
----------------

YAML Bridge
~~~~~~~~~~~

The package uses dynmsg's YAML interface rather than direct binary manipulation because:

1. **Stability**: dynmsg's YAML API is well-tested and handles complex nested types
2. **Type Safety**: YAML preserves type information during conversion
3. **Debugging**: YAML is human-readable for troubleshooting
4. **Performance**: Conversion overhead is negligible compared to network I/O

The JSON ↔ YAML conversion layer adds minimal overhead since both formats
have similar data models (objects, arrays, scalars).

Singleton TypeCache
~~~~~~~~~~~~~~~~~~~

The TypeCache uses the singleton pattern because:

1. **Memory Efficiency**: Type introspection data is shared across all serializers
2. **Thread Safety**: Single mutex protects all cache operations
3. **Lifetime Management**: Cache lives for the process duration, matching dynmsg's design

Thread Safety
~~~~~~~~~~~~~

- ``JsonSerializer`` is stateless and can be used from multiple threads
- ``TypeCache`` uses ``std::shared_mutex`` for concurrent reads
- All public methods are safe to call from any thread

Usage in Gateway
----------------

The serialization package is used by:

1. **DataAccessManager** - Publishing to topics via ``GenericPublisher``
2. **OperationManager** - Calling services/actions via ``GenericClient``
3. **NativeTopicSampler** - Deserializing messages from ``GenericSubscription``

Example: Publishing to a Topic
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: cpp

   JsonSerializer serializer;

   // Create publisher
   auto publisher = node->create_generic_publisher(
       "/cmd_vel", "geometry_msgs/msg/Twist", qos);

   // Serialize JSON to CDR
   nlohmann::json twist_json = {
       {"linear", {{"x", 1.0}, {"y", 0.0}, {"z", 0.0}}},
       {"angular", {{"x", 0.0}, {"y", 0.0}, {"z", 0.5}}}
   };
   auto serialized = serializer.serialize("geometry_msgs/msg/Twist", twist_json);

   // Publish
   publisher->publish(serialized);

Example: Deserializing from Subscription
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: cpp

   JsonSerializer serializer;

   auto callback = [&](std::shared_ptr<rclcpp::SerializedMessage> msg) {
       nlohmann::json json = serializer.deserialize(
           "sensor_msgs/msg/LaserScan", *msg);
       // Process json...
   };

   auto subscription = node->create_generic_subscription(
       "/scan", "sensor_msgs/msg/LaserScan", qos, callback);

