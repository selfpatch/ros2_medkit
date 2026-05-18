DTO Contract Layer
==================

This document describes the typed DTO contract layer introduced in the
ros2_medkit_gateway. It covers the problem it solves, the architecture of the
contract primitives, the three code-generation visitors, the OpenAPI schema
registry, and the workflow for adding new endpoints.

.. contents:: Table of Contents
   :local:
   :depth: 3

Overview
--------

Before this layer existed, a handler in the gateway had three independent
artefacts that described the same wire payload:

1. Hand-written ``nlohmann::json`` construction in the handler body.
2. A ``SchemaBuilder::*_schema()`` factory that produced the matching OpenAPI
   JSON Schema object.
3. An ``XMedkit`` fluent builder that assembled the ``x-medkit`` vendor
   extension block.

These three artefacts had no mechanical relationship. A field added to the
handler body had to be separately added to the schema factory and, if it
appeared in the ``x-medkit`` block, also to the fluent builder. Because the
compiler had no way to enforce the relationship, schemas and wire payloads
drifted silently. The OpenAPI spec served at ``/api/v1/docs`` described a
different shape than what the endpoint actually returned.

The DTO contract layer resolves this by making the C++ struct the single
source of truth. The same descriptor tuple that defines the struct is used
by three template visitors to produce the wire JSON, the OpenAPI schema, and
the request-body parser. Adding a field to the struct and its descriptor
automatically updates all three outputs.

Architecture
------------

The contract is implemented entirely as header-only templates in
``include/ros2_medkit_gateway/dto/``. No virtual dispatch, no runtime type
erasure, and no separate code-generation step are needed.

.. plantuml::
   :caption: DTO Contract Layer - Component Relationships

   @startuml dto_contract_overview

   skinparam linetype ortho
   skinparam classAttributeIconSize 0

   package "dto/" {
       class "contract.hpp" as contract {
           Field<Class, Member>
           dto_fields<T> constexpr tuple
           dto_name<T> string_view
           is_dto_v<T> bool
           for_each_field<T>(visitor)
       }

       class JsonWriter<T> {
           + write(obj: T): json
       }

       class SchemaWriter<T> {
           + schema(): json
       }

       class JsonReader<T> {
           + read(j: json): expected<T, vector<FieldError>>
       }

       class "registry.hpp" as registry {
           AllDtos tuple
           collect_component_schemas(): json
       }
   }

   package "http/handlers/" {
       class HandlerContext {
           + send_dto<T>(res, dto)
           + parse_body<T>(req, res): optional<T>
       }
   }

   package "openapi/" {
       class OpenApiSpecBuilder {
           + build(): json
       }
   }

   JsonWriter .up.|> contract : folds over dto_fields
   SchemaWriter .up.|> contract : folds over dto_fields
   JsonReader .up.|> contract : folds over dto_fields

   HandlerContext --> JsonWriter : send_dto
   HandlerContext --> JsonReader : parse_body
   OpenApiSpecBuilder --> registry : collect_component_schemas
   registry --> SchemaWriter : per DTO in AllDtos

   @enduml

Field Descriptor (``Field<C, M>``)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Each field in a DTO is described by a ``Field<Class, Member>`` aggregate
defined in ``contract.hpp``:

.. code-block:: cpp

   template <class Class, class Member>
   struct Field {
     std::string_view key;          // JSON wire key
     Member Class::*ptr;            // pointer-to-member
     Presence presence;             // kRequired or kOptional
     std::string_view description;  // OpenAPI property description
     const std::string_view * enum_values;  // allowed string values (or nullptr)
     std::size_t enum_count;
   };

Fields are never constructed directly. The ``field()`` and ``field_enum()``
factory functions deduce the class and member types from the pointer-to-member
argument:

.. code-block:: cpp

   // Required string field
   field("fault_code", &FaultListItem::fault_code)

   // Optional field (presence deduced from std::optional<> member type)
   field("description", &FaultListItem::description)

   // Enum-constrained field with inline constexpr string_view array
   field_enum("status", &FaultStatus::aggregated_status, kFaultAggregatedStatusValues)

``dto_fields<T>`` - the Descriptor Tuple
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The descriptor tuple for a type ``T`` is a ``constexpr`` specialization of
the variable template ``dto_fields<T>``:

.. code-block:: cpp

   template <>
   inline constexpr auto dto_fields<FaultListItem> = std::make_tuple(
       field("fault_code", &FaultListItem::fault_code),
       field("severity",   &FaultListItem::severity),
       field("description",&FaultListItem::description),
       field("status",     &FaultListItem::status));

The primary template is a sentinel type (``detail::not_a_dto<T>``). The
``is_dto_v<T>`` trait returns ``true`` only when a specialization exists,
which gates all three visitors at compile time.

**Placement rule:** every ``dto_fields<X>`` and ``dto_name<X>``
specialization must appear in the same header as the struct declaration. A
translation unit that instantiates a visitor before seeing the specialization
silently binds the sentinel, producing a latent ODR-adjacent bug.

``dto_name<T>`` - Schema Registry Key
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Each DTO names itself in ``components/schemas`` via a ``constexpr string_view``
specialization:

.. code-block:: cpp

   template <>
   inline constexpr std::string_view dto_name<FaultListItem> = "FaultListItem";

The name is used by ``SchemaWriter`` when emitting ``$ref`` cross-references
and by ``collect_component_schemas()`` when populating the OpenAPI registry.

The Three Visitors
~~~~~~~~~~~~~~~~~~

All three visitors fold over ``dto_fields<T>`` using ``for_each_field<T>()``,
which calls ``std::apply`` over the constexpr tuple. The fold is entirely at
compile time; no runtime reflection is involved.

**JsonWriter** (``json_writer.hpp``)

Serializes a DTO instance to a ``nlohmann::json`` object. Optional members
that have no value are omitted from the output. Nested DTO members are
recursively serialized. ``std::vector`` members become JSON arrays.
``std::variant`` members are serialized as the active alternative.
``nlohmann::json`` members pass through unchanged.

**SchemaWriter** (``schema_writer.hpp``)

Generates the OpenAPI 3.1 ``components/schemas`` entry for a type. Each
field maps to a JSON Schema property. Required fields are listed in the
``required`` array. Nested DTO types become ``$ref`` entries pointing to
the named schema. Optional wrapper types are unwrapped before schema
generation. Enum-constrained string fields include an ``enum`` array.

**JsonReader** (``json_reader.hpp``)

Parses and validates a ``nlohmann::json`` object into a DTO instance.
Collects all field-level errors rather than short-circuiting on the first
failure, returning ``tl::expected<T, std::vector<FieldError>>`` on
completion. Required fields missing or null produce a ``FieldError``.
Unknown extra fields in the input are silently ignored (lenient parsing).
Enum-constrained string fields are validated against the allowed set after
decoding.

.. plantuml::
   :caption: Request Lifecycle with DTO Contract

   @startuml dto_request_lifecycle

   participant Client
   participant Handler
   participant HandlerContext as ctx
   participant JsonReader
   participant JsonWriter

   == Request body parsing ==

   Client -> Handler : POST /api/v1/.../executions\n{...JSON body...}
   Handler -> ctx : parse_body<ExecutionUpdateRequest>(req, res)
   ctx -> JsonReader : read(body_json)
   JsonReader -> JsonReader : fold over dto_fields<ExecutionUpdateRequest>
   JsonReader --> ctx : expected<ExecutionUpdateRequest, vector<FieldError>>
   alt validation failed
       ctx --> Client : 400 GenericError (field errors joined)
   else validation ok
       ctx --> Handler : ExecutionUpdateRequest dto
   end

   == Response serialization ==

   Handler -> Handler : build OperationExecution dto
   Handler -> ctx : send_dto<OperationExecution>(res, dto)
   ctx -> JsonWriter : write(dto)
   JsonWriter -> JsonWriter : fold over dto_fields<OperationExecution>
   JsonWriter --> ctx : nlohmann::json object
   ctx --> Client : 200 OK + JSON response

   @enduml

AllDtos Registry (``registry.hpp``)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``AllDtos`` is a single ``std::tuple`` listing every named DTO type:

.. code-block:: cpp

   using AllDtos = std::tuple<
       GenericError,
       AreaListItem, AreaDetail,
       FaultListItem, FaultDetail, FaultStatus,
       OperationItem, OperationExecution,
       // ... all other domain DTOs ...
   >;

The free function ``collect_component_schemas()`` iterates ``AllDtos`` at
compile time (via ``std::index_sequence``) and calls
``SchemaWriter<T>::schema()`` for each type, producing the bulk of the
``components/schemas`` map. ``SchemaBuilder::component_schemas()`` (in
``src/openapi/schema_builder.cpp``) merges these DTO-generated entries with a
small number of explicitly hand-written survivors:

- **``OperationExecutionList``** - a thin ``items``-wrapper referencing
  ``OperationExecution``. No dedicated DTO type exists for this list wrapper,
  so it is assembled manually with ``items_wrapper_ref("OperationExecution")``.
- **``from_ros_msg`` / ``from_ros_srv_request`` / ``from_ros_srv_response``** -
  schema factories for dynamic ROS 2 payloads whose field names are not known
  at compile time (topic samples, service request/response bodies).
- **``binary_schema``** and **``generic_object_schema``** - trivial inline
  schema objects used for bulk-data and free-form fields.

With the exception of these survivors, every schema in ``components/schemas``
is generated from ``AllDtos``. No runtime loop over a dynamic registry is
required.

The ``Collection<T>`` template is a generic DTO for paginated list responses
(``{"items": [...]}``). It is specialized for each element type in
``AllDtos`` and given a name like ``"FaultList"`` via a ``dto_name``
specialization.

Escape Hatches
--------------

Not every payload can be expressed as a typed DTO. Two categories are
intentionally kept as raw ``nlohmann::json``:

Genuinely Dynamic Payloads
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Some fields carry data whose schema is not known at compile time:

- **Live ROS 2 message data** - topic samples returned by the data handlers.
  The message type and field names depend on the runtime ROS 2 graph.
- **Free-form metadata** - the ``extended_data_records`` and ``snapshots``
  fields in ``FaultEnvironmentData`` carry fault snapshot data whose shape
  is determined by the fault reporter plugin, not the gateway.
- **OpenAPI spec blobs** - the ``/docs`` handler itself returns raw JSON
  assembled by ``OpenApiSpecBuilder``.
- **Fan-out merged responses** - aggregation responses assembled by merging
  ``items`` arrays from multiple peer gateways.

These fields are typed as ``std::optional<nlohmann::json>`` or
``nlohmann::json`` members inside the enclosing DTO. ``JsonWriter`` passes
them through without transformation; ``SchemaWriter`` emits an empty object
schema (``{}``, meaning "any JSON value") for them. Handlers that return
these payloads use ``ctx_.send_json(res, payload)`` instead of
``ctx_.send_dto<T>(res, dto)``.

Non-DTO Routes
~~~~~~~~~~~~~~

Three categories of routes bypass the DTO layer entirely:

- **Binary download / multipart upload** - ``GET /.../bulk-data/...`` and
  ``POST /.../bulk-data/...`` use ``set_chunked_content_provider`` with
  binary content types. There is no JSON to serialize or validate.
- **SSE streams** - cyclic subscription and fault subscription endpoints
  push newline-delimited ``data:`` frames over a long-lived connection.
  The stream framing is handled by the SSE transport, not by a DTO.
- **Plugin-owned vendor routes** - routes registered by ``GatewayPlugin``
  subclasses via ``PluginContext::register_route()`` are fully controlled
  by the plugin. The plugin may use ``send_dto`` internally but it is not
  required to.

Adding a New DTO
----------------

Follow these four steps when introducing a new typed payload:

1. **Define the struct and its descriptor** in the appropriate domain header
   under ``include/ros2_medkit_gateway/dto/``. Add a ``dto_fields<T>``
   specialization and a ``dto_name<T>`` specialization in the same header.

   .. code-block:: cpp

      // In dto/my_domain.hpp
      struct MyResponse {
        std::string id;
        std::optional<std::string> label;
        int64_t count{0};
      };

      template <>
      inline constexpr auto dto_fields<MyResponse> = std::make_tuple(
          field("id",    &MyResponse::id),
          field("label", &MyResponse::label),
          field("count", &MyResponse::count));

      template <>
      inline constexpr std::string_view dto_name<MyResponse> = "MyResponse";

2. **Register in AllDtos** by adding ``MyResponse`` to the tuple in
   ``include/ros2_medkit_gateway/dto/registry.hpp``. Also add the include
   for ``dto/my_domain.hpp`` at the top of ``registry.hpp``.

3. **Use in the handler** via ``HandlerContext``:

   .. code-block:: cpp

      // GET handler - typed response
      void MyHandlers::handle_get(const httplib::Request & req, httplib::Response & res) {
        auto entity = ctx_.validate_entity_for_route(req, res, req.matches[1].str());
        if (!entity) { return; }

        dto::MyResponse resp;
        resp.id    = entity->id;
        resp.label = "example";
        resp.count = 42;
        ctx_.send_dto(res, resp);
      }

      // POST handler - typed request body
      void MyHandlers::handle_post(const httplib::Request & req, httplib::Response & res) {
        auto body = ctx_.parse_body<dto::MyCreateRequest>(req, res);
        if (!body) { return; }  // 400 already sent

        // use body->field_name
      }

4. **Wire the schema into the route description.** Because ``MyResponse`` is
   now in ``AllDtos``, ``collect_component_schemas()`` automatically includes
   its schema in every ``/docs`` response. You still need to reference it in
   the route's OpenAPI metadata so the spec shows the correct ``$ref``:

   - For **built-in gateway routes**, add a ``.response()`` (or
     ``.request_body()``) call in ``rest_server.cpp::setup_routes()`` when
     registering the route:

     .. code-block:: cpp

        reg.get("/my-entity/{id}/my-resource",
                [this](auto & req, auto & res) { /* handler */ })
            .tag("MyTag")
            .summary("Get my resource")
            .response(200, "Resource detail", SB::ref("MyResponse"))
            .operation_id("getMyResource");

   - For routes whose path items are assembled by ``PathBuilder`` or
     ``CapabilityGenerator`` (entity-scoped resource collections such as
     ``/data``, ``/operations``, ``/faults``), the ``$ref`` is embedded in
     the corresponding ``build_*`` method in
     ``src/openapi/path_builder.cpp``.

   - For **plugin-contributed routes**, use the ``RouteDescriptionBuilder``
     API in ``core/openapi/route_descriptions.hpp`` (the builder classes
     there are intended for plugin use, not for core built-in routes).

Adding a New Endpoint (Full Checklist)
--------------------------------------

A new endpoint with a typed payload follows the standard gateway handler
checklist plus the DTO steps above:

1. Define DTO struct + ``dto_fields`` + ``dto_name`` in a domain header.
2. Add to ``AllDtos`` in ``registry.hpp``.
3. Implement handler in ``src/http/handlers/``.
4. Register route in ``rest_server.cpp::setup_routes()`` (dual-path pattern
   for entity types that share the same route shape).
5. Update ``handle_root`` endpoint list in ``health_handlers.cpp`` to mirror
   the new route.
6. Add URI field to entity detail response if the new route is a resource
   collection.
7. Write a unit test using ``JsonWriter<T>::write()`` and
   ``JsonReader<T>::read()`` directly - no HTTP server needed.
8. Write an integration test that calls the live endpoint.

Known Limitations
-----------------

The ``Collection<T>`` list-wrapper DTO hardcodes its ``x-medkit`` member as the
generic ``XMedkitCollection`` type. The entity list endpoints (areas, components,
apps, functions) emit exactly that struct, so their generated schemas are
accurate. The fault, config, data, and log list endpoints, however, emit a
richer domain-specific collection x-medkit (``FaultListXMedkit``,
``FaultListAggXMedkit``, ``ConfigListXMedkit``, ``DataListXMedkit``,
``LogListXMedkit`` - carrying aggregation counts, peer provenance, and similar
metadata). Those domain structs are defined and registered in ``AllDtos``, but
the ``FaultList`` / ``FaultListAgg`` / ``ConfigList`` / ``DataList`` / ``LogList``
response schemas still type their ``x-medkit`` property as
``XMedkitCollection``, so a generated client does not see the richer
fields from the schema alone. The wire payload remains a valid instance of the
published schema (JSON Schema allows additional properties by default), so this
is a schema-precision gap, not a contract violation. A future refinement could
parameterize ``Collection<T>`` over its x-medkit type (for example,
``Collection<T, XMedkitT>``) so that each list endpoint's schema references its
real collection x-medkit struct.

Key Files
---------

``include/ros2_medkit_gateway/dto/contract.hpp``
    ``Field``, ``dto_fields``, ``dto_name``, ``is_dto_v``,
    ``for_each_field`` - the contract primitives.

``include/ros2_medkit_gateway/dto/json_writer.hpp``
    ``JsonWriter<T>`` - struct to JSON serialization.

``include/ros2_medkit_gateway/dto/schema_writer.hpp``
    ``SchemaWriter<T>`` and ``schema_of<U>`` - type to OpenAPI schema.

``include/ros2_medkit_gateway/dto/json_reader.hpp``
    ``JsonReader<T>`` and ``FieldError`` - JSON to struct with validation.

``include/ros2_medkit_gateway/dto/registry.hpp``
    ``AllDtos`` tuple and ``collect_component_schemas()``.

``include/ros2_medkit_gateway/http/handlers/handler_context.hpp``
    ``HandlerContext::send_dto<T>()`` and ``HandlerContext::parse_body<T>()``.

Domain headers
    ``dto/errors.hpp``, ``dto/entities.hpp``, ``dto/faults.hpp``,
    ``dto/operations.hpp``, ``dto/config.hpp``, ``dto/locks.hpp``,
    ``dto/triggers.hpp``, ``dto/logs.hpp``, ``dto/scripts.hpp``,
    ``dto/updates.hpp``, ``dto/auth.hpp``, ``dto/health.hpp``,
    ``dto/bulkdata.hpp``, ``dto/cyclic_subscriptions.hpp``,
    ``dto/data.hpp``, ``dto/x_medkit.hpp`` - per-domain struct definitions
    with co-located ``dto_fields`` and ``dto_name`` specializations.
    ``dto/errors.hpp`` holds ``GenericError``, the error response DTO used
    by every endpoint.

``dto/enums.hpp``
    Enum-vocabulary header. Contains the ``constexpr string_view`` arrays
    (``kFaultSeverityLabelValues``, ``kOperationExecutionStatusValues``, etc.)
    referenced by ``field_enum()`` descriptors in the domain headers. Does
    not define any DTO structs.
