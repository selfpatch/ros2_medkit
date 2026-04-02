Glossary
========

This glossary defines key terms used throughout ros2_medkit documentation.

.. glossary::
   :sorted:

   Area
      A logical or physical grouping of components, representing a vehicle
      subsystem or domain. Examples: ``powertrain``, ``chassis``, ``body``.
      Areas are manifest-defined only and are never auto-generated in
      runtime mode.

      See: :doc:`tutorials/manifest-discovery`

   App
      A software application, typically mapping to a single ROS 2 node.
      Apps are hosted on Components and can participate in Functions.
      Available in all discovery modes.

      See: :doc:`tutorials/heuristic-apps`

   Component
      A hardware or virtual unit that hosts Apps. In runtime-only mode,
      a single host-level Component is created from ``HostInfoProvider``.
      In manifest mode, components are explicitly defined.

   Configuration
      A ROS 2 node parameter exposed via the ``/configurations`` endpoint.
      Configurations can be read, modified, and reset to defaults.

      See: :doc:`api/rest`

   Data
      Topic data from ROS 2 publishers, exposed via the ``/data`` endpoint.
      Data can be read (sampled) and written (published).

   Discovery Mode
      The method used to map ROS 2 graph entities to SOVD entities:

      - **runtime_only**: ROS 2 graph introspection (default)
      - **hybrid**: Manifest + runtime linking (recommended)
      - **manifest_only**: Only manifest-declared entities

      See: :doc:`config/discovery-options`

   Entity
      A generic term for any SOVD object: Area, Component, App, or Function.
      Each entity has a unique ID, name, and set of resource collections.

   Execution
      An instance of an operation being called. For services, executions
      complete immediately. For actions, executions are asynchronous and
      can be polled for status or cancelled.

   Fault
      An error condition reported by a ROS 2 node to the fault manager.
      Faults have a code, severity, message, and timestamp.

      See: :doc:`design/ros2_medkit_fault_reporter/index`

   Function
      A high-level capability that spans multiple Apps. Functions aggregate
      data and operations from their host Apps. In runtime mode, Functions
      are created from the first namespace segment. In manifest/hybrid mode,
      Functions are explicitly defined.

   Gateway
      The ros2_medkit_gateway node that provides the REST API. It discovers
      ROS 2 entities, handles HTTP requests, and manages executions.

   Manifest
      A YAML file that declares the system structure (areas, components,
      apps, functions) with stable IDs and semantic metadata.

      See: :doc:`config/manifest-schema`

   MCP (Model Context Protocol)
      A protocol for connecting LLMs to external tools. ros2_medkit_mcp
      provides MCP tools that wrap the gateway REST API.

      See: :doc:`tutorials/mcp-server`

   Operation
      A callable action or service exposed via the ``/operations`` endpoint.
      Operations are invoked by creating executions.

   Resource Collection
      A category of resources available on an entity: ``data``, ``operations``,
      ``configurations``, or ``faults``. Not all entities support all collections.

   Runtime Linking
      In hybrid mode, the process of matching manifest-declared Apps to
      running ROS 2 nodes based on their ``ros_binding`` configuration.

   SOVD
      Service-Oriented Vehicle Diagnostics — an ASAM standard for
      automotive diagnostics over IP networks. ros2_medkit implements
      a subset of SOVD adapted for ROS 2 systems.

   Synthetic Component
      (Removed) Previously, Components were automatically created in
      runtime-only mode by grouping ROS 2 nodes by namespace. This
      behavior has been removed. Components now come from
      ``HostInfoProvider`` (single host-level Component) or manifest.
