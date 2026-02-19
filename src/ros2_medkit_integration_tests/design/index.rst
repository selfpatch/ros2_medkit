ros2_medkit_integration_tests
==============================

This section contains design documentation for the ros2_medkit_integration_tests package.

Overview
--------

The integration tests package validates the ros2_medkit system end-to-end by launching
real ROS 2 nodes and exercising the gateway's HTTP API. Tests are split into two categories:

- **Feature tests** validate individual API capabilities in isolation
- **Scenario tests** exercise multi-step workflows that cross multiple features

A shared Python library (``ros2_medkit_test_utils``) provides a base test case,
launch description factories, and assertion helpers.

Architecture
------------

The following diagram shows the test infrastructure and how it relates to the
system under test.

.. plantuml::
   :caption: Integration Test Architecture

   @startuml integration_test_architecture

   skinparam linetype ortho
   skinparam classAttributeIconSize 0

   title Integration Tests - Architecture

   package "Test Infrastructure" {
       class GatewayTestCase {
           +BASE_URL: str
           +MIN_EXPECTED_APPS: int
           +REQUIRED_APPS: set
           +REQUIRED_AREAS: set
           --
           +setUp(): wait for discovery
           +get_json(path): dict
           +delete_request(path): Response
           +assert_entity_exists()
           +wait_for_fault()
           +wait_for_operation()
           +create_execution()
       }

       class "launch_helpers" {
           +DEMO_NODE_REGISTRY: dict
           +create_test_launch(): (LaunchDescription, dict)
           +create_gateway_node(): Node
           +create_fault_manager_node(): Node
           +create_demo_nodes(): list[Node]
       }
   }

   package "Feature Tests" {
       class "test_data_read" as tdr
       class "test_operations_api" as top
       class "test_faults_api" as tfa
       class "..." as etc1
   }

   package "Scenario Tests" {
       class "test_scenario_action_lifecycle" as tsal
       class "test_scenario_fault_lifecycle" as tsfl
       class "test_scenario_discovery_hybrid" as tsdh
       class "..." as etc2
   }

   package "System Under Test" {
       class "gateway_node\n(HTTP API)" as gw
       class "fault_manager_node" as fm
   }

   package "Demo Nodes" {
       class "temp_sensor" as ts
       class "lidar_sensor" as ls
       class "long_calibration" as lc
       class "..." as etc3
   }

   tdr -down-|> GatewayTestCase
   top -down-|> GatewayTestCase
   tfa -down-|> GatewayTestCase
   tsal -down-|> GatewayTestCase
   tsfl -down-|> GatewayTestCase
   tsdh -down-|> GatewayTestCase

   GatewayTestCase ..> gw : HTTP requests
   "launch_helpers" ..> gw : launches
   "launch_helpers" ..> fm : launches
   "launch_helpers" ..> ts : launches
   "launch_helpers" ..> ls : launches
   "launch_helpers" ..> lc : launches

   gw <..> ts : ROS 2 topics
   gw <..> ls : ROS 2 topics
   gw <..> lc : ROS 2 actions
   fm <.. ls : fault reports

   @enduml

Test Categories
---------------

Feature Tests
~~~~~~~~~~~~~

Feature tests (``test/features/``) validate individual API capabilities. Each test file
focuses on one area of functionality and launches only the nodes it needs.

.. list-table:: Feature Tests
   :header-rows: 1
   :widths: 30 70

   * - Test File
     - Description
   * - ``test_data_read``
     - GET /data endpoints for topic sampling
   * - ``test_data_write``
     - PUT /data endpoints for publishing to topics
   * - ``test_operations_api``
     - Service and action operation discovery
   * - ``test_faults_api``
     - Fault listing, filtering, and deletion
   * - ``test_entity_listing``
     - Entity collection endpoints (areas, components, apps)
   * - ``test_entity_routing``
     - Nested entity routing (areas/components/apps)
   * - ``test_hateoas``
     - HATEOAS links and capabilities
   * - ``test_configuration_api``
     - ROS 2 parameter read/write via REST
   * - ``test_sse``
     - Server-Sent Events fault stream
   * - ``test_health``
     - Health check endpoint
   * - ``test_cors``
     - CORS header handling
   * - ``test_auth``
     - JWT authentication and RBAC
   * - ``test_tls``
     - HTTPS/TLS endpoint
   * - ``test_bulk_data_api``
     - Bulk data upload/download
   * - ``test_snapshots_api``
     - Rosbag snapshot capture
   * - ``test_discovery_heuristic``
     - Heuristic app naming from node graph

Scenario Tests
~~~~~~~~~~~~~~

Scenario tests (``test/scenarios/``) exercise end-to-end workflows. Tests within a
scenario are numbered (``test_01_``, ``test_02_``, ...) and execute in order.

.. list-table:: Scenario Tests
   :header-rows: 1
   :widths: 30 70

   * - Test File
     - Description
   * - ``test_scenario_action_lifecycle``
     - Create, poll, cancel, list action executions
   * - ``test_scenario_fault_lifecycle``
     - Fault appearance, bulk delete, single delete
   * - ``test_scenario_fault_inspection``
     - Fault details, snapshots, rosbag download
   * - ``test_scenario_discovery_hybrid``
     - Hybrid discovery mode with manifest + runtime
   * - ``test_scenario_discovery_manifest``
     - Manifest-only discovery mode validation
   * - ``test_scenario_config_management``
     - Parameter read/write/restore workflow
   * - ``test_scenario_data_publish_verify``
     - Publish data and verify topic updates
   * - ``test_scenario_subscriptions``
     - Cyclic subscription CRUD and SSE delivery
   * - ``test_scenario_bulk_data_upload``
     - Upload file, list, download, delete
   * - ``test_scenario_bulk_data_download``
     - Rosbag snapshot download workflow

Demo Nodes
----------

Nine automotive-themed C++ demo nodes simulate a vehicle system. They are launched
by the test infrastructure and also available for manual testing via
``ros2 launch ros2_medkit_integration_tests demo_nodes.launch.py``.

Nodes use standard ROS 2 patterns: publishers with wall timers, service servers,
and action servers. All timer-based nodes cancel their timers in the destructor
to ensure clean shutdown.

Shared Test Library
-------------------

``ros2_medkit_test_utils`` is a Python package installed alongside the tests:

- **GatewayTestCase** -- ``unittest.TestCase`` subclass that waits for the gateway
  to discover expected apps/areas before running tests, provides HTTP helper methods.
- **launch_helpers** -- Factory functions (``create_test_launch``, ``create_gateway_node``,
  ``create_demo_nodes``) that replace ~500 lines of inline launch boilerplate.
- **constants** -- Default port, timeout values.
- **coverage** -- ``GCOV_PREFIX`` environment helpers for CI code coverage collection.

Design Decisions
----------------

Test Split: Features vs Scenarios
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Tests are split into features (isolated, parallel-safe) and scenarios (ordered,
workflow-driven) to enable faster CI runs for focused feature validation while
still exercising realistic end-to-end workflows.

Shared Launch Helpers
~~~~~~~~~~~~~~~~~~~~~

The ``create_test_launch()`` factory replaced ~500 lines of duplicated launch
description boilerplate across 24+ test files. Each test file now has a 3-5 line
``generate_test_description()`` function.

Self-Contained Scenario Steps
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Each scenario test step creates its own resources (goals, faults, subscriptions)
rather than sharing state between tests. This eliminates race conditions like the
concurrent-goal issue in the original monolithic test file (GitHub #222).
