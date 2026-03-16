# DiscoveryHandlers Unit Test Design

**Date:** 2026-03-16
**Issue:** `#179` Add unit tests for `DiscoveryHandlers`

## Goal

Add dedicated unit coverage for all 16 methods in `DiscoveryHandlers` without introducing a large new mocking layer.

## Context

`DiscoveryHandlers` is the largest REST handler in `ros2_medkit_gateway`. Unlike simpler handlers, it reads from both:

- `GatewayNode::get_thread_safe_cache()` for `list_*` endpoints
- `GatewayNode::get_discovery_manager()` for most `get_*` and relationship endpoints

That split means cache-only tests are not sufficient, and a pure mock-based design would require substantial new infrastructure that the current test suite does not use.

## Chosen Approach

Use a hybrid unit-test approach:

1. A lightweight validation fixture with `HandlerContext(nullptr, ...)`
   - Covers request-shape failures such as missing route matches
   - Covers invalid entity ID failures that return before deeper node access

2. A stateful fixture with a real `GatewayNode`
   - Initializes `DiscoveryManager` in `MANIFEST_ONLY` mode with a compact test manifest
   - Seeds `ThreadSafeEntityCache` from the discovered manifest entities so cache-backed and discovery-backed handlers read the same dataset
   - Calls handler methods directly, not through the HTTP server, to keep the tests unit-focused

This keeps the tests close to real behavior while avoiding premature generalization into a cross-handler mocking framework.

## Test Data Model

Use one compact manifest graph that exercises all four entity families and their relationships:

- Areas
  - `vehicle`
  - `sensors` with `parent_area: vehicle`
- Components
  - `main_ecu` in `vehicle`
  - `lidar_unit` in `sensors`, `parent_component_id: main_ecu`
  - `main_ecu` depends on `lidar_unit` and `ghost_component`
- Apps
  - `planner` on `main_ecu`
  - `mapper` on `lidar_unit`, depends on `planner` and `ghost_app`
- Functions
  - `navigation` hosted by `planner`
  - `perception` hosted by `mapper`

Runtime-only fields that improve response assertions can be seeded after discovery:

- `planner.is_online = true`
- `planner.bound_fqn = "/vehicle/main_ecu/planner"`
- `mapper.is_online = false`
- `mapper.bound_fqn = "/sensors/lidar_unit/mapper"`

## Test Structure

Create `src/ros2_medkit_gateway/test/test_discovery_handlers.cpp` with four logical sections:

- Areas
- Components
- Apps
- Functions

Use two fixture classes:

- `DiscoveryHandlersValidationTest`
- `DiscoveryHandlersFixtureTest`

Shared helpers:

- `write_temp_manifest(...)`
- `make_request_with_matches(...)`
- `parse_json(...)`
- `seed_manifest_and_cache(...)`

## Minimum Coverage Matrix

Each handler gets at least one representative behavior test, with extra tests where the method has meaningful validation or relationship behavior.

### Areas

- `handle_list_areas`
  - Returns seeded areas with `href`, `x-medkit`, and `total_count`
- `handle_get_area`
  - Missing matches returns `400`
  - Invalid ID returns `400`
  - Unknown ID returns `404`
  - Known ID returns links and capabilities
- `handle_area_components`
  - Unknown area returns `404`
  - Known area returns only matching components
- `handle_get_subareas`
  - Invalid ID returns `400`
  - Known parent returns child areas and `_links`
- `handle_get_contains`
  - Unknown area returns `404`
  - Known area returns contained components

### Components

- `handle_list_components`
  - Returns seeded components with `description`, `tags`, and `x-medkit.source`
- `handle_get_component`
  - Invalid ID returns `400`
  - Known component returns `belongs-to`, `subcomponents`, `hosts`, optional `depends-on`, and capabilities
- `handle_get_subcomponents`
  - Unknown component returns `404`
  - Known parent returns child components
- `handle_get_hosts`
  - Unknown component returns `404`
  - Known component returns hosted apps with runtime metadata
- `handle_component_depends_on`
  - Unknown component returns `404`
  - Known component returns both resolved and missing dependencies

### Apps

- `handle_list_apps`
  - Returns seeded apps with `component_id`, `bound_fqn`, and `is_online`
- `handle_get_app`
  - Invalid ID returns `400`
  - Known app returns `is-located-on`, optional `depends-on`, links, and capabilities
- `handle_app_depends_on`
  - Unknown app returns `404`
  - Known app returns both resolved and missing dependencies

### Functions

- `handle_list_functions`
  - Returns seeded functions with `description`, `tags`, and `x-medkit.source`
- `handle_get_function`
  - Invalid ID returns `400`
  - Known function returns `hosts`, `x-medkit-graph`, links, and capabilities
- `handle_function_hosts`
  - Unknown function returns `404`
  - Known function returns hosting apps and `_links`

## Why This Scope

- Matches the issue requirement to cover all 16 methods
- Reuses patterns already established by `test_health_handlers.cpp`, `test_auth_handlers.cpp`, and `test_data_handlers.cpp`
- Leaves room to extract a shared helper later if `OperationHandlers` or `ConfigHandlers` need the same fixture, without forcing that abstraction before it proves useful

## Verification Plan

During implementation, verify the work with:

```bash
colcon test --packages-select ros2_medkit_gateway --ctest-args -R test_discovery_handlers --output-on-failure
colcon test-result --verbose
```
