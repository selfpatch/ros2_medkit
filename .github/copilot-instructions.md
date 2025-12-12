# Copilot Instructions for ros2_medkit

## Project Overview

**ros2_medkit** is a ROS 2 diagnostics gateway that exposes ROS 2 system information via a RESTful HTTP API. It models robots as a **diagnostic entity tree** (Area → Component → Function → App) aligned with the SOVD (Service-Oriented Vehicle Diagnostics) specification.

**Tech Stack**: C++23, ROS 2 Jazzy, Ubuntu 24.04

## Architecture

```
GatewayNode (src/ros2_medkit_gateway/src/gateway_node.cpp)
├── DiscoveryManager    - Discovers ROS 2 nodes, services, actions; organizes into Areas/Components
├── DataAccessManager   - Topic sampling and publishing via native rclcpp + CLI fallback
├── OperationManager    - Service calls and action goal handling
├── ConfigurationManager- ROS 2 parameter CRUD operations
└── RESTServer          - HTTP endpoints using cpp-httplib (runs in separate thread)
```

**Key domain model** (see [models.hpp](src/ros2_medkit_gateway/include/ros2_medkit_gateway/models.hpp)):
- **Area**: Top-level namespace grouping (e.g., `/powertrain`, `/chassis`, `/body`, `root`)
- **Component**: Individual ROS 2 node with its topics, services, and actions
- **EntityCache**: Thread-safe in-memory cache of discovered Areas/Components

## Code Style & Conventions

- **C++ Standard**: C++23 with `-Wall -Wextra -Wpedantic -Wshadow -Wconversion`
- **ROS 2 Distribution**: Jazzy (Ubuntu 24.04)
- **Formatting**: Google-based clang-format with 120 column limit, 2-space indent
- **Pointer style**: Middle alignment (`Type * ptr`) per ROS 2 conventions
- **Namespace**: All gateway code lives in `ros2_medkit_gateway` namespace
- **Headers**: Include guards use `#pragma once`
- **Copyright**: Apache 2.0 header required on all source files

## REST API Pattern

All endpoints are versioned under `/api/v1`. Route handlers are in [rest_server.cpp](src/ros2_medkit_gateway/src/rest_server.cpp).

```cpp
// Pattern for adding a new endpoint:
server_->Get(api_path("/your-endpoint").c_str(),
    [this](const httplib::Request & req, httplib::Response & res) {
      handle_your_endpoint(req, res);
    });
```

## Testing Patterns

- **Unit tests**: GTest-based, see `test/test_gateway_node.cpp`, `test/test_operation_manager.cpp`
- **Integration tests**: Python launch tests using `launch_testing`, see `test/test_integration.test.py`
- **Demo nodes**: Automotive-themed test fixtures in `test/demo_nodes/` for integration testing
- **Requirements traceability**: Use `// @verifies REQ_XXX` comments in tests to link to requirements in docs

## Test Coverage Requirements

**All new features must have corresponding tests.** Coverage is tracked via Codecov and enforced in CI:
- Unit tests cover individual manager classes and utilities
- Integration tests verify end-to-end REST API behavior with demo nodes
- Coverage reports are generated on every PR and uploaded to Codecov on main branch merges

When adding a new endpoint or feature:
1. Add unit tests for the underlying logic
2. Add integration tests for the REST API endpoint
3. For spec-compliant endpoints, include `// @verifies REQ_XXX` comments linking to requirements

## Key Dependencies

- **ROS 2 Jazzy** (Ubuntu 24.04)
- **cpp-httplib**: HTTP server (found via pkg-config)
- **nlohmann_json**: JSON serialization
- **yaml-cpp**: Configuration parsing

## Documentation

Sphinx-based docs in `docs/`. Uses sphinx-needs for requirements traceability.

- `docs/design/`: Architecture and design decisions (PlantUML, RST)
- `src/ros2_medkit_gateway/design/`: Component-specific design docs (PlantUML, RST)
- `docs/requirements/`: SOVD specs and project requirements
- `postman/`: API collections for manual testing
- `src/ros2_medkit_gateway/README.md`: Package-level documentation

```bash
# Build docs (uses .venv)
.venv/bin/sphinx-build -b html docs docs/_build/html

# Serve with auto-reload
.venv/bin/sphinx-autobuild --host 0.0.0.0 --port 8000 docs docs/_build/html
```

**Keep documentation up to date**: When adding or modifying endpoints, update the corresponding docs in `docs/`, `src/ros2_medkit_gateway/design/`, `src/ros2_medkit_gateway/README.md`, and `postman/` collections. Ensure tests use `// @verifies REQ_XXX` comments to maintain traceability between requirements, implementation, and tests.

## Design Principles

- **Modularity & Separation of Concerns**: Separate core business logic from ROS 2 infrastructure and HTTP handling. Logic should be testable in isolation.
- **Dependency Injection**: Explicitly pass dependencies to components to facilitate testing and loose coupling. Avoid global state.
- **Thread Safety**: Ensure thread-safe access to shared resources, as the system handles concurrent ROS 2 callbacks and HTTP requests.
- **Standardized Interfaces**: Use consistent data models and serialization patterns across the application boundaries.

## Error Handling Strategy

- **Exceptions**: Use custom exceptions from `exceptions.hpp` for domain-specific errors.
- **API Responses**: REST handlers must catch exceptions and return appropriate HTTP status codes:
  - `400 Bad Request`: Validation failures
  - `404 Not Found`: Unknown components/areas
  - `500 Internal Server Error`: Unexpected exceptions (log these with `RCLCPP_ERROR`)
  - `503 Service Unavailable`: ROS 2 timeouts or communication failures

## Code Review Guidelines

- **Modern C++**: Prefer modern features and standard library over custom implementations.
- **ROS 2 Safety**: Ensure callbacks are non-blocking and thread-safe.
- **Resource Management**: Check for proper lifecycle management of nodes, threads, and HTTP clients.
- **API Consistency**: JSON responses must match `models.hpp` serialization patterns.
- **Error Handling**: Ensure exceptions are caught and mapped to appropriate HTTP status codes.
- **Documentation**: Verify public headers have comments and Sphinx docs are updated for new features.

## Feature Workflow (Traceability)

1. **Check Requirements**: For external API endpoints, check `docs/requirements/` for relevant interoperability specs.
2. **Define Interface**: Add header in `include/ros2_medkit_gateway/`.
3. **Implement Logic**: Add source in `src/` + Unit Test in `test/`.
4. **Implement API**: Add endpoint in `rest_server.cpp` + Integration Test.
5. **Verify**: If implementing a spec-defined endpoint, link the integration test to the requirement via `// @verifies REQ_XXX`.

## Common Tasks

**Add a new REST endpoint**: Modify `rest_server.cpp` setup_routes(), add handler method
**Add a new manager class**: Create header in `include/ros2_medkit_gateway/`, source in `src/`, update `CMakeLists.txt` gateway_lib sources
**Add demo node for testing**: Create in `test/demo_nodes/`, add executable target in `CMakeLists.txt`
