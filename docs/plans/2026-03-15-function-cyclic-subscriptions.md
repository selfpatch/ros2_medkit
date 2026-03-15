# Function-Level Cyclic Subscriptions Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Enable function-level cyclic subscriptions so `x-medkit-graph` can register an SSE sampler on the existing subscription infrastructure.

**Architecture:** Promote `FUNCTION` to a supported cyclic subscription entity alongside `APP` and `COMPONENT`. Keep the existing handler pipeline intact by widening parser, route registration, and capability/discovery exposure rather than adding collection-specific exceptions.

**Tech Stack:** C++, cpp-httplib, gtest, ROS 2 Jazzy, colcon, Docker

---

### Task 1: Add failing tests

**Files:**
- Modify: `src/ros2_medkit_gateway/test/test_cyclic_subscription_handlers.cpp`
- Modify: `src/ros2_medkit_gateway/test/test_gateway_node.cpp`

**Step 1: Write the failing tests**

- Add a parser test that expects `/api/v1/functions/func1/x-medkit-graph` to parse successfully.
- Add a gateway route test that exercises function cyclic subscription create/list/get endpoints.
- Extend the function detail endpoint test to assert `cyclic-subscriptions` appears in the response and capabilities.

**Step 2: Run tests to verify they fail**

Run:

```bash
colcon test --packages-select ros2_medkit_gateway --ctest-args -R "(test_cyclic_subscription_handlers|test_gateway_node)"
```

Expected:

- Function parser assertion fails
- Function route/detail assertions fail

### Task 2: Implement function support

**Files:**
- Modify: `src/ros2_medkit_gateway/src/http/handlers/cyclic_subscription_handlers.cpp`
- Modify: `src/ros2_medkit_gateway/src/http/rest_server.cpp`
- Modify: `src/ros2_medkit_gateway/src/models/entity_capabilities.cpp`
- Modify: `src/ros2_medkit_gateway/src/http/handlers/discovery_handlers.cpp`

**Step 1: Update parser and entity-type mapping**

- Accept `functions` in `parse_resource_uri()`
- Return `"functions"` from `extract_entity_type()` for function routes

**Step 2: Register function cyclic subscription routes**

- Add function CRUD and SSE endpoints in `RestServer::register_routes()`

**Step 3: Expose function capability/discovery**

- Add `CYCLIC_SUBSCRIPTIONS` to function capabilities
- Add `cyclic-subscriptions` URI and capability to function detail responses

**Step 4: Run targeted tests to verify they pass**

Run:

```bash
colcon test --packages-select ros2_medkit_gateway --ctest-args -R "(test_cyclic_subscription_handlers|test_gateway_node)"
```

Expected:

- Targeted tests pass

### Task 3: Full verification

**Files:**
- No source changes expected

**Step 1: Run package build and tests in Docker**

Run:

```bash
docker run --rm --network host -v /media/sewon/Dev/ros2_medkit:/home/devuser/workspace -w /home/devuser/workspace ros2_medkit-devtest bash -lc 'set -e; source /opt/ros/jazzy/setup.bash; colcon build --packages-up-to ros2_medkit_gateway --build-base build-docker-graph --install-base install-docker-graph; source install-docker-graph/setup.bash; colcon test --packages-select ros2_medkit_gateway --build-base build-docker-graph --install-base install-docker-graph --test-result-base test-results-docker-graph; colcon test-result --test-result-base test-results-docker-graph --verbose'
```

Expected:

- Build succeeds
- `ros2_medkit_gateway` tests pass with zero failures

### Task 4: Commit

**Files:**
- Stage only intended source and plan/design docs

**Step 1: Commit**

```bash
git add docs/plans src/ros2_medkit_gateway
git commit -m "Add function cyclic subscription support"
```
