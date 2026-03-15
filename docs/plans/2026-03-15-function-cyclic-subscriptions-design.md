# Function-Level Cyclic Subscriptions Design

## Summary

Add function-level cyclic subscription support so vendor extensions such as `x-medkit-graph` can expose SSE streams through the same cyclic subscription infrastructure already used by apps and components.

## Scope

- Extend cyclic subscription resource URI parsing to accept `/api/v1/functions/{id}/...`
- Register `/functions/{id}/cyclic-subscriptions` CRUD and `/events` routes
- Expose `cyclic-subscriptions` in function discovery responses and capabilities
- Keep existing app/component behavior unchanged

## Approach

Use the existing generic cyclic subscription flow rather than adding a graph-specific bypass. Functions become a first-class entity type for cyclic subscriptions, which keeps URI validation, collection support checks, sampler lookup, transport delivery, and discovery output consistent across entity types.

## Files

- `src/ros2_medkit_gateway/src/http/handlers/cyclic_subscription_handlers.cpp`
- `src/ros2_medkit_gateway/src/http/rest_server.cpp`
- `src/ros2_medkit_gateway/src/models/entity_capabilities.cpp`
- `src/ros2_medkit_gateway/src/http/handlers/discovery_handlers.cpp`
- `src/ros2_medkit_gateway/test/test_cyclic_subscription_handlers.cpp`
- `src/ros2_medkit_gateway/test/test_gateway_node.cpp`

## Testing

- Parser test for valid function resource URIs
- Route test for function cyclic subscription CRUD availability
- Function detail response test for `cyclic-subscriptions` URI and capability
- Package-level `ros2_medkit_gateway` build and test in Docker
