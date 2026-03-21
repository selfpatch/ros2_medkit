# ros2_medkit_cmake

Shared CMake modules for the ros2_medkit workspace. Provides multi-distro compatibility,
build acceleration, and centralized linting configuration across all packages.

## Modules

| Module | Description |
|--------|-------------|
| `ROS2MedkitCcache.cmake` | Auto-detect and configure ccache with PCH-aware sloppiness settings |
| `ROS2MedkitLinting.cmake` | Centralized clang-tidy configuration (opt-in locally, mandatory in CI) |
| `ROS2MedkitCompat.cmake` | Multi-distro compatibility shims for ROS 2 Humble, Jazzy, and Rolling |

### ROS2MedkitCompat

Resolves dependency differences across ROS 2 distributions:

- `medkit_find_yaml_cpp()` - Finds yaml-cpp (namespaced targets on Jazzy, manual fallback on Humble)
- `medkit_find_cpp_httplib()` - Finds cpp-httplib >= 0.14 via pkg-config or CMake config
- `medkit_target_dependencies()` - Drop-in replacement for `ament_target_dependencies` (removed on Rolling)
- `medkit_detect_compat_defs()` / `medkit_apply_compat_defs()` - Compile definitions for version-specific APIs

## Usage

In your package's `CMakeLists.txt`:

```cmake
find_package(ros2_medkit_cmake REQUIRED)
include(ROS2MedkitCcache)
include(ROS2MedkitLinting)
include(ROS2MedkitCompat)
```

Add to `package.xml`:

```xml
<buildtool_depend>ros2_medkit_cmake</buildtool_depend>
```

The cmake modules are automatically available via ament's extras hook after `find_package`.

## License

Apache License 2.0
