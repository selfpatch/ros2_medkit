# dynmsg - Vendored Source

This directory contains vendored source code from the `dynmsg` library, part of the
`dynamic_message_introspection` project.

## Original Project

- **Repository**: https://github.com/osrf/dynamic_message_introspection
- **License**: Apache License 2.0

## Copyright

- Copyright 2020 Open Source Robotics Foundation, Inc.
- Copyright 2021 Christophe Bedard

## Modifications

The following modifications were made for integration into ros2_medkit_serialization:

1. Include paths updated from `"dynmsg/..."` to `"ros2_medkit_serialization/vendored/dynmsg/..."`
2. Include guards updated to use `ROS2_MEDKIT_SERIALIZATION__VENDORED__DYNMSG__` prefix
3. Configuration defines hardcoded in `config.hpp` (instead of CMake-generated):
   - `DYNMSG_VALUE_ONLY` enabled
   - `DYNMSG_YAML_CPP_BAD_INT8_HANDLING` enabled
   - `DYNMSG_PARSER_DEBUG` disabled
4. C API files excluded (only C++ API used)

## Excluded Files

The following files from the original dynmsg library were not vendored as they are
not used by ros2_medkit_serialization:

- `message_reading_c.cpp` (C API)
- `msg_parser_c.cpp` (C API)

## License

Licensed under the Apache License, Version 2.0. See the LICENSE file in the
ros2_medkit_serialization package root for the full license text.
