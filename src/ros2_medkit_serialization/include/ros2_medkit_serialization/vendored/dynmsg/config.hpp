// Copyright 2021 Christophe Bedard
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROS2_MEDKIT_SERIALIZATION__VENDORED__DYNMSG__CONFIG_HPP_
#define ROS2_MEDKIT_SERIALIZATION__VENDORED__DYNMSG__CONFIG_HPP_

// Vendored from dynmsg - configuration hardcoded for ros2_medkit_serialization

// If DYNMSG_VALUE_ONLY is defined, the member's value will be used directly, e.g.
//    frame_id: "my_frame"
// instead of providing the type name as well as the value, e.g.
//    frame_id:
//      type: "string"
//      value: "my_frame"
// This also means that, with DYNMSG_VALUE_ONLY, the resulting
// YAML object can be converted back into a message.
#define DYNMSG_VALUE_ONLY

// [u]int8_t is handled/parsed as a char by yaml-cpp, so force to an intermediate/other type.
// We convert them to string for msg->YAML and then back from string for YAML->msg.
// https://github.com/jbeder/yaml-cpp/issues/201
// (does not appear to be fixed in 0.6.3 or 0.7.0)
#define DYNMSG_YAML_CPP_BAD_INT8_HANDLING

// If DYNMSG_PARSER_DEBUG is defined, some debugging-related stuff will be printed.
// #define DYNMSG_PARSER_DEBUG

#ifdef DYNMSG_PARSER_DEBUG
#  define DYNMSG_DEBUG(code) code
#else
#  define DYNMSG_DEBUG(code) ((void) (0))
#endif  // DYNMSG_PARSER_DEBUG

#endif  // ROS2_MEDKIT_SERIALIZATION__VENDORED__DYNMSG__CONFIG_HPP_
