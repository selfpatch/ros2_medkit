// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef ROS2_MEDKIT_SERIALIZATION__VENDORED__DYNMSG__STRING_UTILS_HPP_
#define ROS2_MEDKIT_SERIALIZATION__VENDORED__DYNMSG__STRING_UTILS_HPP_

#include <string>

extern "C"
{
/// Convert a std::string (8-bit characters) to a std::u16string (16-bit characters).
std::u16string string_to_u16string(const std::string & input);

/// Convert a std::u16string (16-bit characters) to a std::string (8-bit characters).
std::string u16string_to_string(const std::u16string & input);
}  // extern "C"

#endif  // ROS2_MEDKIT_SERIALIZATION__VENDORED__DYNMSG__STRING_UTILS_HPP_
