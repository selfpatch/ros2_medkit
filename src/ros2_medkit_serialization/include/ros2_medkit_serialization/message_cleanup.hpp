// Copyright 2026 Selfpatch
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

#ifndef ROS2_MEDKIT_SERIALIZATION__MESSAGE_CLEANUP_HPP_
#define ROS2_MEDKIT_SERIALIZATION__MESSAGE_CLEANUP_HPP_

#include "ros2_medkit_serialization/vendored/dynmsg/typesupport.hpp"

namespace ros2_medkit_serialization {

/// Destroy a dynamically allocated ROS message created by JsonSerializer::from_json()
///
/// This function properly cleans up messages allocated via the serialization library.
/// It calls the message's finalization function and deallocates the memory.
///
/// @param ros_msg Pointer to the RosMessage_Cpp structure to destroy.
///                If nullptr or if ros_msg->data is nullptr, this is a no-op.
void destroy_ros_message(RosMessage_Cpp * ros_msg);

}  // namespace ros2_medkit_serialization

#endif  // ROS2_MEDKIT_SERIALIZATION__MESSAGE_CLEANUP_HPP_
