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

#include "ros2_medkit_serialization/message_cleanup.hpp"

#include "rcutils/allocator.h"

namespace ros2_medkit_serialization {

void destroy_ros_message(RosMessage_Cpp * ros_msg) {
  if (ros_msg != nullptr && ros_msg->data != nullptr) {
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dynmsg::cpp::ros_message_destroy_with_allocator(ros_msg, &allocator);
  }
}

}  // namespace ros2_medkit_serialization
