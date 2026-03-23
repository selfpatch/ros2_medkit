// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include <dlfcn.h>

#include <sstream>
#include <string>

#include "rcutils/allocator.h"
#include "rcutils/logging_macros.h"

#include "ros2_medkit_serialization/vendored/dynmsg/types.h"
#include "ros2_medkit_serialization/vendored/dynmsg/typesupport.hpp"

namespace dynmsg
{

namespace c
{

const TypeInfo * get_type_info(const InterfaceTypeName & interface_type)
{
  // Delegate to the full version with "msg" as the default interface type
  return get_type_info(FullInterfaceTypeName{interface_type.first, "msg", interface_type.second});
}

const TypeInfo * get_type_info(const FullInterfaceTypeName & interface_type)
{
  const std::string & pkg_name = std::get<0>(interface_type);
  const std::string & iface_type = std::get<1>(interface_type);  // msg, srv, or action
  const std::string & type_name = std::get<2>(interface_type);

  // Load the introspection library for the package containing the requested type
  std::stringstream ts_lib_name;
  ts_lib_name << "lib" << pkg_name << "__rosidl_typesupport_introspection_c.so";
  RCUTILS_LOG_DEBUG_NAMED(
    "dynmsg",
    "Loading introspection type support library %s",
    ts_lib_name.str().c_str());
  void * introspection_type_support_lib = dlopen(ts_lib_name.str().c_str(), RTLD_LAZY);
  if (introspection_type_support_lib == nullptr) {
    RCUTILS_LOG_ERROR_NAMED(
      "dynmsg", "failed to load introspection type support library: %s", dlerror());
    return nullptr;
  }
  // Load the function that, when called, will give us the introspection information for the
  // interface type we are interested in
  std::stringstream ts_func_name;
  ts_func_name << "rosidl_typesupport_introspection_c__get_message_type_support_handle__" <<
    pkg_name << "__" << iface_type << "__" << type_name;
  RCUTILS_LOG_DEBUG_NAMED(
    "dynmsg", "Loading type support function %s", ts_func_name.str().c_str());

  get_message_ts_func introspection_type_support_handle_func =
    reinterpret_cast<get_message_ts_func>(dlsym(
      introspection_type_support_lib,
      ts_func_name.str().c_str()));
  if (introspection_type_support_handle_func == nullptr) {
    RCUTILS_LOG_ERROR_NAMED(
      "dynmsg",
      "failed to load introspection type support function: %s",
      dlerror());
    return nullptr;
  }

  // Call the function to get the introspection information we want
  const rosidl_message_type_support_t * introspection_ts =
    introspection_type_support_handle_func();
  RCUTILS_LOG_DEBUG_NAMED(
    "dynmsg",
    "Loaded type support %s",
    introspection_ts->typesupport_identifier);
  const rosidl_typesupport_introspection_c__MessageMembers * type_info =
    reinterpret_cast<const rosidl_typesupport_introspection_c__MessageMembers *>(
    introspection_ts->data);

  return type_info;
}

dynmsg_ret_t ros_message_with_typeinfo_init(
  const TypeInfo * type_info,
  RosMessage * ros_msg,
  rcutils_allocator_t * allocator)
{
  rcutils_allocator_t default_allocator = rcutils_get_default_allocator();
  if (!allocator) {
    allocator = &default_allocator;
  }
  RCUTILS_LOG_DEBUG_NAMED(
    "dynmsg",
    "Allocating message buffer of size %ld bytes",
    type_info->size_of_);
  // Allocate space to store the binary representation of the message
  uint8_t * data =
    static_cast<uint8_t *>(allocator->allocate(type_info->size_of_, allocator->state));
  if (nullptr == data) {
    return DYNMSG_RET_ERROR;
  }
  // Initialise the message buffer according to the interface type
  type_info->init_function(data, ROSIDL_RUNTIME_C_MSG_INIT_ALL);
  *ros_msg = RosMessage{type_info, data};
  return DYNMSG_RET_OK;
}

dynmsg_ret_t ros_message_init(
  const InterfaceTypeName & interface_type,
  RosMessage * ros_msg)
{
  const auto * type_info = get_type_info(interface_type);
  if (nullptr == type_info) {
    return DYNMSG_RET_ERROR;
  }
  return dynmsg::c::ros_message_with_typeinfo_init(type_info, ros_msg, nullptr);
}

void ros_message_destroy_with_allocator(RosMessage * ros_msg, rcutils_allocator_t * allocator)
{
  ros_msg->type_info->fini_function(ros_msg->data);
  allocator->deallocate(ros_msg->data, allocator->state);
}

void ros_message_destroy(RosMessage * ros_msg)
{
  ros_msg->type_info->fini_function(ros_msg->data);
  delete[] ros_msg->data;
}

}  // namespace c

namespace cpp
{

const TypeInfo_Cpp * get_type_info(const InterfaceTypeName & interface_type)
{
  // Delegate to the full version with "msg" as the default interface type
  return get_type_info(FullInterfaceTypeName{interface_type.first, "msg", interface_type.second});
}

const TypeInfo_Cpp * get_type_info(const FullInterfaceTypeName & interface_type)
{
  const std::string & pkg_name = std::get<0>(interface_type);
  const std::string & iface_type = std::get<1>(interface_type);  // msg, srv, or action
  const std::string & type_name = std::get<2>(interface_type);

  // Load the introspection library for the package containing the requested type
  std::string ts_lib_name = "lib" + pkg_name + "__rosidl_typesupport_introspection_cpp.so";
  RCUTILS_LOG_DEBUG_NAMED(
    "dynmsg",
    "Loading C++ introspection type support library %s",
    ts_lib_name.c_str());
  void * introspection_type_support_lib = dlopen(ts_lib_name.c_str(), RTLD_LAZY);
  if (nullptr == introspection_type_support_lib) {
    RCUTILS_LOG_ERROR_NAMED(
      "dynmsg", "failed to load C++ introspection type support library: %s", dlerror());
    return nullptr;
  }

  // Try the unmangled C-style function name first (more reliable across compilers)
  // Format: rosidl_typesupport_introspection_cpp__get_message_type_support_handle__
  //         [pkg]__[iface]__[type]
  std::string ts_func_name_c =
    "rosidl_typesupport_introspection_cpp__get_message_type_support_handle__" +
    pkg_name + "__" + iface_type + "__" + type_name;
  RCUTILS_LOG_DEBUG_NAMED("dynmsg", "Trying C-style function name: %s", ts_func_name_c.c_str());

  get_message_ts_func introspection_type_support_handle_func =
    reinterpret_cast<get_message_ts_func>(dlsym(
      introspection_type_support_lib,
      ts_func_name_c.c_str()));

  if (nullptr == introspection_type_support_handle_func) {
    // Fall back to mangled C++ name for messages only (backward compatibility)
    // The mangled name format only works reliably for "msg" types
    if (iface_type == "msg") {
      std::string ts_func_name_cpp =
        "_ZN36rosidl_typesupport_introspection_cpp31get_message_type_support_handleIN" +
        std::to_string(pkg_name.length()) + pkg_name + "3msg" +
        std::to_string(type_name.length() + 1) + type_name +
        "_ISaIvEEEEEPK29rosidl_message_type_support_tv";
      RCUTILS_LOG_DEBUG_NAMED("dynmsg", "Trying mangled C++ function name: %s",
        ts_func_name_cpp.c_str());

      introspection_type_support_handle_func =
        reinterpret_cast<get_message_ts_func>(dlsym(
          introspection_type_support_lib,
          ts_func_name_cpp.c_str()));
    }
  }

  if (nullptr == introspection_type_support_handle_func) {
    RCUTILS_LOG_ERROR_NAMED(
      "dynmsg",
      "failed to load C++ introspection type support function: %s",
      dlerror());
    return nullptr;
  }

  // Call the function to get the introspection information we want
  const rosidl_message_type_support_t * introspection_ts = introspection_type_support_handle_func();
  RCUTILS_LOG_DEBUG_NAMED(
    "dynmsg",
    "Loaded C++ type support %s",
    introspection_ts->typesupport_identifier);
  const TypeInfo_Cpp * type_info = reinterpret_cast<const TypeInfo_Cpp *>(introspection_ts->data);

  return type_info;
}

dynmsg_ret_t ros_message_with_typeinfo_init(
  const TypeInfo_Cpp * type_info,
  RosMessage_Cpp * ros_msg,
  rcutils_allocator_t * allocator)
{
  RCUTILS_LOG_DEBUG_NAMED(
    "dynmsg",
    "Allocating message buffer of size %ld bytes",
    type_info->size_of_);
  // Allocate space to store the binary representation of the message
  uint8_t * data =
    static_cast<uint8_t *>(allocator->allocate(type_info->size_of_, allocator->state));
  if (nullptr == data) {
    return DYNMSG_RET_ERROR;
  }
  // Initialise the message buffer according to the interface type
  type_info->init_function(data, rosidl_runtime_cpp::MessageInitialization::ALL);
  *ros_msg = RosMessage_Cpp{type_info, data};
  return DYNMSG_RET_OK;
}

void ros_message_destroy_with_allocator(RosMessage_Cpp * ros_msg, rcutils_allocator_t * allocator)
{
  ros_msg->type_info->fini_function(ros_msg->data);
  allocator->deallocate(ros_msg->data, allocator->state);
}

}  // namespace cpp

}  // namespace dynmsg
