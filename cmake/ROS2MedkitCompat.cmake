# Copyright 2026 bburda
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# =============================================================================
# ROS2MedkitCompat.cmake — Multi-distro compatibility module for ros2_medkit
# =============================================================================
#
# Centralizes all dependency resolution workarounds for supporting multiple
# ROS 2 distributions (Humble, Jazzy, Rolling) in a single place.
#
# Usage (in each package's CMakeLists.txt, after find_package(ament_cmake)):
#   list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../cmake")
#   include(ROS2MedkitCompat)
#
# Macros provided:
#   medkit_find_yaml_cpp()              — Find yaml-cpp, ensure yaml-cpp::yaml-cpp target
#   medkit_find_cpp_httplib()           — Find cpp-httplib, create cpp_httplib_target alias
#   medkit_detect_compat_defs()         — Detect rclcpp/rosbag2 versions, set compat variables
#   medkit_apply_compat_defs(target)    — Apply compile definitions to a target
#
# Variables set by medkit_detect_compat_defs():
#   MEDKIT_RCLCPP_VERSION_MAJOR     — integer (e.g., 16 for Humble, 28 for Jazzy)
#   MEDKIT_ROSBAG2_OLD_TIMESTAMP    — ON if rosbag2_storage < 0.22.0 (Humble)
#

include_guard(GLOBAL)

# ---------------------------------------------------------------------------
# medkit_find_yaml_cpp()
# ---------------------------------------------------------------------------
# Jazzy's yaml_cpp_vendor exports a namespaced yaml-cpp::yaml-cpp cmake target.
# Humble's yaml_cpp_vendor bundles yaml-cpp but does NOT export the cmake target.
# This macro creates an IMPORTED INTERFACE target when find_package doesn't.
#
# Prerequisite: find_package(yaml_cpp_vendor REQUIRED) must be called before.
# ---------------------------------------------------------------------------
macro(medkit_find_yaml_cpp)
  find_package(yaml-cpp QUIET)
  if(NOT TARGET yaml-cpp::yaml-cpp)
    find_library(_MEDKIT_YAML_CPP_LIB yaml-cpp)
    find_path(_MEDKIT_YAML_CPP_INCLUDE yaml-cpp/yaml.h)
    if(_MEDKIT_YAML_CPP_LIB AND _MEDKIT_YAML_CPP_INCLUDE)
      add_library(yaml-cpp::yaml-cpp IMPORTED INTERFACE)
      set_target_properties(yaml-cpp::yaml-cpp PROPERTIES
        INTERFACE_LINK_LIBRARIES "${_MEDKIT_YAML_CPP_LIB}"
        INTERFACE_INCLUDE_DIRECTORIES "${_MEDKIT_YAML_CPP_INCLUDE}"
      )
      message(STATUS "[MedkitCompat] yaml-cpp: created target from system library (${_MEDKIT_YAML_CPP_LIB})")
    else()
      message(FATAL_ERROR "[MedkitCompat] Could not find yaml-cpp library. "
        "Ensure yaml_cpp_vendor is installed: apt install ros-${ROS_DISTRO}-yaml-cpp-vendor")
    endif()
    unset(_MEDKIT_YAML_CPP_LIB)
    unset(_MEDKIT_YAML_CPP_INCLUDE)
  else()
    message(STATUS "[MedkitCompat] yaml-cpp: using native cmake target")
  endif()
endmacro()

# ---------------------------------------------------------------------------
# medkit_find_cpp_httplib()
# ---------------------------------------------------------------------------
# On Jazzy/Noble, libcpp-httplib-dev is available via apt and provides a
# pkg-config .pc file.  On Humble/Jammy, cpp-httplib must be built from
# source, which installs a CMake config file (httplibConfig.cmake).
#
# Creates a unified alias target `cpp_httplib_target` for consumers.
# ---------------------------------------------------------------------------
macro(medkit_find_cpp_httplib)
  find_package(PkgConfig QUIET)
  if(PkgConfig_FOUND)
    pkg_check_modules(cpp_httplib IMPORTED_TARGET cpp-httplib)
  endif()
  if(cpp_httplib_FOUND)
    add_library(cpp_httplib_target ALIAS PkgConfig::cpp_httplib)
    message(STATUS "[MedkitCompat] cpp-httplib: using pkg-config (system package)")
  else()
    find_package(httplib REQUIRED)
    add_library(cpp_httplib_target ALIAS httplib::httplib)
    message(STATUS "[MedkitCompat] cpp-httplib: using cmake config (source build)")
  endif()
endmacro()

# ---------------------------------------------------------------------------
# medkit_detect_compat_defs()
# ---------------------------------------------------------------------------
# Detects rclcpp and rosbag2 versions to set compatibility variables.
# Call AFTER find_package(rclcpp) and optionally find_package(rosbag2_storage).
#
# Sets:
#   MEDKIT_RCLCPP_VERSION_MAJOR  — integer (16=Humble, 21+=Iron, 28+=Jazzy)
#   MEDKIT_ROSBAG2_OLD_TIMESTAMP — ON if rosbag2_storage < 0.22.0 (Humble)
# ---------------------------------------------------------------------------
macro(medkit_detect_compat_defs)
  # --- rclcpp version ---
  if(rclcpp_VERSION)
    string(REGEX MATCH "^([0-9]+)" _medkit_rclcpp_major "${rclcpp_VERSION}")
    set(MEDKIT_RCLCPP_VERSION_MAJOR ${_medkit_rclcpp_major})
    unset(_medkit_rclcpp_major)
  else()
    set(MEDKIT_RCLCPP_VERSION_MAJOR 0)
  endif()

  if(MEDKIT_RCLCPP_VERSION_MAJOR GREATER_EQUAL 21)
    message(STATUS "[MedkitCompat] rclcpp ${rclcpp_VERSION} (Iron+): native GenericClient + BestAvailable QoS")
  else()
    message(STATUS "[MedkitCompat] rclcpp ${rclcpp_VERSION} (Humble): using compat shim for GenericClient")
  endif()

  # --- rosbag2 timestamp API ---
  if(rosbag2_storage_VERSION)
    if(rosbag2_storage_VERSION VERSION_LESS "0.22.0")
      set(MEDKIT_ROSBAG2_OLD_TIMESTAMP ON)
      message(STATUS "[MedkitCompat] rosbag2_storage ${rosbag2_storage_VERSION}: using time_stamp field (Humble)")
    else()
      set(MEDKIT_ROSBAG2_OLD_TIMESTAMP OFF)
      message(STATUS "[MedkitCompat] rosbag2_storage ${rosbag2_storage_VERSION}: using recv_timestamp (Iron+)")
    endif()
  endif()
endmacro()

# ---------------------------------------------------------------------------
# medkit_apply_compat_defs(target)
# ---------------------------------------------------------------------------
# Applies all relevant compile definitions to a target based on detected
# compatibility state.  Call AFTER medkit_detect_compat_defs().
# ---------------------------------------------------------------------------
function(medkit_apply_compat_defs target)
  if(MEDKIT_ROSBAG2_OLD_TIMESTAMP)
    target_compile_definitions(${target} PRIVATE ROSBAG2_USE_OLD_TIMESTAMP_FIELD)
  endif()
endfunction()
