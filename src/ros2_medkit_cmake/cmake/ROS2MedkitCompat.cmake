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
# Usage (in each package's CMakeLists.txt):
#   find_package(ros2_medkit_cmake REQUIRED)
#   include(ROS2MedkitCompat)
#
# Macros provided:
#   medkit_find_yaml_cpp()              — Find yaml-cpp, ensure yaml-cpp::yaml-cpp target
#   medkit_find_cpp_httplib()           — Find cpp-httplib, create cpp_httplib_target alias
#   medkit_detect_compat_defs()         — Detect rclcpp/rosbag2 versions, set compat variables
#   medkit_apply_compat_defs(target)    — Apply compile definitions to a target
#   medkit_target_dependencies(target ...) - Drop-in ament_target_dependencies replacement
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
# Finds cpp-httplib >= 0.14 through a multi-tier fallback chain:
#   1. pkg-config (Jazzy/Noble system package)
#   2. cmake find_package(httplib) (source builds, Pixi)
#   3. VENDORED_DIR parameter (bundled header-only copy)
#
# On Humble/Jammy the system package is 0.10.x (too old); the vendored
# fallback in ros2_medkit_gateway handles this automatically.
#
# Creates a unified alias target `cpp_httplib_target` for consumers.
# ---------------------------------------------------------------------------
macro(medkit_find_cpp_httplib)
  cmake_parse_arguments(_mfch "" "VENDORED_DIR" "" ${ARGN})
  find_package(PkgConfig QUIET)
  if(PkgConfig_FOUND)
    pkg_check_modules(cpp_httplib IMPORTED_TARGET cpp-httplib>=0.14)
  endif()
  if(cpp_httplib_FOUND)
    add_library(cpp_httplib_target ALIAS PkgConfig::cpp_httplib)
    message(STATUS "[MedkitCompat] cpp-httplib: using pkg-config (${cpp_httplib_VERSION})")
  else()
    find_package(httplib QUIET)
    if(TARGET httplib::httplib)
      add_library(cpp_httplib_target ALIAS httplib::httplib)
      message(STATUS "[MedkitCompat] cpp-httplib: using cmake config (source build)")
    elseif(_mfch_VENDORED_DIR AND EXISTS "${_mfch_VENDORED_DIR}/httplib.h")
      add_library(cpp_httplib_vendored INTERFACE)
      target_include_directories(cpp_httplib_vendored INTERFACE "${_mfch_VENDORED_DIR}")
      add_library(cpp_httplib_target ALIAS cpp_httplib_vendored)
      message(STATUS "[MedkitCompat] cpp-httplib: using vendored header (${_mfch_VENDORED_DIR}/httplib.h)")
    else()
      message(FATAL_ERROR
        "[MedkitCompat] Could not find cpp-httplib >= 0.14.\n"
        "  Tried: pkg-config, cmake find_package(httplib), VENDORED_DIR.\n"
        "  ros2_medkit_gateway vendors cpp-httplib 0.14.3 - ensure ros2_medkit_gateway\n"
        "  is built first, or pass VENDORED_DIR to medkit_find_cpp_httplib().\n"
        "  See: https://selfpatch.github.io/ros2_medkit/installation.html")
    endif()
  endif()
  unset(_mfch_VENDORED_DIR)
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

# ---------------------------------------------------------------------------
# medkit_target_dependencies(target [PUBLIC|PRIVATE|INTERFACE] dep1 dep2 ...)
# ---------------------------------------------------------------------------
# Drop-in replacement for ament_target_dependencies that works on Rolling
# (where ament_target_dependencies was removed from ament_cmake).
#
# On Humble/Jazzy: delegates to ament_target_dependencies (available).
# On Rolling:      uses target_link_libraries with ${dep_TARGETS}.
#
# When no visibility keyword (PUBLIC/PRIVATE/INTERFACE) is passed, the macro
# uses the plain target_link_libraries signature. This avoids conflicts with
# ament_add_gtest_executable, which also uses the plain signature internally.
# When an explicit visibility keyword IS passed, the keyword signature is used.
#
# Special cases:
#   yaml_cpp_vendor  - vendor package, no _TARGETS; links yaml-cpp::yaml-cpp
#                      (must call medkit_find_yaml_cpp() first)
# ---------------------------------------------------------------------------
macro(medkit_target_dependencies target)
  if(COMMAND ament_target_dependencies)
    ament_target_dependencies(${target} ${ARGN})
  else()
    # Rolling fallback: resolve dependency targets explicitly.
    #
    # CMake forbids mixing the "plain" and "keyword" (PUBLIC/PRIVATE/INTERFACE)
    # signatures of target_link_libraries on the same target.
    # ament_add_gtest_executable uses the plain signature internally, so we must
    # also use the plain signature when no explicit visibility is requested.
    # When the caller passes PUBLIC/PRIVATE/INTERFACE, we honour it (keyword form).
    set(_mtd_has_visibility FALSE)
    set(_mtd_visibility "")
    set(_mtd_plain_deps)
    set(_mtd_public_deps)
    set(_mtd_private_deps)
    set(_mtd_interface_deps)
    foreach(_mtd_arg ${ARGN})
      if(_mtd_arg STREQUAL "PUBLIC" OR _mtd_arg STREQUAL "PRIVATE" OR _mtd_arg STREQUAL "INTERFACE")
        set(_mtd_has_visibility TRUE)
        set(_mtd_visibility ${_mtd_arg})
      else()
        # Resolve dependency to concrete CMake targets
        set(_mtd_dep_targets)
        if(_mtd_arg STREQUAL "yaml_cpp_vendor")
          # yaml_cpp_vendor is a wrapper package - link the real target
          list(APPEND _mtd_dep_targets yaml-cpp::yaml-cpp)
        else()
          # Standard ament package - prefer exported <dep>_TARGETS, fall back to <dep>::<dep>
          set(_mtd_targets_var "${_mtd_arg}_TARGETS")
          if(DEFINED ${_mtd_targets_var} AND NOT "${${_mtd_targets_var}}" STREQUAL "")
            list(APPEND _mtd_dep_targets ${${_mtd_targets_var}})
          elseif(TARGET ${_mtd_arg}::${_mtd_arg})
            list(APPEND _mtd_dep_targets ${_mtd_arg}::${_mtd_arg})
          elseif(DEFINED ${_mtd_arg}_INCLUDE_DIRS OR DEFINED ${_mtd_arg}_LIBRARIES)
            # Header-only or legacy ament package (e.g. ros2_medkit_gateway exports
            # only include dirs via ament_export_include_directories, no CMake targets).
            # Create an IMPORTED INTERFACE target on the fly.
            set(_mtd_iface_target "${_mtd_arg}::${_mtd_arg}")
            if(NOT TARGET ${_mtd_iface_target})
              add_library(${_mtd_iface_target} IMPORTED INTERFACE)
              if(DEFINED ${_mtd_arg}_INCLUDE_DIRS)
                set_target_properties(${_mtd_iface_target} PROPERTIES
                  INTERFACE_INCLUDE_DIRECTORIES "${${_mtd_arg}_INCLUDE_DIRS}")
              endif()
              if(DEFINED ${_mtd_arg}_LIBRARIES AND NOT "${${_mtd_arg}_LIBRARIES}" STREQUAL "")
                set_target_properties(${_mtd_iface_target} PROPERTIES
                  INTERFACE_LINK_LIBRARIES "${${_mtd_arg}_LIBRARIES}")
              endif()
              message(STATUS "[MedkitCompat] ${_mtd_arg}: created interface target from _INCLUDE_DIRS/_LIBRARIES")
            endif()
            list(APPEND _mtd_dep_targets ${_mtd_iface_target})
            unset(_mtd_iface_target)
          else()
            message(FATAL_ERROR
              "[MedkitCompat] medkit_target_dependencies: could not resolve dependency '${_mtd_arg}' "
              "for target '${target}'. Expected variable ${_mtd_arg}_TARGETS or imported target "
              "${_mtd_arg}::${_mtd_arg} or ${_mtd_arg}_INCLUDE_DIRS")
          endif()
          unset(_mtd_targets_var)
        endif()
        # Buffer resolved targets per visibility bucket
        if(_mtd_visibility STREQUAL "PRIVATE")
          list(APPEND _mtd_private_deps ${_mtd_dep_targets})
        elseif(_mtd_visibility STREQUAL "INTERFACE")
          list(APPEND _mtd_interface_deps ${_mtd_dep_targets})
        elseif(_mtd_visibility STREQUAL "PUBLIC")
          list(APPEND _mtd_public_deps ${_mtd_dep_targets})
        else()
          # No visibility keyword seen yet - use plain signature
          list(APPEND _mtd_plain_deps ${_mtd_dep_targets})
        endif()
        unset(_mtd_dep_targets)
      endif()
    endforeach()
    # Plain deps (no visibility keyword) - use plain signature to stay compatible
    # with ament_add_gtest_executable and other plain target_link_libraries calls
    if(_mtd_plain_deps)
      target_link_libraries(${target} ${_mtd_plain_deps})
    endif()
    # Keyword deps - only emitted when caller explicitly specified visibility
    if(_mtd_public_deps)
      target_link_libraries(${target} PUBLIC ${_mtd_public_deps})
    endif()
    if(_mtd_private_deps)
      target_link_libraries(${target} PRIVATE ${_mtd_private_deps})
    endif()
    if(_mtd_interface_deps)
      target_link_libraries(${target} INTERFACE ${_mtd_interface_deps})
    endif()
    unset(_mtd_has_visibility)
    unset(_mtd_visibility)
    unset(_mtd_plain_deps)
    unset(_mtd_public_deps)
    unset(_mtd_private_deps)
    unset(_mtd_interface_deps)
  endif()
endmacro()
