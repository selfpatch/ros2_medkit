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

include_guard(GLOBAL)

# Production-grade compiler warning configuration for ros2_medkit packages.
# Aligned with AUTOSAR C++14, MISRA C++, OpenSSF, and Airbus SecLab guidelines.
#
# Strategy: all warnings enabled, selective -Werror=<flag> only for warnings
# that don't false-positive on external headers (STL, ROS 2, gtest, nlohmann).
# Flags that DO fire on external code remain as warnings for visibility.
#
# Provides:
#   option ENABLE_WERROR (default ON) - selective warnings-as-errors
#   function ros2_medkit_relax_vendor_warnings() - call after ament_add_gtest/gmock
#
# Usage:
#   find_package(ros2_medkit_cmake REQUIRED)
#   include(ROS2MedkitWarnings)
#   ...
#   if(BUILD_TESTING)
#     ament_add_gtest(...)
#     ros2_medkit_relax_vendor_warnings()
#   endif()

option(ENABLE_WERROR "Treat select compiler warnings as errors" ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # -- All warnings (report everything, some as errors, rest as warnings) ------
  add_compile_options(
    # Core warnings
    -Wall
    -Wextra
    -Wpedantic

    # Type safety (MISRA, OpenSSF)
    -Wconversion
    -Wsign-conversion
    -Wdouble-promotion
    -Wfloat-equal            # MISRA M6-2-2, AUTOSAR M6-2-2

    # Shadow / hiding
    -Wshadow
    -Woverloaded-virtual

    # OOP correctness
    -Wnon-virtual-dtor
    -Wold-style-cast

    # Const correctness (MISRA, Airbus SecLab)
    -Wcast-qual

    # Memory / alignment
    -Wcast-align
    -Wnull-dereference

    # Control flow
    -Wimplicit-fallthrough
    -Wswitch-enum            # AUTOSAR A6-4-6

    # Preprocessor (MISRA)
    -Wundef

    # Format strings (OpenSSF)
    -Wformat=2

    # Modern C++ (AUTOSAR A4-10-1)
    -Wzero-as-null-pointer-constant

    # Code cleanliness (AUTOSAR)
    -Wextra-semi
  )

  # -- Selective -Werror for flags safe from external header false positives ----
  # NOT promoted: -Wconversion, -Wsign-conversion, -Wdouble-promotion,
  #   -Wnull-dereference, -Wcast-align (false positives on STL/ROS 2/nlohmann)
  if(ENABLE_WERROR)
    add_compile_options(
      -Werror=shadow
      -Werror=switch-enum
      -Werror=old-style-cast
      -Werror=float-equal
      -Werror=cast-qual
      -Werror=undef
      -Werror=zero-as-null-pointer-constant
      -Werror=extra-semi
      -Werror=overloaded-virtual
      -Werror=non-virtual-dtor
      -Werror=implicit-fallthrough
      -Werror=format=2
    )
  endif()

  # GCC-only warnings
  if(CMAKE_COMPILER_IS_GNUCXX)
    add_compile_options(
      -Wduplicated-cond
      -Wduplicated-branches
      -Wlogical-op
      -Wuseless-cast
    )
    if(ENABLE_WERROR)
      add_compile_options(
        -Werror=duplicated-cond
        -Werror=duplicated-branches
        -Werror=logical-op
        -Werror=useless-cast
      )
    endif()
  endif()
endif()

# Suppress strict warnings on gtest/gmock vendor targets.
# ament_add_gtest/ament_add_gmock build gtest/gmock sources as subdirectory
# targets, which inherit add_compile_options flags. Vendored code may trigger
# promoted warnings (-Werror=switch-enum, -Werror=useless-cast, etc.).
# Call this once at the end of the BUILD_TESTING block.
function(ros2_medkit_relax_vendor_warnings)
  foreach(_target gmock gmock_main gtest gtest_main)
    if(TARGET ${_target})
      target_compile_options(${_target} PRIVATE -w)
    endif()
  endforeach()
endfunction()
