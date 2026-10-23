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

# ROS-free reporter core: plain CMake, no ament. Included by the ament build and by scripts/check_cores_without_ros.
if(CMAKE_VERSION VERSION_LESS 3.16)
  message(FATAL_ERROR "fault_reporter_core needs CMake 3.16 or newer")
endif()

set(_frc_root "${CMAKE_CURRENT_LIST_DIR}/..")

# The ament build provides the namespaced imported target; a plain project gets the header by path.
if(TARGET ros2_medkit_fault_detection::ros2_medkit_fault_detection)
  set(_frc_fault_detection ros2_medkit_fault_detection::ros2_medkit_fault_detection)
else()
  if(NOT TARGET ros2_medkit_fault_detection)
    add_library(ros2_medkit_fault_detection INTERFACE)
    target_include_directories(ros2_medkit_fault_detection INTERFACE
      "${_frc_root}/../ros2_medkit_fault_detection/include")
  endif()
  set(_frc_fault_detection ros2_medkit_fault_detection)
endif()

add_library(fault_reporter_core STATIC
  "${_frc_root}/src/local_filter.cpp"
  "${_frc_root}/src/core/reporter_core.cpp"
)
set_target_properties(fault_reporter_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(fault_reporter_core PUBLIC cxx_std_17)
target_include_directories(fault_reporter_core PUBLIC "${_frc_root}/include")
target_link_libraries(fault_reporter_core PUBLIC ${_frc_fault_detection})
