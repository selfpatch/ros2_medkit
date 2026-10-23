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

cmake_minimum_required(VERSION 3.16)
project(check_cores_without_ros LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(${CMAKE_CURRENT_LIST_DIR}/../../src/ros2_medkit_fault_reporter/cmake/fault_reporter_core.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/../../src/ros2_medkit_fault_manager/cmake/fault_manager_core.cmake)

add_executable(check_cores_without_ros main.cpp)
target_link_libraries(check_cores_without_ros PRIVATE fault_reporter_core fault_manager_core)
