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

# Auto-detect and enable ccache if available.
# Include this module early in CMakeLists.txt (before add_library/add_executable).
#
# Override with: -DCMAKE_CXX_COMPILER_LAUNCHER=<path> / -DCMAKE_C_COMPILER_LAUNCHER=<path>
#
# When using with precompiled headers (PCH), set the environment variable:
#   export CCACHE_SLOPPINESS=pch_defines,time_macros
# Without this, ccache will have poor hit rates on PCH-using targets.

if(NOT CMAKE_C_COMPILER_LAUNCHER OR NOT CMAKE_CXX_COMPILER_LAUNCHER)
  find_program(_CCACHE ccache)
  if(_CCACHE)
    if(NOT CMAKE_C_COMPILER_LAUNCHER)
      set(CMAKE_C_COMPILER_LAUNCHER "${_CCACHE}" CACHE STRING "C compiler launcher")
    endif()
    if(NOT CMAKE_CXX_COMPILER_LAUNCHER)
      set(CMAKE_CXX_COMPILER_LAUNCHER "${_CCACHE}" CACHE STRING "C++ compiler launcher")
    endif()
    message(STATUS "ccache found: ${_CCACHE}")
  endif()
  unset(_CCACHE)
endif()
