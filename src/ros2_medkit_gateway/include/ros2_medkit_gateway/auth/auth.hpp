// Copyright 2025 bburda
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

/**
 * @file auth.hpp
 * @brief Convenience header that includes all authentication components
 *
 * This header provides a single include for all authentication-related
 * functionality in the ros2_medkit_gateway.
 */

#pragma once

#include "ros2_medkit_gateway/auth/auth_config.hpp"
#include "ros2_medkit_gateway/auth/auth_manager.hpp"
#include "ros2_medkit_gateway/auth/auth_middleware.hpp"
#include "ros2_medkit_gateway/auth/auth_models.hpp"
#include "ros2_medkit_gateway/auth/auth_requirement_policy.hpp"
