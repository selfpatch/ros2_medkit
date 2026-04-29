// Copyright 2026 bburda
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

#include "ros2_medkit_gateway/core/managers/fault_manager.hpp"

#include <utility>

namespace ros2_medkit_gateway {

FaultManager::FaultManager(std::shared_ptr<FaultServiceTransport> transport) : transport_(std::move(transport)) {
}

FaultResult FaultManager::report_fault(const std::string & fault_code, uint8_t severity,
                                       const std::string & description, const std::string & source_id) {
  return transport_->report_fault(fault_code, severity, description, source_id);
}

FaultResult FaultManager::list_faults(const std::string & source_id, bool include_prefailed, bool include_confirmed,
                                      bool include_cleared, bool include_healed, bool include_muted,
                                      bool include_clusters) {
  return transport_->list_faults(source_id, include_prefailed, include_confirmed, include_cleared, include_healed,
                                 include_muted, include_clusters);
}

FaultWithEnvJsonResult FaultManager::get_fault_with_env(const std::string & fault_code, const std::string & source_id) {
  return transport_->get_fault_with_env(fault_code, source_id);
}

FaultResult FaultManager::get_fault(const std::string & fault_code, const std::string & source_id) {
  return transport_->get_fault(fault_code, source_id);
}

FaultResult FaultManager::clear_fault(const std::string & fault_code) {
  return transport_->clear_fault(fault_code);
}

FaultResult FaultManager::get_snapshots(const std::string & fault_code, const std::string & topic) {
  return transport_->get_snapshots(fault_code, topic);
}

FaultResult FaultManager::get_rosbag(const std::string & fault_code) {
  return transport_->get_rosbag(fault_code);
}

FaultResult FaultManager::list_rosbags(const std::string & entity_fqn) {
  return transport_->list_rosbags(entity_fqn);
}

bool FaultManager::is_available() const {
  return transport_->is_available();
}

bool FaultManager::wait_for_services(std::chrono::duration<double> timeout) {
  return transport_->wait_for_services(timeout);
}

}  // namespace ros2_medkit_gateway
