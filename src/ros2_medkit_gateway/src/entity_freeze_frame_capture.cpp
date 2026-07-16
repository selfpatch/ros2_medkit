// Copyright 2026 mfaferek93
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

#include "ros2_medkit_gateway/entity_freeze_frame_capture.hpp"

#include <chrono>

#include "ros2_medkit_gateway/fault_manager_paths.hpp"

namespace ros2_medkit_gateway {

EntityFreezeFrameCapture::EntityFreezeFrameCapture(rclcpp::Node * node, DataProviderResolver resolver,
                                                   size_t max_faults)
  : resolver_(std::move(resolver)), logger_(node->get_logger()), max_faults_(max_faults > 0 ? max_faults : 1) {
  const auto fault_events_topic = build_fault_manager_events_topic(node);
  subscription_ = node->create_subscription<ros2_medkit_msgs::msg::FaultEvent>(
      fault_events_topic, rclcpp::QoS(100).reliable(),
      [this](const ros2_medkit_msgs::msg::FaultEvent::ConstSharedPtr & msg) {
        on_fault_event(msg);
      });

  RCLCPP_INFO(logger_, "EntityFreezeFrameCapture initialized, subscribed to %s", fault_events_topic.c_str());
}

EntityFreezeFrameCapture::~EntityFreezeFrameCapture() {
  subscription_.reset();
}

std::vector<EntityFreezeFrameCapture::Frame>
EntityFreezeFrameCapture::frames_for(const std::string & fault_code) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = frames_.find(fault_code);
  return it != frames_.end() ? it->second : std::vector<Frame>{};
}

nlohmann::json EntityFreezeFrameCapture::values_from_list_content(const nlohmann::json & content) {
  if (!content.contains("items") || !content["items"].is_array()) {
    return content;
  }
  nlohmann::json values = nlohmann::json::object();
  for (const auto & item : content["items"]) {
    if (!item.is_object()) {
      continue;
    }
    std::string key = item.value("id", item.value("name", ""));
    if (key.empty()) {
      continue;
    }
    values[key] = item.contains("value") ? item["value"] : nlohmann::json{};
  }
  return values;
}

void EntityFreezeFrameCapture::on_fault_event(const ros2_medkit_msgs::msg::FaultEvent::ConstSharedPtr & msg) {
  // Mirror the fault_manager's own capture trigger: freeze-frames are taken
  // when a fault confirms, not on every update.
  if (msg->event_type != ros2_medkit_msgs::msg::FaultEvent::EVENT_CONFIRMED) {
    return;
  }

  std::vector<Frame> frames;
  for (const auto & source : msg->fault.reporting_sources) {
    DataProvider * provider = resolver_ ? resolver_(source) : nullptr;
    if (provider == nullptr) {
      continue;  // not a plugin-owned entity (ROS sources are the fault_manager's job)
    }

    // list_data is expected to serve from the plugin's latest polled values
    // (cheap); plugin code still gets exception-guarded like every other
    // provider call site.
    try {
      auto result = provider->list_data(source);
      if (!result) {
        RCLCPP_WARN(logger_, "Entity freeze-frame for fault '%s': list_data('%s') failed: %s",
                    msg->fault.fault_code.c_str(), source.c_str(), result.error().message.c_str());
        continue;
      }
      Frame frame;
      frame.entity_id = source;
      frame.values = values_from_list_content(result->content);
      frame.captured_at_ns =
          std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch())
              .count();
      frames.push_back(std::move(frame));
    } catch (const std::exception & e) {
      RCLCPP_WARN(logger_, "Entity freeze-frame for fault '%s': plugin threw for entity '%s': %s",
                  msg->fault.fault_code.c_str(), source.c_str(), e.what());
    }
  }

  if (frames.empty()) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  const std::string & fault_code = msg->fault.fault_code;
  if (frames_.find(fault_code) == frames_.end()) {
    insertion_order_.push_back(fault_code);
    while (frames_.size() >= max_faults_ && !insertion_order_.empty()) {
      frames_.erase(insertion_order_.front());
      insertion_order_.pop_front();
    }
  }
  frames_[fault_code] = std::move(frames);

  RCLCPP_DEBUG(logger_, "Captured entity freeze-frame(s) for fault '%s'", fault_code.c_str());
}

}  // namespace ros2_medkit_gateway
