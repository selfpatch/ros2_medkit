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

EntityFreezeFrameCapture::EntityFreezeFrameCapture(rclcpp::Node * node, ros2_common::Ros2SubscriptionExecutor & exec,
                                                   DataProviderResolver resolver, RouteDataFetcher route_fetcher,
                                                   size_t max_faults)
  : resolver_(std::move(resolver))
  , route_fetcher_(std::move(route_fetcher))
  , logger_(node->get_logger())
  , max_faults_(max_faults > 0 ? max_faults : 1) {
  // Resolve the topic from the gateway node (it owns fault_manager.namespace);
  // the subscription itself is created on the executor's dedicated _sub node so
  // it never races rcl's hash-map on the main node (issue #375).
  const auto fault_events_topic = build_fault_manager_events_topic(node);
  auto slot = ros2_common::Ros2SubscriptionSlot::create_typed<ros2_medkit_msgs::msg::FaultEvent>(
      exec, fault_events_topic, rclcpp::QoS(100).reliable(),
      [this](std::shared_ptr<const ros2_medkit_msgs::msg::FaultEvent> msg) {
        on_fault_event(msg);
      });
  if (!slot) {
    RCLCPP_ERROR(logger_, "EntityFreezeFrameCapture: failed to subscribe to %s: %s", fault_events_topic.c_str(),
                 slot.error().c_str());
    return;
  }
  subscription_slot_ = std::move(*slot);

  RCLCPP_INFO(logger_, "EntityFreezeFrameCapture initialized, subscribed to %s", fault_events_topic.c_str());
}

EntityFreezeFrameCapture::~EntityFreezeFrameCapture() {
  subscription_slot_.reset();
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

bool EntityFreezeFrameCapture::route_content_has_live_data(const nlohmann::json & content) {
  if (!content.is_object()) {
    return false;
  }
  if (content.contains("connected") && content["connected"].is_boolean() && !content["connected"].get<bool>()) {
    return false;
  }
  return content.contains("items") && content["items"].is_array() && !content["items"].empty();
}

std::optional<EntityFreezeFrameCapture::Frame>
EntityFreezeFrameCapture::capture_via_route(const std::string & entity_id, const std::string & fault_code) {
  std::optional<nlohmann::json> content;
  try {
    content = route_fetcher_(entity_id);
  } catch (const std::exception & e) {
    log_fallback_failure_once(fault_code, std::string("x-plc-data dispatch threw: ") + e.what());
    return std::nullopt;
  }
  if (!content) {
    return std::nullopt;  // not plugin-owned, no x-plc-data route, or handler error
  }
  if (!route_content_has_live_data(*content)) {
    log_fallback_failure_once(fault_code, "entity '" + entity_id + "' x-plc-data reported no live values");
    return std::nullopt;
  }
  Frame frame;
  frame.entity_id = entity_id;
  frame.values = values_from_list_content(*content);
  frame.captured_at_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  return frame;
}

void EntityFreezeFrameCapture::log_fallback_failure_once(const std::string & fault_code, const std::string & message) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!fallback_logged_.insert(fault_code).second) {
      return;
    }
  }
  RCLCPP_WARN(logger_, "Entity freeze-frame for fault '%s': %s", fault_code.c_str(), message.c_str());
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
      // No DataProvider: the owning plugin may still serve live values through
      // its own x-plc-data route (the commercial PLC bridges). For non-plugin
      // (ROS) sources the fetcher resolves no owner and returns nullopt.
      if (route_fetcher_) {
        if (auto frame = capture_via_route(source, msg->fault.fault_code)) {
          frames.push_back(std::move(*frame));
        }
      }
      continue;
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
