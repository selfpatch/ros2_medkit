// Copyright 2026 Wiktor Jelen
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

#include "ros2_medkit_fault_manager/pending_topic_watcher.hpp"

#include <map>
#include <stdexcept>
#include <utility>

namespace ros2_medkit_fault_manager {

PendingTopicWatcher::PendingTopicWatcher(rclcpp::Node * node, std::mutex & node_ops_mutex,
                                         std::vector<std::string> topics, std::chrono::milliseconds period,
                                         OnTypeKnown on_type_known)
  : node_(node)
  , node_ops_mutex_(node_ops_mutex)
  , pending_(std::move(topics))
  , on_type_known_(std::move(on_type_known)) {
  if (!node_) {
    throw std::invalid_argument("PendingTopicWatcher requires a valid node pointer");
  }
  // A wall timer, so a paused or not-yet-published /clock cannot hold the
  // checks back.
  std::lock_guard<std::mutex> lock(node_ops_mutex_);
  timer_ = node_->create_wall_timer(period, [this]() {
    check();
  });
}

PendingTopicWatcher::~PendingTopicWatcher() {
  timer_->cancel();
  std::lock_guard<std::mutex> lock(node_ops_mutex_);
  timer_.reset();
}

void PendingTopicWatcher::check() {
  // One read of the graph for every pending topic, not one per topic: each
  // read copies the whole topic map, and a long list of topics configured
  // ahead of a stack that is not up yet would otherwise pay for it N times a
  // period on the thread that serves the node's callbacks.
  std::map<std::string, std::vector<std::string>> names_and_types;
  try {
    names_and_types = node_->get_topic_names_and_types();
  } catch (const std::runtime_error &) {
    return;  // context invalidated during shutdown; the next check, if any, reads again
  }

  std::vector<std::pair<std::string, std::string>> known;
  std::vector<std::string> still_pending;
  for (auto & topic : pending_) {
    const auto it = names_and_types.find(topic);
    if (it != names_and_types.end() && !it->second.empty()) {
      known.emplace_back(std::move(topic), it->second.front());
    } else {
      still_pending.push_back(std::move(topic));
    }
  }
  pending_ = std::move(still_pending);

  // Cancelled, not reset: the executor holds the timer while this callback
  // runs, so a reset here would destroy it later, on the executor's terms and
  // outside the node-ops mutex. The destructor resets it under the mutex.
  if (pending_.empty()) {
    timer_->cancel();
  }

  for (const auto & [topic, type] : known) {
    on_type_known_(topic, type);
  }
}

}  // namespace ros2_medkit_fault_manager
