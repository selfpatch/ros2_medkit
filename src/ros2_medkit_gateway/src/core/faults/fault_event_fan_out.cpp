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

#include "ros2_medkit_gateway/core/faults/fault_event_fan_out.hpp"

#include <algorithm>
#include <utility>

namespace ros2_medkit_gateway {

namespace fd = ros2_medkit::fault_detection;

class FaultEventFanOut::Branch : public FaultEventSource {
 public:
  explicit Branch(FaultEventFanOut & owner) : owner_(owner) {
  }
  ~Branch() override {
    unsubscribe();
  }

  void subscribe(Callback callback) override {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      callback_ = std::move(callback);
    }
    owner_.attach(this);
  }

  void unsubscribe() override {
    owner_.detach(this);
    // Waits for a callback that is running.
    std::lock_guard<std::mutex> lock(mutex_);
    callback_ = nullptr;
  }

  void forward(const fd::FaultEventRecord & event) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (callback_) {
      callback_(event);
    }
  }

 private:
  FaultEventFanOut & owner_;
  std::mutex mutex_;
  Callback callback_;
};

FaultEventFanOut::FaultEventFanOut(FaultEventSource & upstream) : upstream_(upstream) {
}

FaultEventFanOut::~FaultEventFanOut() {
  std::lock_guard<std::mutex> lock(upstream_mutex_);
  bool attached = false;
  {
    std::lock_guard<std::mutex> inner(mutex_);
    attached = !branches_.empty();
  }
  if (attached) {
    upstream_.unsubscribe();
  }
}

std::unique_ptr<FaultEventSource> FaultEventFanOut::make_branch() {
  return std::make_unique<Branch>(*this);
}

void FaultEventFanOut::attach(Branch * branch) {
  std::lock_guard<std::mutex> lock(upstream_mutex_);
  bool first = false;
  {
    std::lock_guard<std::mutex> inner(mutex_);
    if (std::find(branches_.begin(), branches_.end(), branch) != branches_.end()) {
      return;
    }
    first = branches_.empty();
    branches_.push_back(branch);
  }
  if (first) {
    upstream_.subscribe([this](const fd::FaultEventRecord & event) {
      deliver(event);
    });
  }
}

void FaultEventFanOut::detach(Branch * branch) {
  std::lock_guard<std::mutex> lock(upstream_mutex_);
  bool last = false;
  {
    std::lock_guard<std::mutex> inner(mutex_);
    const auto it = std::find(branches_.begin(), branches_.end(), branch);
    if (it == branches_.end()) {
      return;
    }
    branches_.erase(it);
    last = branches_.empty();
  }
  if (last) {
    upstream_.unsubscribe();
  }
}

void FaultEventFanOut::deliver(const fd::FaultEventRecord & event) {
  // Held across the calls, so a branch cannot be destroyed while it is being served.
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto * branch : branches_) {
    branch->forward(event);
  }
}

}  // namespace ros2_medkit_gateway
