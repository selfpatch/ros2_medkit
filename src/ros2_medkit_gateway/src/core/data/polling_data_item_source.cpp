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

#include "ros2_medkit_gateway/core/data/polling_data_item_source.hpp"

#include <algorithm>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include "ros2_medkit_gateway/core/data/data_item_reader.hpp"

namespace ros2_medkit_gateway {

namespace {
using Clock = std::chrono::steady_clock;

struct Watch {
  std::string entity_id;
  std::string item_name;
  std::mutex mutex;  ///< held while the item is read and the callback runs
  bool alive{true};
  DataItemSource::SampleCallback callback;
  Clock::time_point next_due;  ///< guarded by Shared::mutex
};
}  // namespace

struct PollingDataItemSource::Shared {
  Shared(ReadFn read_fn, std::chrono::milliseconds poll_interval)
    : read(std::move(read_fn)), interval(poll_interval), thread([this] {
      run();
    }) {
  }

  Shared(const Shared &) = delete;
  Shared & operator=(const Shared &) = delete;
  Shared(Shared &&) = delete;
  Shared & operator=(Shared &&) = delete;

  ~Shared() {
    {
      std::lock_guard<std::mutex> lock(mutex);
      stop = true;
    }
    wake.notify_all();
    thread.join();
  }

  void run() {
    std::unique_lock<std::mutex> lock(mutex);
    while (!stop) {
      const auto now = Clock::now();
      std::vector<std::shared_ptr<Watch>> due;
      auto next = Clock::time_point::max();
      for (const auto & watch : watches) {
        if (watch->next_due <= now) {
          due.push_back(watch);
        } else {
          next = std::min(next, watch->next_due);
        }
      }
      if (due.empty()) {
        if (next == Clock::time_point::max()) {
          wake.wait(lock);
        } else {
          wake.wait_until(lock, next);
        }
        continue;
      }
      lock.unlock();
      for (const auto & watch : due) {
        poll(*watch);
      }
      lock.lock();
      const auto after = Clock::now();
      for (const auto & watch : due) {
        watch->next_due = after + interval;
      }
    }
  }

  void poll(Watch & watch) const {
    std::lock_guard<std::mutex> lock(watch.mutex);
    if (!watch.alive) {
      return;
    }
    try {
      auto value = read(watch.entity_id, watch.item_name);
      if (value) {
        watch.callback(*value);
      }
    } catch (...) {
      // A throwing reader delivers nothing this round.
    }
  }

  ReadFn read;
  std::chrono::milliseconds interval;
  std::mutex mutex;  ///< guards watches, stop and every Watch::next_due
  std::condition_variable wake;
  std::vector<std::shared_ptr<Watch>> watches;
  bool stop{false};
  std::thread thread;  ///< last member: starts after the others are built
};

class PollingDataItemSource::Handle : public TopicSubscriptionHandle {
 public:
  Handle(std::weak_ptr<Shared> shared, std::shared_ptr<Watch> watch)
    : shared_(std::move(shared)), watch_(std::move(watch)) {
  }
  ~Handle() override {
    {
      std::lock_guard<std::mutex> lock(watch_->mutex);
      watch_->alive = false;
      watch_->callback = nullptr;
    }
    if (auto shared = shared_.lock()) {
      std::lock_guard<std::mutex> lock(shared->mutex);
      shared->watches.erase(std::remove(shared->watches.begin(), shared->watches.end(), watch_), shared->watches.end());
    }
  }
  Handle(const Handle &) = delete;
  Handle & operator=(const Handle &) = delete;
  Handle(Handle &&) = delete;
  Handle & operator=(Handle &&) = delete;

 private:
  std::weak_ptr<Shared> shared_;
  std::shared_ptr<Watch> watch_;
};

PollingDataItemSource::PollingDataItemSource(ReadFn read, std::chrono::milliseconds interval)
  : shared_(std::make_shared<Shared>(std::move(read), interval)) {
}

PollingDataItemSource::~PollingDataItemSource() = default;

std::unique_ptr<TopicSubscriptionHandle>
PollingDataItemSource::watch(const std::string & entity_id, const std::string & item_name, SampleCallback callback) {
  if (!callback) {
    return nullptr;
  }
  auto watch = std::make_shared<Watch>();
  watch->entity_id = entity_id;
  watch->item_name = item_name;
  watch->callback = std::move(callback);
  watch->next_due = Clock::now();
  {
    std::lock_guard<std::mutex> lock(shared_->mutex);
    shared_->watches.push_back(watch);
  }
  shared_->wake.notify_all();
  return std::make_unique<Handle>(shared_, std::move(watch));
}

std::unique_ptr<DataItemSource> make_plugin_data_item_source(PluginManager * plugins,
                                                             std::chrono::milliseconds interval) {
  return std::make_unique<PollingDataItemSource>(
      [plugins](const std::string & entity_id,
                const std::string & item_name) -> tl::expected<nlohmann::json, std::string> {
        auto item = read_plugin_data_item(plugins, entity_id, item_name);
        if (!item) {
          return tl::make_unexpected(item.error().message);
        }
        return std::move(item->content);
      },
      interval);
}

}  // namespace ros2_medkit_gateway
