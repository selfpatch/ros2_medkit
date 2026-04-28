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

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/data/ros2_topic_data_provider.hpp"
#include "ros2_medkit_gateway/ros2_common/ros2_subscription_executor.hpp"
#include "ros2_medkit_serialization/json_serializer.hpp"

using ros2_medkit_gateway::Ros2TopicDataProvider;
using ros2_medkit_gateway::TopicDataProvider;
using ros2_medkit_gateway::ros2_common::Ros2SubscriptionExecutor;
using std::chrono_literals::operator""ms;
using std::chrono_literals::operator""s;

class Ros2TopicDataProviderTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override {
    node_ = std::make_shared<rclcpp::Node>("provider_test_gateway");
    // Publishers live on a dedicated node that is NOT added to the main
    // executor. Creating / destroying publishers mutates the owning node's
    // internal hash map; if that node is concurrently iterated by a
    // MultiThreadedExecutor spin thread, TSan flags rcutils_hash_map_*
    // races on every test that exercises a publisher. Keeping the publisher
    // node off the executor removes the reader side entirely - publishers
    // do not need spinning.
    publisher_node_ = std::make_shared<rclcpp::Node>("provider_test_publisher");
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this] {
      executor_->spin();
    });
    sub_exec_ = std::make_shared<Ros2SubscriptionExecutor>(node_);
    serializer_ = std::make_shared<ros2_medkit_serialization::JsonSerializer>();
    provider_ = std::make_unique<Ros2TopicDataProvider>(sub_exec_, serializer_);
  }
  void TearDown() override {
    // Cancel the main executor and join the spin thread first so
    // ~Ros2SubscriptionExecutor does not race the spin thread on the
    // subscription node's rcl internals.
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    provider_.reset();
    sub_exec_.reset();
    executor_.reset();
    publisher_node_.reset();
    node_.reset();
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Node> publisher_node_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::shared_ptr<Ros2SubscriptionExecutor> sub_exec_;
  std::shared_ptr<ros2_medkit_serialization::JsonSerializer> serializer_;
  std::unique_ptr<Ros2TopicDataProvider> provider_;
};

TEST_F(Ros2TopicDataProviderTest, RejectsNullExecutor) {
  EXPECT_THROW(Ros2TopicDataProvider(nullptr, serializer_), std::invalid_argument);
}

TEST_F(Ros2TopicDataProviderTest, ConstructedProviderHasEmptyStats) {
  auto s = provider_->stats();
  EXPECT_EQ(s.pool_size, 0u);
  EXPECT_EQ(s.pool_hits, 0u);
  EXPECT_EQ(s.pool_misses, 0u);
  EXPECT_GT(s.pool_cap, 0u);
}

// @verifies REQ_INTEROP_018
TEST_F(Ros2TopicDataProviderTest, DiscoverFindsPublisher) {
  auto pub = publisher_node_->create_publisher<std_msgs::msg::Int32>("/provider_test_topic", 10);

  auto deadline = std::chrono::steady_clock::now() + 2s;
  bool found = false;
  while (std::chrono::steady_clock::now() < deadline) {
    auto info = provider_->get_topic_info("/provider_test_topic");
    if (info.has_value() && info->publisher_count > 0) {
      found = true;
      EXPECT_EQ(info->type, "std_msgs/msg/Int32");
      EXPECT_GE(info->publisher_count, 1u);
      break;
    }
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_TRUE(found) << "publisher not discovered within 2s";
  EXPECT_TRUE(provider_->has_publishers("/provider_test_topic"));
  EXPECT_FALSE(provider_->has_publishers("/nonexistent_topic_xxx"));
}

// @verifies REQ_INTEROP_018
TEST_F(Ros2TopicDataProviderTest, DiscoverByNamespaceGroupsTopics) {
  auto pub_a = publisher_node_->create_publisher<std_msgs::msg::Int32>("/ns_test/topic_a", 10);
  auto pub_b = publisher_node_->create_publisher<std_msgs::msg::Int32>("/ns_test/topic_b", 10);

  auto deadline = std::chrono::steady_clock::now() + 2s;
  bool found_both = false;
  while (std::chrono::steady_clock::now() < deadline) {
    auto result = provider_->discover_topics_by_namespace();
    if (result.namespaces.count("ns_test") != 0u) {
      auto it = result.topics_by_ns.find("/ns_test");
      if (it != result.topics_by_ns.end() && it->second.publishes.size() >= 2u) {
        found_both = true;
        break;
      }
    }
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_TRUE(found_both);
}

// @verifies REQ_INTEROP_019
TEST_F(Ros2TopicDataProviderTest, SampleWithoutPublishersReturnsMetadataOnly) {
  auto r = provider_->sample("/nonexistent_sample_topic", 100ms);
  ASSERT_TRUE(r.has_value());
  EXPECT_FALSE(r->has_data);
  EXPECT_EQ(r->topic_name, "/nonexistent_sample_topic");
  EXPECT_EQ(r->publisher_count, 0u);
}

// @verifies REQ_INTEROP_019
TEST_F(Ros2TopicDataProviderTest, SampleMatchesReliablePublisherQoS) {
  // Reliable publisher with depth 10 + latched (transient_local). Without
  // QoS matching, a best-effort subscriber would connect but could miss
  // the latched message; a TransientLocal publisher specifically relies on
  // matching subscriber durability for the last-message replay.
  rclcpp::QoS reliable_latched(10);
  reliable_latched.reliable();
  reliable_latched.transient_local();
  auto pub = publisher_node_->create_publisher<std_msgs::msg::Int32>("/reliable_latched_topic", reliable_latched);

  // Pre-publish one message so the latched durability can replay it on
  // subscriber connection.
  std_msgs::msg::Int32 msg;
  msg.data = 123;
  pub->publish(msg);

  auto deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < deadline) {
    if (provider_->has_publishers("/reliable_latched_topic")) {
      break;
    }
    std::this_thread::sleep_for(50ms);
  }

  // First sample creates the subscriber with matching QoS; the latched
  // message should be delivered to it promptly.
  auto data_deadline = std::chrono::steady_clock::now() + 3s;
  bool got = false;
  while (std::chrono::steady_clock::now() < data_deadline) {
    auto r = provider_->sample("/reliable_latched_topic", 500ms);
    ASSERT_TRUE(r.has_value());
    if (r->has_data) {
      got = true;
      break;
    }
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_TRUE(got);
}

// @verifies REQ_INTEROP_019
TEST_F(Ros2TopicDataProviderTest, SampleHitReturnsDataAfterPublish) {
  auto pub = publisher_node_->create_publisher<std_msgs::msg::Int32>("/pool_data_topic", rclcpp::SensorDataQoS());

  // Wait for discovery, then publish repeatedly to ensure subscription catches.
  auto discovery_deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < discovery_deadline) {
    auto info = provider_->get_topic_info("/pool_data_topic");
    if (info.has_value() && info->publisher_count > 0) {
      break;
    }
    std::this_thread::sleep_for(50ms);
  }

  // Priming call: creates pool entry + slot (miss).
  (void)provider_->sample("/pool_data_topic", 100ms);

  std_msgs::msg::Int32 msg;
  msg.data = 42;
  auto deadline = std::chrono::steady_clock::now() + 3s;
  bool got_data = false;
  while (std::chrono::steady_clock::now() < deadline) {
    pub->publish(msg);
    auto r = provider_->sample("/pool_data_topic", 200ms);
    ASSERT_TRUE(r.has_value());
    if (r->has_data) {
      got_data = true;
      EXPECT_EQ(r->topic_name, "/pool_data_topic");
      EXPECT_GE(r->publisher_count, 1u);
      break;
    }
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_TRUE(got_data);

  auto s = provider_->stats();
  EXPECT_GE(s.pool_size, 1u);
  EXPECT_GE(s.pool_misses, 1u);
  EXPECT_GE(s.pool_hits, 1u);
}

TEST_F(Ros2TopicDataProviderTest, PoolCapEvictsLruWhenFull) {
  Ros2TopicDataProvider::Config tight;
  tight.max_pool_size = 1;
  auto local = std::make_unique<Ros2TopicDataProvider>(sub_exec_, serializer_, tight);

  auto pub1 = publisher_node_->create_publisher<std_msgs::msg::Int32>("/cap_topic_1", rclcpp::SensorDataQoS());
  auto pub2 = publisher_node_->create_publisher<std_msgs::msg::Int32>("/cap_topic_2", rclcpp::SensorDataQoS());

  auto deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < deadline) {
    if (local->has_publishers("/cap_topic_1") && local->has_publishers("/cap_topic_2")) {
      break;
    }
    std::this_thread::sleep_for(50ms);
  }

  // First miss: pool grows to 1.
  (void)local->sample("/cap_topic_1", 50ms);
  auto stats1 = local->stats();
  EXPECT_EQ(stats1.pool_size, 1u);
  EXPECT_EQ(stats1.evictions_total, 0u);

  // Second miss on a different topic: LRU eviction replaces topic_1 with topic_2.
  (void)local->sample("/cap_topic_2", 50ms);
  auto stats2 = local->stats();
  EXPECT_EQ(stats2.pool_size, 1u);
  EXPECT_GE(stats2.evictions_total, 1u);
}

TEST_F(Ros2TopicDataProviderTest, HitPromotesTopicToMru) {
  Ros2TopicDataProvider::Config tight;
  tight.max_pool_size = 2;
  auto local = std::make_unique<Ros2TopicDataProvider>(sub_exec_, serializer_, tight);

  auto pub1 = publisher_node_->create_publisher<std_msgs::msg::Int32>("/lru_a", rclcpp::SensorDataQoS());
  auto pub2 = publisher_node_->create_publisher<std_msgs::msg::Int32>("/lru_b", rclcpp::SensorDataQoS());
  auto pub3 = publisher_node_->create_publisher<std_msgs::msg::Int32>("/lru_c", rclcpp::SensorDataQoS());

  auto deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < deadline) {
    if (local->has_publishers("/lru_a") && local->has_publishers("/lru_b") && local->has_publishers("/lru_c")) {
      break;
    }
    std::this_thread::sleep_for(50ms);
  }

  (void)local->sample("/lru_a", 50ms);  // pool = [a]          lru = [a]
  (void)local->sample("/lru_b", 50ms);  // pool = [a, b]       lru = [b, a]
  (void)local->sample("/lru_a", 50ms);  // hit on a, promote.  lru = [a, b]

  const auto prev_evictions = local->stats().evictions_total;
  (void)local->sample("/lru_c", 50ms);  // miss, evict LRU=b.  lru = [c, a]

  auto s = local->stats();
  EXPECT_EQ(s.pool_size, 2u);
  EXPECT_GE(s.evictions_total, prev_evictions + 1);

  // a was promoted by the hit in step 3, so it survived the eviction and the
  // next sample on /lru_a is another hit. b was LRU and should now be a miss.
  const auto prev_hits = s.pool_hits;
  const auto prev_misses = s.pool_misses;
  (void)local->sample("/lru_a", 50ms);
  auto s2 = local->stats();
  EXPECT_GT(s2.pool_hits, prev_hits);

  (void)local->sample("/lru_b", 50ms);  // b was evicted: this is a miss
  auto s3 = local->stats();
  EXPECT_GT(s3.pool_misses, prev_misses);
}

// @verifies REQ_INTEROP_018
TEST_F(Ros2TopicDataProviderTest, SampleParallelReturnsOneResultPerTopic) {
  auto r = provider_->sample_parallel({"/topic_a_notexist", "/topic_b_notexist"}, 100ms);
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->size(), 2u);
  EXPECT_EQ((*r)[0].topic_name, "/topic_a_notexist");
  EXPECT_EQ((*r)[1].topic_name, "/topic_b_notexist");
  EXPECT_FALSE((*r)[0].has_data);
  EXPECT_FALSE((*r)[1].has_data);
}

// @verifies REQ_INTEROP_019
TEST_F(Ros2TopicDataProviderTest, SampleParallelHonorsMaxParallelSamplesAndPreservesOrder) {
  // With max_parallel_samples=2 and 5 input topics the chunk loop must run 3
  // times. Regression that drops chunking entirely (or computes the wrong
  // chunk_size) is what this test pins.
  Ros2TopicDataProvider::Config cfg;
  cfg.max_parallel_samples = 2;
  auto local = std::make_unique<Ros2TopicDataProvider>(sub_exec_, serializer_, cfg);

  const std::vector<std::string> topics{"/c_a_unknown", "/c_b_unknown", "/c_c_unknown", "/c_d_unknown", "/c_e_unknown"};
  auto r = local->sample_parallel(topics, 50ms);
  ASSERT_TRUE(r.has_value());
  ASSERT_EQ(r->size(), topics.size());
  for (std::size_t i = 0; i < topics.size(); ++i) {
    EXPECT_EQ((*r)[i].topic_name, topics[i]) << "Order must survive chunk boundaries";
    EXPECT_FALSE((*r)[i].has_data) << "No publishers means metadata-only";
  }
}

// @verifies REQ_INTEROP_019
TEST_F(Ros2TopicDataProviderTest, SampleParallelEmbedsPerTopicErrorInsteadOfFailingBatch) {
  // Cold-wait cap saturates at 1; holder thread keeps the slot reserved.
  // sample_parallel embeds the per-topic 503 in the corresponding result so
  // the rest of the batch still serves successfully (one bad topic should
  // not kill the whole /components/{id}/data response under load).
  Ros2TopicDataProvider::Config cfg;
  cfg.cold_wait_cap = 1;
  auto capped = std::make_unique<Ros2TopicDataProvider>(sub_exec_, serializer_, cfg);

  auto pub_holder = publisher_node_->create_publisher<std_msgs::msg::Int32>("/sp_cold_holder", rclcpp::QoS(10));
  auto pub_second = publisher_node_->create_publisher<std_msgs::msg::Int32>("/sp_cold_second", rclcpp::QoS(10));

  auto deadline = std::chrono::steady_clock::now() + 2s;
  while ((!capped->has_publishers("/sp_cold_holder") || !capped->has_publishers("/sp_cold_second")) &&
         std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(20ms);
  }
  ASSERT_TRUE(capped->has_publishers("/sp_cold_holder"));
  ASSERT_TRUE(capped->has_publishers("/sp_cold_second"));

  std::thread holder([&capped] {
    auto r = capped->sample("/sp_cold_holder", 500ms);
    (void)r;
  });
  std::this_thread::sleep_for(150ms);

  auto r = capped->sample_parallel({"/sp_cold_second"}, 200ms);
  ASSERT_TRUE(r.has_value()) << "Per-topic errors must be embedded, not surfaced as batch failure";
  ASSERT_EQ(r->size(), 1u);
  const auto & res = (*r)[0];
  EXPECT_EQ(res.topic_name, "/sp_cold_second");
  EXPECT_FALSE(res.has_data);
  ASSERT_TRUE(res.error_code.has_value());
  EXPECT_EQ(*res.error_code, ros2_medkit_gateway::ERR_X_MEDKIT_COLD_WAIT_CAP_EXCEEDED);
  EXPECT_EQ(res.error_http_status, 503);

  holder.join();
}

TEST_F(Ros2TopicDataProviderTest, IsSystemTopicMatchesExpectedSet) {
  EXPECT_TRUE(Ros2TopicDataProvider::is_system_topic("/parameter_events"));
  EXPECT_TRUE(Ros2TopicDataProvider::is_system_topic("/rosout"));
  EXPECT_TRUE(Ros2TopicDataProvider::is_system_topic("/clock"));
  EXPECT_FALSE(Ros2TopicDataProvider::is_system_topic("/tf"));
  EXPECT_FALSE(Ros2TopicDataProvider::is_system_topic("/cmd_vel"));
}

TEST_F(Ros2TopicDataProviderTest, DeletedCopyAndMove) {
  EXPECT_FALSE(std::is_copy_constructible_v<Ros2TopicDataProvider>);
  EXPECT_FALSE(std::is_move_constructible_v<Ros2TopicDataProvider>);
}

TEST_F(Ros2TopicDataProviderTest, ColdWaitCapShedsLoad) {
  // cold_wait_cap = 2 means at most 2 sampler threads can block on cold topics
  // simultaneously; further callers get metadata-only so /health stays live.
  Ros2TopicDataProvider::Config cfg;
  cfg.cold_wait_cap = 2;
  cfg.max_pool_size = 16;
  auto local = std::make_unique<Ros2TopicDataProvider>(sub_exec_, serializer_, cfg);

  // Publisher with no actual data published -> cold topic.
  auto pub = publisher_node_->create_publisher<std_msgs::msg::Int32>("/cold_topic", rclcpp::SensorDataQoS());
  auto deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < deadline) {
    if (local->has_publishers("/cold_topic")) {
      break;
    }
    std::this_thread::sleep_for(20ms);
  }
  // Prime pool entry so subsequent samples find it cold (has publishers, no data).
  (void)local->sample("/cold_topic", 50ms);

  // Four concurrent samplers, all with long timeouts. Two should be allowed to
  // wait; two should bounce back immediately as metadata-only.
  constexpr int kCallers = 4;
  std::vector<std::future<bool>> results;  // true if sample call returned before deadline
  results.reserve(kCallers);
  auto t_start = std::chrono::steady_clock::now();
  for (int i = 0; i < kCallers; ++i) {
    results.push_back(std::async(std::launch::async, [&local] {
      auto r = local->sample("/cold_topic", 1s);
      return r.has_value();
    }));
  }

  // All callers must return within the cold waiters' timeout. Without the cap,
  // all 4 would block for 1s; with the cap, 2 block and 2 return immediately.
  // We assert that at least one completes in <200ms (the fast path).
  int fast = 0;
  for (auto & f : results) {
    if (f.wait_for(200ms) == std::future_status::ready) {
      ++fast;
    }
  }
  EXPECT_GE(fast, kCallers - static_cast<int>(cfg.cold_wait_cap));

  // Join everyone before TearDown.
  for (auto & f : results) {
    (void)f.get();
  }
  auto elapsed = std::chrono::steady_clock::now() - t_start;
  EXPECT_LT(elapsed, 2s);
}

TEST_F(Ros2TopicDataProviderTest, IdleSweepEvictsStaleEntries) {
  Ros2TopicDataProvider::Config cfg;
  cfg.max_pool_size = 8;
  cfg.idle_safety_net = 100ms;  // anything older than 100ms is stale
  cfg.idle_sweep_tick = 10s;    // long - we trigger sweep manually
  auto local = std::make_unique<Ros2TopicDataProvider>(sub_exec_, serializer_, cfg);

  auto pub = publisher_node_->create_publisher<std_msgs::msg::Int32>("/idle_topic", rclcpp::SensorDataQoS());
  auto deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < deadline) {
    if (local->has_publishers("/idle_topic")) {
      break;
    }
    std::this_thread::sleep_for(20ms);
  }
  (void)local->sample("/idle_topic", 50ms);
  EXPECT_EQ(local->stats().pool_size, 1u);

  // Simulate the entry becoming stale.
  std::this_thread::sleep_for(150ms);
  const auto prev_evictions = local->stats().evictions_total;

  local->sweep_idle_entries();
  auto s = local->stats();
  EXPECT_EQ(s.pool_size, 0u);
  EXPECT_GE(s.evictions_total, prev_evictions + 1);
}

TEST_F(Ros2TopicDataProviderTest, IdleSweepKeepsActiveEntries) {
  Ros2TopicDataProvider::Config cfg;
  cfg.max_pool_size = 8;
  cfg.idle_safety_net = 1s;
  cfg.idle_sweep_tick = 10s;
  auto local = std::make_unique<Ros2TopicDataProvider>(sub_exec_, serializer_, cfg);

  auto pub = publisher_node_->create_publisher<std_msgs::msg::Int32>("/active_topic", rclcpp::SensorDataQoS());
  auto deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < deadline) {
    if (local->has_publishers("/active_topic")) {
      break;
    }
    std::this_thread::sleep_for(20ms);
  }

  (void)local->sample("/active_topic", 50ms);
  local->sweep_idle_entries();  // entry just sampled, not stale yet
  EXPECT_EQ(local->stats().pool_size, 1u);
}

// @verifies REQ_INTEROP_019
//
// IMPORTANT: This test is designed to be run under TSan or ASan to actually
// catch the issue #375 regression. Without sanitizers it passes ~99% of the
// time even if the fix is reverted, because the race window is narrow and
// only the FIRST cold-create per topic exercises it - subsequent samples hit
// the warm pool. See ConcurrentSampleStressColdCreatePath below for the
// variant that forces every iteration through the cold path.
//
// CI sanitizer-asan and sanitizer-tsan jobs run all gateway unit tests, so
// this test gets full TSan/ASan coverage on every push.
TEST_F(Ros2TopicDataProviderTest, ConcurrentSampleFromMultipleThreadsDoesNotCrash) {
  // Issue #375 race-fix regression test: multiple httplib-like threads hammering
  // sample() on the same topic must not crash or deadlock. With
  // NativeTopicSampler this would race inside rcl's hash map during
  // create_generic_subscription / destroy. With Ros2TopicDataProvider the
  // subscription lifecycle runs on the single worker, and the pool entry is
  // shared across concurrent samplers.
  //
  // Use a high cold_wait_cap so the test exercises the create-race window
  // without being limited by back-pressure shedding (which is its own test).
  Ros2TopicDataProvider::Config cfg;
  cfg.cold_wait_cap = 64;
  auto local = std::make_unique<Ros2TopicDataProvider>(sub_exec_, serializer_, cfg);
  auto pub = publisher_node_->create_publisher<std_msgs::msg::Int32>("/concurrent_test_topic", rclcpp::SensorDataQoS());

  // Wait for publisher to appear in graph.
  auto discovery_deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < discovery_deadline) {
    if (local->has_publishers("/concurrent_test_topic")) {
      break;
    }
    std::this_thread::sleep_for(20ms);
  }

  // Drive publisher in the background so active samples see data.
  std::atomic<bool> stop{false};
  std::thread pub_thread([&stop, &pub] {
    std_msgs::msg::Int32 msg;
    int counter = 0;
    while (!stop.load(std::memory_order_acquire)) {
      msg.data = counter++;
      pub->publish(msg);
      std::this_thread::sleep_for(5ms);
    }
  });

  constexpr int kThreads = 16;
  constexpr int kIterations = 50;
  std::atomic<int> completions{0};
  std::vector<std::thread> workers;
  workers.reserve(kThreads);
  for (int t = 0; t < kThreads; ++t) {
    workers.emplace_back([&local, &completions] {
      for (int i = 0; i < kIterations; ++i) {
        auto r = local->sample("/concurrent_test_topic", 200ms);
        if (r.has_value()) {
          completions.fetch_add(1);
        }
      }
    });
  }
  for (auto & w : workers) {
    w.join();
  }
  stop.store(true, std::memory_order_release);
  pub_thread.join();

  EXPECT_EQ(completions.load(), kThreads * kIterations);

  // Pool should have EXACTLY one entry for this topic despite concurrent
  // misses. The original NativeTopicSampler bug leaked one subscription per
  // concurrent caller (each caller created its own SampleState + handler
  // subscription). EXPECT_GE would accept a regression where the pool grew
  // one entry per thread; assert equality to prove the deduplication invariant.
  auto s = local->stats();
  EXPECT_EQ(s.pool_size, 1u) << "expected exactly one pool entry, got " << s.pool_size
                             << " - concurrent callers leaking subscriptions?";
}

// @verifies REQ_INTEROP_019
//
// Sibling stress test that forces every iteration through the cold-create
// path - this is the actual race window from issue #375 (concurrent calls to
// rcl_subscription_init from multiple threads). Rotating topic names each
// iteration prevents the warm-pool fast-path that the previous test mostly
// hits. Run under TSan/ASan to catch reintroduced races.
TEST_F(Ros2TopicDataProviderTest, ConcurrentSampleStressColdCreatePath) {
  constexpr int kThreads = 8;
  constexpr int kTopicsPerThread = 12;

  // High cold_wait_cap so every cold sample reaches the create path rather
  // than being shed by back-pressure (cap behavior has its own dedicated
  // tests; this test exists to stress the slot-create race on cold misses).
  Ros2TopicDataProvider::Config cfg;
  cfg.cold_wait_cap = 256;
  auto local = std::make_unique<Ros2TopicDataProvider>(sub_exec_, serializer_, cfg);

  // Pre-create publishers for every {thread, iteration} cell so each cold
  // sample sees publisher_count > 0 and goes through Ros2SubscriptionSlot
  // creation (the rcl_subscription_init race site). Drive each publisher
  // briefly so the cold-wait path can resolve within the test timeout.
  std::vector<rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr> pubs;
  pubs.reserve(static_cast<std::size_t>(kThreads) * kTopicsPerThread);
  for (int t = 0; t < kThreads; ++t) {
    for (int i = 0; i < kTopicsPerThread; ++i) {
      const std::string topic = "/cold_stress_t" + std::to_string(t) + "_i" + std::to_string(i);
      pubs.push_back(publisher_node_->create_publisher<std_msgs::msg::Int32>(topic, rclcpp::SensorDataQoS()));
    }
  }

  // Wait for the last publisher to be visible to the subscription node so the
  // cold-create path actually attempts a real subscribe.
  const std::string last_topic =
      "/cold_stress_t" + std::to_string(kThreads - 1) + "_i" + std::to_string(kTopicsPerThread - 1);
  auto deadline = std::chrono::steady_clock::now() + 3s;
  while (!local->has_publishers(last_topic) && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(20ms);
  }

  // Background driver: publish to every topic at 5ms cadence so cold-wait
  // returns data instead of timing out on this test's 200ms budget.
  std::atomic<bool> stop{false};
  std::thread pub_thread([&pubs, &stop] {
    std_msgs::msg::Int32 msg;
    int counter = 0;
    while (!stop.load(std::memory_order_acquire)) {
      msg.data = counter++;
      for (auto & p : pubs) {
        p->publish(msg);
      }
      std::this_thread::sleep_for(5ms);
    }
  });

  std::atomic<int> issued{0};
  std::vector<std::thread> workers;
  workers.reserve(kThreads);
  for (int t = 0; t < kThreads; ++t) {
    workers.emplace_back([&local, t, &issued] {
      for (int i = 0; i < kTopicsPerThread; ++i) {
        const std::string topic = "/cold_stress_t" + std::to_string(t) + "_i" + std::to_string(i);
        auto r = local->sample(topic, 200ms);
        if (r.has_value()) {
          issued.fetch_add(1);
        }
      }
    });
  }
  for (auto & w : workers) {
    w.join();
  }
  stop.store(true, std::memory_order_release);
  pub_thread.join();
  EXPECT_EQ(issued.load(), kThreads * kTopicsPerThread);
}

TEST_F(Ros2TopicDataProviderTest, TimeoutWithoutPublisherDoesNotCrash) {
  // Repeated short-timeout samples on a non-existent topic must not crash. With
  // NativeTopicSampler the shared_ptr SampleState bug could let a subscription
  // callback fire after stack locals were destroyed; the pool-backed path must
  // likewise survive this pattern.
  for (int i = 0; i < 20; ++i) {
    auto r = provider_->sample("/no_such_topic_xyz", 50ms);
    ASSERT_TRUE(r.has_value());
    EXPECT_FALSE(r->has_data);
  }
}

TEST_F(Ros2TopicDataProviderTest, RapidTimeoutWithActivePublisher) {
  // Rapid sample() calls with a very short timeout while a publisher is active
  // stress the callback-vs-wait race: a message may arrive exactly as wait_for
  // returns timeout. The pool entry keeps the subscription alive across calls,
  // so this exercises concurrent buf_mtx / buf_cv rather than subscription
  // lifetime, but we still require no crashes or hangs.
  auto pub = publisher_node_->create_publisher<std_msgs::msg::Int32>("/rapid_timeout_topic", rclcpp::SensorDataQoS());
  std::atomic<bool> stop{false};
  std::thread pub_thread([&stop, &pub] {
    std_msgs::msg::Int32 msg;
    int counter = 0;
    while (!stop.load(std::memory_order_acquire)) {
      msg.data = counter++;
      pub->publish(msg);
      std::this_thread::sleep_for(5ms);
    }
  });

  for (int i = 0; i < 30; ++i) {
    auto r = provider_->sample("/rapid_timeout_topic", 20ms);
    ASSERT_TRUE(r.has_value());
    // has_data is timing-dependent; we only require the call to return.
  }

  stop.store(true, std::memory_order_release);
  pub_thread.join();
}

// @verifies REQ_INTEROP_019
TEST_F(Ros2TopicDataProviderTest, InterfacePolymorphismWorks) {
  TopicDataProvider & iface = *provider_;
  auto r = iface.sample("/via_interface", 100ms);
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->topic_name, "/via_interface");
}

// @verifies REQ_INTEROP_019
TEST_F(Ros2TopicDataProviderTest, ColdWaitCapExceededReturns503) {
  // Use a dedicated provider with cold-wait-cap 1 so one holder caller in
  // flight triggers the cap on the second caller. The fixture provider
  // uses cap 4 which is hard to saturate deterministically.
  Ros2TopicDataProvider::Config cfg;
  cfg.cold_wait_cap = 1;
  auto capped = std::make_unique<Ros2TopicDataProvider>(sub_exec_, serializer_, cfg);

  // Publishers exist but never publish - sample() enters the cold-wait path
  // instead of bailing early on publisher_count==0.
  auto pub_holder = publisher_node_->create_publisher<std_msgs::msg::Int32>("/cold_wait_cap_holder", rclcpp::QoS(10));
  auto pub_second = publisher_node_->create_publisher<std_msgs::msg::Int32>("/cold_wait_cap_second", rclcpp::QoS(10));

  // Wait for the provider's subscription node to observe the publishers.
  auto deadline = std::chrono::steady_clock::now() + 2s;
  while ((!capped->has_publishers("/cold_wait_cap_holder") || !capped->has_publishers("/cold_wait_cap_second")) &&
         std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(20ms);
  }
  ASSERT_TRUE(capped->has_publishers("/cold_wait_cap_holder"));
  ASSERT_TRUE(capped->has_publishers("/cold_wait_cap_second"));

  // Holder blocks inside sample() on buf_cv.wait_for until its 500ms
  // timeout. During that window it owns the single cold-wait slot.
  std::thread holder([&capped] {
    auto r = capped->sample("/cold_wait_cap_holder", 500ms);
    (void)r;
  });

  // Give the holder enough time to reach buf_cv.wait_for and reserve the
  // cold-wait slot (subscription creation via executor run_sync takes
  // multiple milliseconds).
  std::this_thread::sleep_for(150ms);

  // Second caller should get a 503 with the cold-wait cap code rather than
  // a metadata-only "success" response. Before the fix this returned
  // has_data=false which is indistinguishable from "no publishers yet".
  auto r2 = capped->sample("/cold_wait_cap_second", 200ms);
  ASSERT_FALSE(r2.has_value()) << "expected 503, got metadata-only success";
  EXPECT_EQ(r2.error().http_status, 503);
  EXPECT_EQ(r2.error().code, ros2_medkit_gateway::ERR_X_MEDKIT_COLD_WAIT_CAP_EXCEEDED);
  EXPECT_EQ(r2.error().params["cold_wait_cap"], 1u);

  holder.join();
}
