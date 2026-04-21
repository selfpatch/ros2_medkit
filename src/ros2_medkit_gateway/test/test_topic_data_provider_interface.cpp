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
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "ros2_medkit_gateway/data/topic_data_provider.hpp"

using ros2_medkit_gateway::ComponentTopics;
using ros2_medkit_gateway::ErrorInfo;
using ros2_medkit_gateway::TopicConnection;
using ros2_medkit_gateway::TopicDataProvider;
using ros2_medkit_gateway::TopicDiscoveryResult;
using ros2_medkit_gateway::TopicEndpoint;
using ros2_medkit_gateway::TopicInfo;
using ros2_medkit_gateway::TopicSampleResult;

namespace {

class MockTopicDataProvider : public TopicDataProvider {
 public:
  tl::expected<TopicSampleResult, ErrorInfo> sample(const std::string & topic,
                                                    std::chrono::milliseconds /*timeout*/) override {
    ++sample_calls;
    last_sampled_topic = topic;
    if (topic == "/missing") {
      return tl::unexpected(ErrorInfo{"ERR_TOPIC_NOT_FOUND", "no such topic", 404, nlohmann::json::object()});
    }
    TopicSampleResult r;
    r.topic_name = topic;
    r.message_type = "std_msgs/msg/String";
    r.has_data = true;
    r.publisher_count = 1;
    return r;
  }

  tl::expected<std::vector<TopicSampleResult>, ErrorInfo> sample_parallel(const std::vector<std::string> & topics,
                                                                          std::chrono::milliseconds timeout) override {
    std::vector<TopicSampleResult> out;
    out.reserve(topics.size());
    for (const auto & t : topics) {
      auto one = sample(t, timeout);
      if (!one) {
        return tl::unexpected(one.error());
      }
      out.push_back(*one);
    }
    return out;
  }

  std::optional<TopicInfo> get_topic_info(const std::string & topic) override {
    if (topic == "/missing") {
      return std::nullopt;
    }
    TopicInfo i;
    i.name = topic;
    i.type = "std_msgs/msg/String";
    i.publisher_count = 1;
    return i;
  }

  bool has_publishers(const std::string & topic) override {
    return topic != "/missing";
  }

  std::vector<TopicInfo> discover(const std::string & namespace_prefix) override {
    if (namespace_prefix == "/none") {
      return {};
    }
    return {TopicInfo{namespace_prefix + "/a", "std_msgs/msg/String", 1, 0},
            TopicInfo{namespace_prefix + "/b", "std_msgs/msg/String", 1, 0}};
  }

  std::vector<TopicInfo> discover_all() override {
    return discover("/root");
  }

  std::map<std::string, ComponentTopics> build_component_topic_map() override {
    return {};
  }

  ComponentTopics get_component_topics(const std::string & /*component_fqn*/) override {
    return {};
  }

  TopicDiscoveryResult discover_topics_by_namespace() override {
    return {};
  }

  std::set<std::string> discover_topic_namespaces() override {
    return {};
  }

  ComponentTopics get_topics_for_namespace(const std::string & /*ns_prefix*/) override {
    return {};
  }

  std::vector<TopicEndpoint> get_topic_publishers(const std::string & /*topic*/) override {
    return {};
  }

  std::vector<TopicEndpoint> get_topic_subscribers(const std::string & /*topic*/) override {
    return {};
  }

  TopicConnection get_topic_connection(const std::string & topic) override {
    TopicConnection c;
    c.topic_name = topic;
    return c;
  }

  int sample_calls{0};
  std::string last_sampled_topic;
};

}  // namespace

TEST(TopicDataProviderInterface, MockCanBeInvokedViaReference) {
  MockTopicDataProvider mock;
  TopicDataProvider & iface = mock;  // only via interface reference

  auto r = iface.sample("/hello", std::chrono::milliseconds{100});
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->topic_name, "/hello");
  EXPECT_TRUE(r->has_data);
  EXPECT_EQ(mock.sample_calls, 1);
}

TEST(TopicDataProviderInterface, ErrorBranchCarriesErrorInfo) {
  MockTopicDataProvider mock;
  TopicDataProvider & iface = mock;

  auto r = iface.sample("/missing", std::chrono::milliseconds{100});
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().code, "ERR_TOPIC_NOT_FOUND");
  EXPECT_EQ(r.error().http_status, 404);
}

TEST(TopicDataProviderInterface, SampleParallelPreservesOrder) {
  MockTopicDataProvider mock;
  TopicDataProvider & iface = mock;

  auto r = iface.sample_parallel({"/a", "/b", "/c"}, std::chrono::milliseconds{100});
  ASSERT_TRUE(r.has_value());
  ASSERT_EQ(r->size(), 3u);
  EXPECT_EQ((*r)[0].topic_name, "/a");
  EXPECT_EQ((*r)[1].topic_name, "/b");
  EXPECT_EQ((*r)[2].topic_name, "/c");
}

TEST(TopicDataProviderInterface, MetadataLookupsDoNotRequireRclcpp) {
  // This test runs without rclcpp::init - verifies the interface itself has
  // no hidden ROS 2 dependency when consumed via a pure-C++ mock.
  MockTopicDataProvider mock;
  TopicDataProvider & iface = mock;

  EXPECT_TRUE(iface.has_publishers("/hello"));
  EXPECT_FALSE(iface.has_publishers("/missing"));

  auto info = iface.get_topic_info("/hello");
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->publisher_count, 1u);

  auto none = iface.get_topic_info("/missing");
  EXPECT_FALSE(none.has_value());

  auto topics = iface.discover("/ns");
  EXPECT_EQ(topics.size(), 2u);
  EXPECT_EQ(topics[0].name, "/ns/a");

  auto conn = iface.get_topic_connection("/x");
  EXPECT_EQ(conn.topic_name, "/x");
}
