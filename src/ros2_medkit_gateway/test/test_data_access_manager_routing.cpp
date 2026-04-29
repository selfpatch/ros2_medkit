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
#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include "ros2_medkit_gateway/core/managers/data_access_manager.hpp"
#include "ros2_medkit_gateway/core/transports/topic_transport.hpp"

namespace ros2_medkit_gateway {
namespace {

class MockTopicTransport : public TopicTransport {
 public:
  MockTopicTransport() = default;
  ~MockTopicTransport() override = default;
  MockTopicTransport(const MockTopicTransport &) = delete;
  MockTopicTransport & operator=(const MockTopicTransport &) = delete;
  MockTopicTransport(MockTopicTransport &&) = delete;
  MockTopicTransport & operator=(MockTopicTransport &&) = delete;

  json publish(const std::string & topic_path, const std::string & msg_type, const json & data,
               std::chrono::duration<double> timeout) override {
    last_publish_topic_ = topic_path;
    last_publish_type_ = msg_type;
    last_publish_data_ = data;
    last_publish_timeout_ = timeout.count();
    return json{{"status", "published"}, {"topic", topic_path}};
  }

  TopicSample sample(const std::string & topic_name, std::chrono::duration<double> timeout) override {
    last_sample_topic_ = topic_name;
    last_sample_timeout_ = timeout.count();
    TopicSample r;
    r.success = sample_publishers_ > 0;
    r.status = (sample_publishers_ > 0) ? "data" : "metadata_only";
    r.topic = topic_name;
    r.type = "std_msgs/msg/Float32";
    r.publisher_count = sample_publishers_;
    if (r.success) {
      r.data = json{{"value", 42.0}};
    }
    return r;
  }

  std::pair<uint64_t, uint64_t> count_publishers_subscribers(const std::string &) const override {
    return {sample_publishers_, 0};
  }

  ros2_medkit_serialization::TypeIntrospection * get_type_introspection() const override {
    return nullptr;
  }

  uint64_t sample_publishers_ = 1;
  std::string last_publish_topic_;
  std::string last_publish_type_;
  json last_publish_data_;
  double last_publish_timeout_ = 0.0;
  std::string last_sample_topic_;
  double last_sample_timeout_ = 0.0;
};

}  // namespace

TEST(DataAccessManagerRoutingTest, PublishDelegatesToTransport) {
  auto mock = std::make_shared<MockTopicTransport>();
  DataAccessManager mgr(mock, 5.0);
  auto out = mgr.publish_to_topic("/foo", "std_msgs/msg/Float32", json{{"data", 1.0}}, 2.0);
  EXPECT_EQ(mock->last_publish_topic_, "/foo");
  EXPECT_EQ(mock->last_publish_type_, "std_msgs/msg/Float32");
  EXPECT_EQ(mock->last_publish_data_["data"], 1.0);
  EXPECT_DOUBLE_EQ(mock->last_publish_timeout_, 2.0);
  EXPECT_EQ(out["status"], "published");
}

TEST(DataAccessManagerRoutingTest, SampleWithFallbackUsesDefaultTimeout) {
  auto mock = std::make_shared<MockTopicTransport>();
  DataAccessManager mgr(mock, 7.5);
  mgr.get_topic_sample_with_fallback("/bar", -1.0);
  EXPECT_DOUBLE_EQ(mock->last_sample_timeout_, 7.5);
}

TEST(DataAccessManagerRoutingTest, SampleWithFallbackUsesProvidedTimeout) {
  auto mock = std::make_shared<MockTopicTransport>();
  DataAccessManager mgr(mock, 7.5);
  mgr.get_topic_sample_with_fallback("/baz", 3.0);
  EXPECT_DOUBLE_EQ(mock->last_sample_timeout_, 3.0);
}

TEST(DataAccessManagerRoutingTest, NativeSampleWithoutPublishersReturnsMetadataWithoutTransportCall) {
  auto mock = std::make_shared<MockTopicTransport>();
  mock->sample_publishers_ = 0;
  DataAccessManager mgr(mock, 5.0);
  auto out = mgr.get_topic_sample_native("/empty", 1.0);
  EXPECT_EQ(out["status"], "metadata_only");
  EXPECT_EQ(out["topic"], "/empty");
  EXPECT_EQ(out["publisher_count"].get<uint64_t>(), 0u);
  EXPECT_TRUE(mock->last_sample_topic_.empty());  // sample() must not be called
}

TEST(DataAccessManagerRoutingTest, NativeSampleWithPublishersDelegatesAndIncludesData) {
  auto mock = std::make_shared<MockTopicTransport>();
  mock->sample_publishers_ = 2;
  DataAccessManager mgr(mock, 5.0);
  auto out = mgr.get_topic_sample_native("/has_pub", 1.0);
  EXPECT_EQ(out["status"], "data");
  EXPECT_EQ(out["data"]["value"], 42.0);
  EXPECT_EQ(mock->last_sample_topic_, "/has_pub");
}

}  // namespace ros2_medkit_gateway
