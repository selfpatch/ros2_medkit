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
#include <stdexcept>
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
    if (!publish_success_) {
      // The real Ros2TopicTransport surfaces publish failures via exceptions
      // (TypeNotFoundError / SerializationError / std::runtime_error).
      // Mirror that contract here so DataAccessManager's exception path is
      // exercised by the failure-path test.
      throw std::runtime_error(publish_error_msg_.empty() ? "mock publish failure" : publish_error_msg_);
    }
    return json{{"status", "published"}, {"topic", topic_path}};
  }

  TopicSample sample(const std::string & topic_name, std::chrono::duration<double> timeout) override {
    last_sample_topic_ = topic_name;
    last_sample_timeout_ = timeout.count();
    TopicSample r;
    r.topic = topic_name;
    r.type = "std_msgs/msg/Float32";
    r.publisher_count = sample_publishers_;
    if (!sample_success_) {
      r.success = false;
      r.status = "error";
      r.error_message = sample_error_msg_.empty() ? "mock sample failure" : sample_error_msg_;
      return r;
    }
    r.success = sample_publishers_ > 0;
    r.status = (sample_publishers_ > 0) ? "data" : "metadata_only";
    if (r.success) {
      r.data = json{{"value", 42.0}};
    }
    return r;
  }

  std::pair<uint64_t, uint64_t> count_publishers_subscribers(const std::string & /*topic*/) const override {
    return {sample_publishers_, 0};
  }

  ros2_medkit_serialization::TypeIntrospection * get_type_introspection() const override {
    return nullptr;
  }

  uint64_t sample_publishers_ = 1;
  // Failure-path knobs. When publish_success_ is false, publish() throws to
  // match the real Ros2TopicTransport contract. When sample_success_ is false,
  // sample() returns a TopicSample{success=false, status="error", error_message}
  // so DataAccessManager can propagate the error string to the SOVD response.
  bool publish_success_ = true;
  std::string publish_error_msg_;
  bool sample_success_ = true;
  std::string sample_error_msg_;
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

// Topic-sample responses carry a wall-clock nanosecond timestamp on every
// status, not just "data". Magnitude check: a sane wall clock since epoch
// is well above 1e18 ns (year ~2001) and below 1e20 ns; the test only checks
// that the unit is plausibly ns rather than ms.
TEST(DataAccessManagerRoutingTest, NativeSampleAlwaysIncludesNanosecondTimestamp) {
  auto mock = std::make_shared<MockTopicTransport>();

  // status == "data"
  mock->sample_publishers_ = 1;
  DataAccessManager mgr(mock, 5.0);
  auto with_data = mgr.get_topic_sample_native("/has_pub", 1.0);
  EXPECT_EQ(with_data["status"], "data");
  ASSERT_TRUE(with_data.contains("timestamp"));
  ASSERT_TRUE(with_data["timestamp"].is_number_integer());
  const auto ts_data = with_data["timestamp"].get<int64_t>();
  EXPECT_GT(ts_data, static_cast<int64_t>(1e18));  // would only ever be true for ns
  EXPECT_LT(ts_data, static_cast<int64_t>(1e20));

  // status == "metadata_only"
  mock->sample_publishers_ = 0;
  auto meta = mgr.get_topic_sample_native("/idle", 1.0);
  EXPECT_EQ(meta["status"], "metadata_only");
  ASSERT_TRUE(meta.contains("timestamp"));
  ASSERT_TRUE(meta["timestamp"].is_number_integer());
  const auto ts_meta = meta["timestamp"].get<int64_t>();
  EXPECT_GT(ts_meta, static_cast<int64_t>(1e18));
  EXPECT_LT(ts_meta, static_cast<int64_t>(1e20));
}

// Failure-path coverage: publish() in the real transport throws on
// serialization or runtime errors. DataAccessManager::publish_to_topic must
// propagate that exception unchanged so the handler can translate it to a 5xx.
TEST(DataAccessManagerRoutingTest, PublishFailurePropagatesExceptionFromTransport) {
  auto mock = std::make_shared<MockTopicTransport>();
  mock->publish_success_ = false;
  mock->publish_error_msg_ = "Unknown message type 'bogus/msg/Type'";
  DataAccessManager mgr(mock, 5.0);

  try {
    mgr.publish_to_topic("/foo", "bogus/msg/Type", json::object(), 1.0);
    FAIL() << "expected publish_to_topic to throw on transport failure";
  } catch (const std::exception & e) {
    EXPECT_STREQ(e.what(), "Unknown message type 'bogus/msg/Type'");
  }
  // Mock was still invoked with the requested arguments.
  EXPECT_EQ(mock->last_publish_topic_, "/foo");
  EXPECT_EQ(mock->last_publish_type_, "bogus/msg/Type");
}

// Failure-path coverage: when the transport reports a sample-time error
// (TopicSample{success=false, error_message=...}) the manager must surface
// the error_message in the response under the "error" key without losing the
// topic / status fields the handler needs.
TEST(DataAccessManagerRoutingTest, SampleFailurePropagatesErrorMessageInResponse) {
  auto mock = std::make_shared<MockTopicTransport>();
  mock->sample_publishers_ = 1;  // ensure we reach transport_->sample()
  mock->sample_success_ = false;
  mock->sample_error_msg_ = "wait_for_message timed out";
  DataAccessManager mgr(mock, 5.0);

  auto out = mgr.get_topic_sample_native("/broken", 1.0);
  EXPECT_EQ(out["status"], "error");
  EXPECT_EQ(out["topic"], "/broken");
  ASSERT_TRUE(out.contains("error"));
  EXPECT_EQ(out["error"], "wait_for_message timed out");
  // Timestamp still emitted for error responses so logs/correlation works.
  ASSERT_TRUE(out.contains("timestamp"));
  EXPECT_TRUE(out["timestamp"].is_number_integer());
}

}  // namespace ros2_medkit_gateway
