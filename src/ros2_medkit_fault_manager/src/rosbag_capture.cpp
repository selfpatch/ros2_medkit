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

#include "ros2_medkit_fault_manager/rosbag_capture.hpp"

#include <rcutils/types/uint8_array.h>
#include <unistd.h>

#include <algorithm>
#include <filesystem>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <set>
#include <sstream>

#include "ros2_medkit_fault_manager/time_utils.hpp"

namespace ros2_medkit_fault_manager {

namespace {

/// Custom deleter for rcutils_uint8_array_t that calls rcutils_uint8_array_fini
struct Uint8ArrayDeleter {
  void operator()(rcutils_uint8_array_t * array) const {
    if (array) {
      // Cleanup is best-effort - nothing meaningful to do if it fails during destruction
      [[maybe_unused]] rcutils_ret_t ret = rcutils_uint8_array_fini(array);
      delete array;
    }
  }
};

/// Create a properly-initialized SerializedBagMessage with RAII-safe memory management
/// @param topic Topic name for the message
/// @param timestamp_ns Timestamp in nanoseconds
/// @param src_msg Source serialized message to copy from
/// @param logger Logger for error reporting
/// @return Shared pointer to bag message, or nullptr on allocation failure
std::shared_ptr<rosbag2_storage::SerializedBagMessage> create_bag_message(const std::string & topic,
                                                                          int64_t timestamp_ns,
                                                                          const rcl_serialized_message_t & src_msg,
                                                                          const rclcpp::Logger & logger) {
  auto bag_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  bag_msg->topic_name = topic;
  bag_msg->recv_timestamp = timestamp_ns;
  bag_msg->send_timestamp = timestamp_ns;

  // Create serialized_data with custom deleter that calls rcutils_uint8_array_fini
  auto serialized_data = std::shared_ptr<rcutils_uint8_array_t>(new rcutils_uint8_array_t(), Uint8ArrayDeleter{});

  // Initialize with rcutils (RAII-safe allocation)
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rcutils_ret_t ret = rcutils_uint8_array_init(serialized_data.get(), src_msg.buffer_length, &allocator);
  if (ret != RCUTILS_RET_OK) {
    RCLCPP_ERROR(logger, "Failed to allocate serialized message buffer (%zu bytes): %s", src_msg.buffer_length,
                 rcutils_get_error_string().str);
    rcutils_reset_error();
    return nullptr;
  }

  // Copy data
  memcpy(serialized_data->buffer, src_msg.buffer, src_msg.buffer_length);
  serialized_data->buffer_length = src_msg.buffer_length;

  bag_msg->serialized_data = serialized_data;
  return bag_msg;
}

}  // namespace

RosbagCapture::RosbagCapture(rclcpp::Node * node, FaultStorage * storage, const RosbagConfig & config,
                             const SnapshotConfig & snapshot_config)
  : node_(node), storage_(storage), config_(config), snapshot_config_(snapshot_config) {
  if (!node_) {
    throw std::invalid_argument("RosbagCapture requires a valid node pointer");
  }
  if (!storage_) {
    throw std::invalid_argument("RosbagCapture requires a valid storage pointer");
  }

  if (!config_.enabled) {
    RCLCPP_INFO(node_->get_logger(), "RosbagCapture disabled");
    return;
  }

  // Validate storage format before proceeding
  validate_storage_format();

  RCLCPP_INFO(node_->get_logger(), "RosbagCapture initialized (duration=%.1fs, after=%.1fs, lazy_start=%s, format=%s)",
              config_.duration_sec, config_.duration_after_sec, config_.lazy_start ? "true" : "false",
              config_.format.c_str());

  // Start immediately if not lazy_start
  if (!config_.lazy_start) {
    start();
  }
}

RosbagCapture::~RosbagCapture() {
  stop();
}

void RosbagCapture::start() {
  if (!config_.enabled || running_.load()) {
    return;
  }

  init_subscriptions();
  running_.store(true);

  RCLCPP_INFO(node_->get_logger(), "RosbagCapture started with %zu topic subscriptions", subscriptions_.size());
}

void RosbagCapture::stop() {
  if (!running_.load()) {
    return;
  }

  running_.store(false);

  // Cancel any pending post-fault timer
  if (post_fault_timer_) {
    post_fault_timer_->cancel();
    post_fault_timer_.reset();
  }

  // Clear subscriptions
  subscriptions_.clear();

  // Clear buffer
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    message_buffer_.clear();
  }

  RCLCPP_INFO(node_->get_logger(), "RosbagCapture stopped");
}

bool RosbagCapture::is_running() const {
  return running_.load();
}

void RosbagCapture::on_fault_prefailed(const std::string & fault_code) {
  if (!config_.enabled) {
    return;
  }

  // Start buffer if lazy_start and not already running
  if (config_.lazy_start && !running_.load()) {
    RCLCPP_INFO(node_->get_logger(), "RosbagCapture starting on PREFAILED for fault '%s'", fault_code.c_str());
    start();
  }
}

void RosbagCapture::on_fault_confirmed(const std::string & fault_code) {
  if (!config_.enabled || !running_.load()) {
    return;
  }

  // Don't start a new recording if we're already recording post-fault
  if (recording_post_fault_.load()) {
    RCLCPP_WARN(node_->get_logger(), "Already recording post-fault data, skipping confirmation for '%s'",
                fault_code.c_str());
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "RosbagCapture: fault '%s' confirmed, flushing buffer to bag", fault_code.c_str());

  // Flush buffer to bag
  std::string bag_path = flush_to_bag(fault_code);
  if (bag_path.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Failed to create bag file for fault '%s'", fault_code.c_str());
    return;
  }

  // If duration_after_sec > 0, continue recording
  if (config_.duration_after_sec > 0.0) {
    current_fault_code_ = fault_code;
    current_bag_path_ = bag_path;
    recording_post_fault_.store(true);

    // Create timer for post-fault recording
    auto duration = std::chrono::duration<double>(config_.duration_after_sec);
    post_fault_timer_ =
        node_->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(duration), [this]() {
          post_fault_timer_callback();
        });

    RCLCPP_DEBUG(node_->get_logger(), "Recording %.1fs more after fault confirmation", config_.duration_after_sec);
  } else {
    // No post-fault recording, close writer and finalize immediately
    {
      std::lock_guard<std::mutex> wlock(writer_mutex_);
      active_writer_.reset();
      created_topics_.clear();
    }

    size_t bag_size = calculate_bag_size(bag_path);

    RosbagFileInfo info;
    info.fault_code = fault_code;
    info.file_path = bag_path;
    info.format = config_.format;
    info.duration_sec = config_.duration_sec;
    info.size_bytes = bag_size;
    info.created_at_ns = get_wall_clock_ns();

    storage_->store_rosbag_file(info);
    enforce_storage_limits();

    RCLCPP_INFO(node_->get_logger(), "Bag file created: %s (%.2f MB)", bag_path.c_str(),
                static_cast<double>(bag_size) / (1024.0 * 1024.0));
  }
}

void RosbagCapture::on_fault_cleared(const std::string & fault_code) {
  if (!config_.enabled || !config_.auto_cleanup) {
    return;
  }

  // Delete the bag file for this fault
  if (storage_->delete_rosbag_file(fault_code)) {
    RCLCPP_INFO(node_->get_logger(), "Auto-cleanup: deleted bag file for fault '%s'", fault_code.c_str());
  }
}

void RosbagCapture::init_subscriptions() {
  auto topics = resolve_topics();
  if (topics.empty()) {
    RCLCPP_WARN(node_->get_logger(), "No topics configured for rosbag capture");
    return;
  }

  subscriptions_.clear();

  // Track topics that couldn't be subscribed yet (type not discoverable)
  std::vector<std::string> pending_topics;

  for (const auto & topic : topics) {
    if (!try_subscribe_topic(topic)) {
      pending_topics.push_back(topic);
    }
  }

  // If we have pending topics, start a timer to retry subscription
  if (!pending_topics.empty() && !discovery_retry_timer_) {
    pending_topics_ = pending_topics;
    discovery_retry_count_ = 0;
    discovery_retry_timer_ = node_->create_wall_timer(std::chrono::milliseconds(500), [this]() {
      discovery_retry_callback();
    });
    RCLCPP_INFO(node_->get_logger(), "Will retry subscribing to %zu pending topics", pending_topics_.size());
  }
}

bool RosbagCapture::try_subscribe_topic(const std::string & topic) {
  std::string msg_type = get_topic_type(topic);
  if (msg_type.empty()) {
    RCLCPP_DEBUG(node_->get_logger(), "Cannot determine type for topic '%s', will retry", topic.c_str());
    return false;
  }

  // Cache the topic type
  {
    std::lock_guard<std::mutex> lock(topic_types_mutex_);
    topic_types_[topic] = msg_type;
  }

  try {
    rclcpp::QoS qos = rclcpp::SensorDataQoS();

    auto callback = [this, topic, msg_type](const std::shared_ptr<const rclcpp::SerializedMessage> & msg) {
      message_callback(topic, msg_type, msg);
    };

    auto subscription = node_->create_generic_subscription(topic, msg_type, qos, callback);
    subscriptions_.push_back(subscription);

    RCLCPP_INFO(node_->get_logger(), "Subscribed to '%s' (%s) for rosbag capture", topic.c_str(), msg_type.c_str());
    return true;

  } catch (const std::exception & e) {
    RCLCPP_WARN(node_->get_logger(), "Failed to create subscription for '%s': %s", topic.c_str(), e.what());
    return false;
  }
}

void RosbagCapture::discovery_retry_callback() {
  if (pending_topics_.empty() || !running_.load()) {
    if (discovery_retry_timer_) {
      discovery_retry_timer_->cancel();
      discovery_retry_timer_.reset();
    }
    return;
  }

  discovery_retry_count_++;
  constexpr int max_retries = 20;  // 10 seconds total (20 * 500ms)

  std::vector<std::string> still_pending;
  for (const auto & topic : pending_topics_) {
    if (!try_subscribe_topic(topic)) {
      still_pending.push_back(topic);
    }
  }
  pending_topics_ = still_pending;

  if (pending_topics_.empty()) {
    RCLCPP_INFO(node_->get_logger(), "All topics subscribed after %d retries", discovery_retry_count_);
    discovery_retry_timer_->cancel();
    discovery_retry_timer_.reset();
  } else if (discovery_retry_count_ >= max_retries) {
    RCLCPP_WARN(node_->get_logger(), "Giving up on %zu topics after %d retries: ", pending_topics_.size(), max_retries);
    for (const auto & topic : pending_topics_) {
      RCLCPP_WARN(node_->get_logger(), "  - %s", topic.c_str());
    }
    discovery_retry_timer_->cancel();
    discovery_retry_timer_.reset();
    pending_topics_.clear();
  }
}

void RosbagCapture::message_callback(const std::string & topic, const std::string & msg_type,
                                     const std::shared_ptr<const rclcpp::SerializedMessage> & msg) {
  if (!running_.load()) {
    return;
  }

  // Use wall clock time, not sim time, for proper timestamps
  int64_t timestamp_ns = get_wall_clock_ns();

  // During post-fault recording, write directly to bag (no buffering)
  if (recording_post_fault_.load()) {
    std::lock_guard<std::mutex> wlock(writer_mutex_);
    if (active_writer_) {
      try {
        // Create topic if not already created
        if (created_topics_.find(topic) == created_topics_.end()) {
          rosbag2_storage::TopicMetadata topic_meta;
          topic_meta.name = topic;
          topic_meta.type = msg_type;
          topic_meta.serialization_format = "cdr";
          active_writer_->create_topic(topic_meta);
          created_topics_.insert(topic);
        }

        auto bag_msg = create_bag_message(topic, timestamp_ns, msg->get_rcl_serialized_message(), node_->get_logger());
        if (bag_msg) {
          active_writer_->write(bag_msg);
        }
        // Memory is automatically cleaned up by RAII when bag_msg goes out of scope
      } catch (const std::exception & e) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Failed to write post-fault message: %s",
                             e.what());
      }
    }
    return;  // Don't buffer during post-fault recording
  }

  // Normal buffering mode
  BufferedMessage buffered;
  buffered.topic = topic;
  buffered.message_type = msg_type;
  buffered.serialized_data = std::make_shared<rclcpp::SerializedMessage>(*msg);
  buffered.timestamp_ns = timestamp_ns;

  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    message_buffer_.push_back(std::move(buffered));
  }

  // Prune old messages
  prune_buffer();
}

void RosbagCapture::prune_buffer() {
  // Don't prune during post-fault recording - we need all messages for the final flush
  if (recording_post_fault_.load()) {
    return;
  }

  std::lock_guard<std::mutex> lock(buffer_mutex_);

  if (message_buffer_.empty()) {
    return;
  }

  // Use wall clock time, not sim time, for consistent buffer pruning
  int64_t now_ns = get_wall_clock_ns();
  int64_t duration_ns = static_cast<int64_t>(config_.duration_sec * 1e9);
  int64_t cutoff_ns = now_ns - duration_ns;

  // Remove messages older than cutoff
  while (!message_buffer_.empty() && message_buffer_.front().timestamp_ns < cutoff_ns) {
    message_buffer_.pop_front();
  }
}

std::vector<std::string> RosbagCapture::resolve_topics() const {
  std::set<std::string> topics_set;

  if (config_.topics == "config") {
    // Reuse JSON snapshot config topics
    for (const auto & topic : snapshot_config_.default_topics) {
      topics_set.insert(topic);
    }
    for (const auto & [code, topics_vec] : snapshot_config_.fault_specific) {
      for (const auto & topic : topics_vec) {
        topics_set.insert(topic);
      }
    }
    for (const auto & [pattern, topics_vec] : snapshot_config_.patterns) {
      for (const auto & topic : topics_vec) {
        topics_set.insert(topic);
      }
    }
  } else if (config_.topics == "all" || config_.topics == "auto") {
    // Discover all available topics ("auto" is an alias for "all")
    auto topic_names_and_types = node_->get_topic_names_and_types();
    for (const auto & [topic, types] : topic_names_and_types) {
      // Skip internal ROS topics
      if (topic.find("/parameter_events") != std::string::npos || topic.find("/rosout") != std::string::npos) {
        continue;
      }
      topics_set.insert(topic);
    }
  } else if (config_.topics == "explicit") {
    // Explicit mode: use only include_topics, no topic derivation
    // Topics will be populated from include_topics below
  } else {
    // Comma-separated list of topics
    std::istringstream iss(config_.topics);
    std::string topic;
    while (std::getline(iss, topic, ',')) {
      // Trim whitespace
      topic.erase(0, topic.find_first_not_of(" \t"));
      topic.erase(topic.find_last_not_of(" \t") + 1);
      if (!topic.empty()) {
        topics_set.insert(topic);
      }
    }
  }

  // Add include_topics
  for (const auto & topic : config_.include_topics) {
    topics_set.insert(topic);
  }

  // Remove exclude_topics
  for (const auto & topic : config_.exclude_topics) {
    topics_set.erase(topic);
  }

  return {topics_set.begin(), topics_set.end()};
}

std::string RosbagCapture::get_topic_type(const std::string & topic) const {
  auto topic_names_and_types = node_->get_topic_names_and_types();
  auto it = topic_names_and_types.find(topic);
  if (it != topic_names_and_types.end() && !it->second.empty()) {
    return it->second[0];
  }
  return "";
}

std::string RosbagCapture::flush_to_bag(const std::string & fault_code) {
  // Copy buffer under lock, then release to avoid holding mutex during IO
  std::deque<BufferedMessage> messages_to_write;
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    if (message_buffer_.empty()) {
      RCLCPP_WARN(node_->get_logger(), "Buffer is empty, cannot create bag file");
      return "";
    }
    messages_to_write = std::move(message_buffer_);
    message_buffer_.clear();
  }

  std::string bag_path = generate_bag_path(fault_code);

  try {
    // Create parent directory if needed
    std::filesystem::path bag_dir(bag_path);
    if (!bag_dir.parent_path().empty()) {
      std::filesystem::create_directories(bag_dir.parent_path());
    }

    // Create writer and store as member for post-fault recording
    {
      std::lock_guard<std::mutex> wlock(writer_mutex_);
      active_writer_ = std::make_unique<rosbag2_cpp::Writer>();
      created_topics_.clear();

      rosbag2_storage::StorageOptions storage_options;
      storage_options.uri = bag_path;
      storage_options.storage_id = config_.format;
      storage_options.max_bagfile_size = config_.max_bag_size_mb * 1024 * 1024;

      active_writer_->open(storage_options);
    }

    // Write messages (no buffer_mutex_ held, writer_mutex_ only for brief access)
    size_t msg_count = 0;
    for (const auto & msg : messages_to_write) {
      std::lock_guard<std::mutex> wlock(writer_mutex_);
      if (!active_writer_) {
        break;
      }

      // Create topic if not already created
      if (created_topics_.find(msg.topic) == created_topics_.end()) {
        rosbag2_storage::TopicMetadata topic_meta;
        topic_meta.name = msg.topic;
        topic_meta.type = msg.message_type;
        topic_meta.serialization_format = "cdr";
        active_writer_->create_topic(topic_meta);
        created_topics_.insert(msg.topic);
      }

      auto bag_msg = create_bag_message(msg.topic, msg.timestamp_ns, msg.serialized_data->get_rcl_serialized_message(),
                                        node_->get_logger());
      if (bag_msg) {
        active_writer_->write(bag_msg);
        ++msg_count;
      }
      // Memory is automatically cleaned up by RAII when bag_msg goes out of scope
    }

    RCLCPP_DEBUG(node_->get_logger(), "Flushed %zu messages to bag: %s", msg_count, bag_path.c_str());

    // Note: Writer is NOT closed here - it stays open for post-fault recording
    // It will be closed in post_fault_timer_callback()

    return bag_path;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to write bag file '%s': %s", bag_path.c_str(), e.what());

    // Clean up writer and partial bag file
    {
      std::lock_guard<std::mutex> wlock(writer_mutex_);
      active_writer_.reset();
      created_topics_.clear();
    }
    std::error_code ec;
    std::filesystem::remove_all(bag_path, ec);

    return "";
  }
}

std::string RosbagCapture::generate_bag_path(const std::string & fault_code) const {
  std::string base_path;

  if (config_.storage_path.empty()) {
    // Use system temp directory
    base_path = std::filesystem::temp_directory_path().string();
  } else {
    base_path = config_.storage_path;
  }

  // Create unique name with timestamp
  auto now = std::chrono::system_clock::now();
  auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

  std::ostringstream oss;
  oss << base_path << "/fault_" << fault_code << "_" << timestamp;

  return oss.str();
}

size_t RosbagCapture::calculate_bag_size(const std::string & bag_path) const {
  size_t total_size = 0;

  try {
    if (std::filesystem::is_directory(bag_path)) {
      for (const auto & entry : std::filesystem::recursive_directory_iterator(bag_path)) {
        if (entry.is_regular_file()) {
          total_size += entry.file_size();
        }
      }
    } else if (std::filesystem::is_regular_file(bag_path)) {
      total_size = std::filesystem::file_size(bag_path);
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(node_->get_logger(), "Failed to calculate bag size for '%s': %s", bag_path.c_str(), e.what());
  }

  return total_size;
}

void RosbagCapture::enforce_storage_limits() {
  size_t max_bytes = config_.max_total_storage_mb * 1024 * 1024;
  size_t current_bytes = storage_->get_total_rosbag_storage_bytes();

  if (current_bytes <= max_bytes) {
    return;
  }

  // Get all bags sorted by creation time (oldest first)
  auto all_bags = storage_->get_all_rosbag_files();

  for (const auto & bag : all_bags) {
    if (current_bytes <= max_bytes) {
      break;
    }

    RCLCPP_INFO(node_->get_logger(), "Deleting old bag file for fault '%s' to enforce storage limit",
                bag.fault_code.c_str());

    current_bytes -= bag.size_bytes;
    storage_->delete_rosbag_file(bag.fault_code);
  }
}

void RosbagCapture::post_fault_timer_callback() {
  if (!recording_post_fault_.load()) {
    return;
  }

  // Cancel timer (one-shot)
  if (post_fault_timer_) {
    post_fault_timer_->cancel();
    post_fault_timer_.reset();
  }

  // Stop post-fault recording (no more direct writes to bag)
  recording_post_fault_.store(false);

  // Close the writer (messages were written directly during post-fault period)
  {
    std::lock_guard<std::mutex> wlock(writer_mutex_);
    active_writer_.reset();
    created_topics_.clear();
  }

  // Calculate final size and store metadata
  size_t bag_size = calculate_bag_size(current_bag_path_);

  RosbagFileInfo info;
  info.fault_code = current_fault_code_;
  info.file_path = current_bag_path_;
  info.format = config_.format;
  info.duration_sec = config_.duration_sec + config_.duration_after_sec;
  info.size_bytes = bag_size;
  info.created_at_ns = get_wall_clock_ns();

  storage_->store_rosbag_file(info);
  enforce_storage_limits();

  RCLCPP_INFO(node_->get_logger(), "Bag file completed: %s (%.2f MB, %.1fs)", current_bag_path_.c_str(),
              static_cast<double>(bag_size) / (1024.0 * 1024.0), info.duration_sec);

  current_fault_code_.clear();
  current_bag_path_.clear();
}

void RosbagCapture::validate_storage_format() const {
  // Validate format is one of the known options
  if (config_.format != "sqlite3" && config_.format != "mcap") {
    throw std::runtime_error("Invalid rosbag storage format '" + config_.format +
                             "'. "
                             "Valid options: 'sqlite3', 'mcap'");
  }

  // sqlite3 is always available (built into rosbag2)
  if (config_.format == "sqlite3") {
    return;
  }

  // For MCAP, verify the plugin is available by trying to create a test bag
  if (config_.format == "mcap") {
    std::string test_path =
        std::filesystem::temp_directory_path().string() + "/.rosbag_mcap_test_" + std::to_string(getpid());

    try {
      rosbag2_cpp::Writer writer;
      rosbag2_storage::StorageOptions opts;
      opts.uri = test_path;
      opts.storage_id = "mcap";
      writer.open(opts);
      // Success - plugin is available, clean up test file
    } catch (const std::exception & e) {
      // Clean up any partial test files
      std::error_code ec;
      std::filesystem::remove_all(test_path, ec);

      throw std::runtime_error(
          "MCAP storage format requested but rosbag2_storage_mcap plugin is not available. "
          "Install with: sudo apt install ros-${ROS_DISTRO}-rosbag2-storage-mcap "
          "Or use format: 'sqlite3' (default). "
          "Error: " +
          std::string(e.what()));
    }

    // Clean up test file
    std::error_code ec;
    std::filesystem::remove_all(test_path, ec);

    RCLCPP_INFO(node_->get_logger(), "MCAP storage format validated successfully");
  }
}

}  // namespace ros2_medkit_fault_manager
