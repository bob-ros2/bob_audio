// Copyright 2026 Bob Ros
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

/**
 * @file convert_node.cpp
 * @brief ROS 2 node to convert raw audio streams (FIFO/Pipe) into ROS messages.
 */

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

/**
 * @brief Gets an environment variable as a std::string.
 */
static std::string get_env(const char * name, const std::string & default_val)
{
  const char * val = std::getenv(name);
  return val ? std::string(val) : default_val;
}

/**
 * @brief Gets an environment variable as a bool.
 */
static bool get_env(const char * name, bool default_val)
{
  const char * val = std::getenv(name);
  if (!val) {
    return default_val;
  }
  std::string s(val);
  std::transform(
    s.begin(), s.end(), s.begin(),
    [](unsigned char c) {return std::tolower(c);});
  return s == "true" || s == "1" || s == "yes" || s == "on";
}

/**
 * @brief Gets an environment variable as an int.
 */
static int get_env(const char * name, int default_val)
{
  const char * val = std::getenv(name);
  if (!val) {
    return default_val;
  }
  try {
    return std::stoi(val);
  } catch (...) {
    return default_val;
  }
}

class ConvertNode : public rclcpp::Node
{
public:
  ConvertNode()
  : Node("convert")
  {
    // Ignore SIGPIPE to prevent crashing if a pipe reader disconnects
    signal(SIGPIPE, SIG_IGN);

    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

    // --- Mode Configuration ---
    descriptor.description =
      "Conversion mode: 'fifo_to_ros', 'stdin_to_ros', or 'ros_to_fifo' "
      "(Default: fifo_to_ros, Env: CONVERT_MODE).";
    std::string mode = this->declare_parameter(
      "mode",
      get_env("CONVERT_MODE", std::string("fifo_to_ros")), descriptor);

    descriptor.description =
      "Path to the FIFO pipe (used for fifo_to_ros and ros_to_fifo) "
      "(Default: /tmp/audio_pipe, Env: CONVERT_FIFO_PATH).";
    std::string fifo_path = this->declare_parameter(
      "fifo_path",
      get_env("CONVERT_FIFO_PATH", std::string("/tmp/audio_pipe")), descriptor);

    // --- Audio Format ---
    descriptor.description =
      "Audio sample rate "
      "(Default: 44100, Env: CONVERT_SAMPLE_RATE).";
    int sample_rate = this->declare_parameter(
      "sample_rate",
      get_env("CONVERT_SAMPLE_RATE", 44100), descriptor);

    descriptor.description =
      "Number of audio channels "
      "(Default: 2, Env: CONVERT_CHANNELS).";
    int channels = this->declare_parameter(
      "channels",
      get_env("CONVERT_CHANNELS", 2), descriptor);

    descriptor.description =
      "Chunk size in milliseconds per message "
      "(Default: 20, Env: CONVERT_CHUNK_MS).";
    int chunk_ms = this->declare_parameter(
      "chunk_ms",
      get_env("CONVERT_CHUNK_MS", 20), descriptor);

    descriptor.description =
      "Generate silent frames if no ROS audio is received (for ros_to_fifo) "
      "(Default: true, Env: CONVERT_HEARTBEAT).";
    bool enable_heartbeat = this->declare_parameter(
      "heartbeat",
      get_env("CONVERT_HEARTBEAT", true), descriptor);

    values_per_chunk_ = (sample_rate * chunk_ms / 1000) * channels;

    // --- Mode Execution ---
    if (mode == "fifo_to_ros") {
      pub_ = this->create_publisher<std_msgs::msg::Int16MultiArray>("out", 10);
      mkfifo(fifo_path.c_str(), 0666);
      input_fd_ = open(fifo_path.c_str(), O_RDONLY | O_NONBLOCK);
      if (input_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open input FIFO: %s", fifo_path.c_str());
      } else {
        RCLCPP_INFO(this->get_logger(), "Mode: FIFO -> ROS. FIFO: %s", fifo_path.c_str());
        read_thread_ = std::thread(&ConvertNode::read_loop, this);
      }
    } else if (mode == "stdin_to_ros") {
      pub_ = this->create_publisher<std_msgs::msg::Int16MultiArray>("out", 10);
      input_fd_ = STDIN_FILENO;
      RCLCPP_INFO(this->get_logger(), "Mode: stdin -> ROS.");
      read_thread_ = std::thread(&ConvertNode::read_loop, this);
    } else if (mode == "ros_to_fifo") {
      mkfifo(fifo_path.c_str(), 0666);
      output_fd_ = open(fifo_path.c_str(), O_RDWR | O_NONBLOCK);
      if (output_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open output FIFO: %s", fifo_path.c_str());
      } else {
        RCLCPP_INFO(this->get_logger(), "Mode: ROS -> FIFO. FIFO: %s", fifo_path.c_str());
        sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
          "in", 10, std::bind(&ConvertNode::write_callback, this, std::placeholders::_1));

        if (enable_heartbeat) {
          last_audio_time_ = this->now();
          heartbeat_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(chunk_ms),
            std::bind(&ConvertNode::heartbeat_callback, this));
          RCLCPP_INFO(this->get_logger(), "Heartbeat (silence) enabled: %d ms", chunk_ms);
        }
      }
    } else {
      RCLCPP_ERROR(
        this->get_logger(), "FATAL: Invalid mode '%s'. Check CONVERT_MODE.",
        mode.c_str());
    }
  }

  ~ConvertNode()
  {
    running_ = false;
    if (read_thread_.joinable()) {
      read_thread_.join();
    }
    if (input_fd_ >= 0 && input_fd_ != STDIN_FILENO) {
      close(input_fd_);
    }
    if (output_fd_ >= 0) {
      close(output_fd_);
    }
  }

private:
  void write_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
  {
    if (output_fd_ >= 0 && !msg->data.empty()) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        last_audio_time_ = this->now();
      }
      ssize_t written = write(output_fd_, msg->data.data(), msg->data.size() * sizeof(int16_t));
      if (written < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
          RCLCPP_DEBUG(this->get_logger(), "Write error to output FIFO: %s", strerror(errno));
        }
      }
    }
  }

  void heartbeat_callback()
  {
    if (output_fd_ < 0) {return;}

    rclcpp::Time last;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      last = last_audio_time_;
    }

    auto now = this->now();
    // If no audio for > 2x chunk duration, send a silent chunk to keep FFmpeg running
    if ((now - last).nanoseconds() > (values_per_chunk_ * 1000000000LL / 44100 / 2)) {
      std::vector<int16_t> silence(values_per_chunk_, 0);
      write(output_fd_, silence.data(), silence.size() * sizeof(int16_t));
    }
  }

  void read_loop()
  {
    std::vector<int16_t> buffer(values_per_chunk_);
    size_t chunk_bytes = values_per_chunk_ * sizeof(int16_t);

    while (rclcpp::ok() && running_) {
      size_t total_read = 0;
      uint8_t * ptr = reinterpret_cast<uint8_t *>(buffer.data());

      // Read until we have a full chunk or an error occurs
      while (total_read < chunk_bytes && rclcpp::ok() && running_) {
        ssize_t n = read(input_fd_, ptr + total_read, chunk_bytes - total_read);
        if (n > 0) {
          total_read += n;
        } else if (n == 0) {
          // EOF
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
          if (total_read == 0) {goto next_loop;} else {break;}
        } else {
          if (errno == EAGAIN || errno == EWOULDBLOCK) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
          }
          RCLCPP_ERROR(this->get_logger(), "Read error: %s", strerror(errno));
          return;
        }
      }

      if (total_read > 0) {
        auto msg = std_msgs::msg::Int16MultiArray();
        msg.data.assign(buffer.begin(), buffer.begin() + (total_read / sizeof(int16_t)));
        pub_->publish(msg);
        RCLCPP_DEBUG(
          this->get_logger(), "Published %zu samples to ROS topic",
          total_read / sizeof(int16_t));
      }

next_loop:
      continue;
    }
  }

  int input_fd_ = -1;
  int output_fd_ = -1;
  int values_per_chunk_ = 0;
  bool running_ = true;
  std::thread read_thread_;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sub_;

  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::Time last_audio_time_;
  std::mutex mutex_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ConvertNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
