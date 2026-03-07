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

    // --- Input Configuration (FIFO/stdin -> ROS) ---
    descriptor.description =
      "Enable reading from an input FIFO "
      "(Default: false, Env: CONVERT_ENABLE_FIFO).";
    bool enable_fifo = this->declare_parameter(
      "enable_fifo",
      get_env("CONVERT_ENABLE_FIFO", false), descriptor);

    descriptor.description =
      "Path to the input FIFO "
      "(Default: /tmp/audio_pipe, Env: CONVERT_INPUT_FIFO).";
    std::string input_fifo_path = this->declare_parameter(
      "input_fifo",
      get_env("CONVERT_INPUT_FIFO", std::string("/tmp/audio_pipe")), descriptor);

    descriptor.description =
      "Enable reading from stdin (Pipe) "
      "(Default: false, Env: CONVERT_ENABLE_STDIN).";
    bool enable_stdin = this->declare_parameter(
      "enable_stdin",
      get_env("CONVERT_ENABLE_STDIN", false), descriptor);

    // --- Output Configuration (ROS -> FIFO) ---
    descriptor.description =
      "Enable writing ROS messages to an output FIFO "
      "(Default: false, Env: CONVERT_ENABLE_FIFO_OUTPUT).";
    bool enable_fifo_output = this->declare_parameter(
      "enable_fifo_output",
      get_env("CONVERT_ENABLE_FIFO_OUTPUT", false), descriptor);

    descriptor.description =
      "Path to the output FIFO "
      "(Default: /tmp/audio_out_pipe, Env: CONVERT_OUTPUT_FIFO).";
    std::string output_fifo_path = this->declare_parameter(
      "output_fifo",
      get_env("CONVERT_OUTPUT_FIFO", std::string("/tmp/audio_out_pipe")), descriptor);

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

    values_per_chunk_ = (sample_rate * chunk_ms / 1000) * channels;

    // --- Setup Input (FIFO/stdin -> ROS) ---
    pub_ = this->create_publisher<std_msgs::msg::Int16MultiArray>("out", 10);

    if (enable_fifo) {
      mkfifo(input_fifo_path.c_str(), 0666);
      input_fd_ = open(input_fifo_path.c_str(), O_RDONLY | O_NONBLOCK);
      if (input_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open input FIFO: %s", input_fifo_path.c_str());
      } else {
        RCLCPP_INFO(this->get_logger(), "Reading from input FIFO: %s", input_fifo_path.c_str());
      }
    } else if (enable_stdin) {
      input_fd_ = STDIN_FILENO;
      RCLCPP_INFO(this->get_logger(), "Reading from stdin (Pipe).");
    }

    if (input_fd_ >= 0) {
      read_thread_ = std::thread(&ConvertNode::read_loop, this);
    }

    // --- Setup Output (ROS -> FIFO) ---
    if (enable_fifo_output) {
      mkfifo(output_fifo_path.c_str(), 0666);
      output_fd_ = open(output_fifo_path.c_str(), O_RDWR | O_NONBLOCK);
      if (output_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open output FIFO: %s", output_fifo_path.c_str());
      } else {
        RCLCPP_INFO(this->get_logger(), "Writing to output FIFO: %s", output_fifo_path.c_str());
        sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
          "in", 10, std::bind(&ConvertNode::write_callback, this, std::placeholders::_1));
      }
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
      ssize_t written = write(output_fd_, msg->data.data(), msg->data.size() * sizeof(int16_t));
      if (written < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
          RCLCPP_DEBUG(this->get_logger(), "Write error to output FIFO: %s", strerror(errno));
        }
      }
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
        RCLCPP_DEBUG(this->get_logger(), "Published %zu samples to ROS topic", total_read / sizeof(int16_t));
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
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ConvertNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
