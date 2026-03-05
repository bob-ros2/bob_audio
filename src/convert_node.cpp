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
  : Node("convert_node")
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

    // --- Input Configuration ---
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

    pub_ = this->create_publisher<std_msgs::msg::Int16MultiArray>("out", 10);

    if (enable_fifo) {
      mkfifo(input_fifo_path.c_str(), 0666);
      input_fd_ = open(input_fifo_path.c_str(), O_RDONLY | O_NONBLOCK);
      if (input_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open FIFO: %s", input_fifo_path.c_str());
      } else {
        RCLCPP_INFO(this->get_logger(), "Reading from FIFO: %s", input_fifo_path.c_str());
      }
    } else if (enable_stdin) {
      input_fd_ = STDIN_FILENO;
      RCLCPP_INFO(this->get_logger(), "Reading from stdin (Pipe).");
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "No input source enabled! Use CONVERT_FIFO_ENABLE or CONVERT_STDIN_ENABLE.");
    }

    if (input_fd_ >= 0) {
      read_thread_ = std::thread(&ConvertNode::read_loop, this);
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
  }

private:
  void read_loop()
  {
    std::vector<int16_t> buffer(values_per_chunk_);
    size_t chunk_bytes = values_per_chunk_ * 2;

    while (rclcpp::ok() && running_) {
      ssize_t bytes_read = read(input_fd_, buffer.data(), chunk_bytes);

      if (bytes_read > 0) {
        auto msg = std_msgs::msg::Int16MultiArray();
        size_t samples = bytes_read / 2;
        msg.data.assign(buffer.begin(), buffer.begin() + samples);
        pub_->publish(msg);
      } else if (bytes_read == 0) {
        // EOF, might happen on pipe. Wait a bit or exit?
        // On FIFO, read 0 means no writer. We wait.
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      } else {
        // Error or EAGAIN
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        } else {
          RCLCPP_ERROR(this->get_logger(), "Read error: %s", strerror(errno));
          break;
        }
      }
    }
  }

  int input_fd_ = -1;
  int values_per_chunk_ = 0;
  bool running_ = true;
  std::thread read_thread_;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ConvertNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
