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
 * @file mixer_node.cpp
 * @brief ROS 2 node for high-performance audio mixing.
 */

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <csignal>
#include <cstdlib>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

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

class MixerNode : public rclcpp::Node
{
public:
  MixerNode()
  : Node("mixer")
  {
    // Ignore SIGPIPE to prevent crashing if a pipe reader disconnects
    signal(SIGPIPE, SIG_IGN);

    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

    // --- Output Parameters ---
    descriptor.description = "Enable FIFO output (Default: true).";
    bool enable_fifo_output = this->declare_parameter(
      "enable_fifo_output", get_env("MIXER_ENABLE_FIFO_OUTPUT", true), descriptor);

    descriptor.description = "Output FIFO path.";
    std::string default_pipe = "/tmp/audio_master_pipe";
    std::string output_fifo_path = this->declare_parameter(
      "output_fifo", get_env("MIXER_OUTPUT_FIFO", default_pipe), descriptor);

    // --- Audio Format ---
    // TTS node defaults to 44100 Hz output
    sample_rate_ = this->declare_parameter(
      "sample_rate", get_env("MIXER_SAMPLE_RATE", 44100), descriptor);
    channels_ = this->declare_parameter(
      "channels", get_env("MIXER_CHANNELS", 1), descriptor);
    int chunk_ms = this->declare_parameter(
      "chunk_ms", get_env("MIXER_CHUNK_MS", 20), descriptor);

    values_per_chunk_ = (sample_rate_ * chunk_ms * channels_) / 1000;

    if (enable_fifo_output) {
      mkfifo(output_fifo_path.c_str(), 0666);
      output_fifo_fd_ = open(output_fifo_path.c_str(), O_RDWR);
      if (output_fifo_fd_ < 0) {
        RCLCPP_ERROR(
          this->get_logger(), "Failed to open output FIFO: %s",
          output_fifo_path.c_str());
      } else {
        RCLCPP_INFO(
          this->get_logger(), "Output FIFO opened (Blocking): %s",
          output_fifo_path.c_str());
      }
    }

    // Subscribers
    for (int i = 0; i < 8; ++i) {
      std::string topic = "in" + std::to_string(i);
      subs_.push_back(
        this->create_subscription<std_msgs::msg::Int16MultiArray>(
          topic, 10,
          [this, i](const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_);
            for (auto v : msg->data) {
              input_buffers_[i].push_back(v);
            }
          }));
      input_buffers_.emplace_back();
      input_gains_.push_back(1.0f);
    }

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(chunk_ms),
      std::bind(&MixerNode::mix_loop, this));

    RCLCPP_INFO(
      this->get_logger(), "Mixer started: %dHz, %dch, %dms",
      sample_rate_, channels_, chunk_ms);
  }

  ~MixerNode()
  {
    if (output_fifo_fd_ >= 0) {
      close(output_fifo_fd_);
    }
  }

private:
  void mix_loop()
  {
    std::vector<int32_t> mixed_data(values_per_chunk_, 0);
    bool any_source_ready = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      // SOLID CHUNK PROTECTION: Only process if at least one buffer has a full chunk
      for (int i = 0; i < 8; ++i) {
        if (input_buffers_[i].size() >= static_cast<size_t>(values_per_chunk_)) {
          any_source_ready = true;
          break;
        }
      }

      if (!any_source_ready) {
        return;  // Wait for real audio data
      }

      for (int i = 0; i < 8; ++i) {
        size_t available = input_buffers_[i].size();
        size_t to_pull = std::min(available, static_cast<size_t>(values_per_chunk_));
        float gain = input_gains_[i];
        for (size_t j = 0; j < to_pull; ++j) {
          mixed_data[j] += static_cast<int32_t>(input_buffers_[i].front() * gain);
          input_buffers_[i].pop_front();
        }
      }
    }

    std::vector<int16_t> final_data(values_per_chunk_);
    for (int i = 0; i < values_per_chunk_; ++i) {
      int32_t val = static_cast<int32_t>(mixed_data[i] * master_gain_);
      final_data[i] = static_cast<int16_t>(std::max(-32768, std::min(32767, val)));
    }

    if (output_fifo_fd_ >= 0) {
      write(output_fifo_fd_, final_data.data(), values_per_chunk_ * 2);
    }
  }

  int sample_rate_, channels_, values_per_chunk_;
  int output_fifo_fd_ = -1;
  std::vector<rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr> subs_;
  std::vector<std::deque<int16_t>> input_buffers_;
  std::vector<float> input_gains_;
  float master_gain_ = 1.0f;
  std::mutex mutex_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MixerNode>());
  rclcpp::shutdown();
  return 0;
}
