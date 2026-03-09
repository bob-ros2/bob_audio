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
    descriptor.description =
      "Path to the output FIFO pipe. If set, FIFO output is enabled. "
      "(Default: '', Env: MIXER_OUTPUT_FIFO).";
    std::string output_fifo_path = this->declare_parameter(
      "output_fifo",
      get_env("MIXER_OUTPUT_FIFO", std::string("")), descriptor);

    descriptor.description =
      "Enable writing mixed audio to stdout for piping "
      "(Default: false, Env: MIXER_ENABLE_STDOUT_OUTPUT).";
    enable_stdout_output_ = this->declare_parameter(
      "enable_stdout_output",
      get_env("MIXER_ENABLE_STDOUT_OUTPUT", false), descriptor);

    descriptor.description =
      "Enable publishing mixed audio to a ROS topic "
      "(Default: true, Env: MIXER_ENABLE_TOPIC_OUTPUT).";
    bool enable_topic_output = this->declare_parameter(
      "enable_topic_output",
      get_env("MIXER_ENABLE_TOPIC_OUTPUT", true), descriptor);

    descriptor.description =
      "Generate silent frames if no input audio is received "
      "(Default: true, Env: MIXER_HEARTBEAT).";
    heartbeat_ = this->declare_parameter(
      "heartbeat",
      get_env("MIXER_HEARTBEAT", true), descriptor);

    // --- Input Parameters ---
    descriptor.description =
      "Enable reading audio from an input FIFO pipe "
      "(Default: false, Env: MIXER_ENABLE_FIFO_INPUT).";
    bool enable_fifo_input = this->declare_parameter(
      "enable_fifo_input",
      get_env("MIXER_ENABLE_FIFO_INPUT", false), descriptor);

    descriptor.description =
      "Path to the input FIFO pipe "
      "(Default: /tmp/audio_pipe, Env: MIXER_INPUT_FIFO).";
    std::string input_fifo_path = this->declare_parameter(
      "input_fifo",
      get_env("MIXER_INPUT_FIFO", std::string("/tmp/audio_pipe")), descriptor);

    // --- Audio Format ---
    descriptor.description = "Audio sample rate (Default: 44100, Env: MIXER_SAMPLE_RATE).";
    sample_rate_ = this->declare_parameter(
      "sample_rate",
      get_env("MIXER_SAMPLE_RATE", 44100), descriptor);

    descriptor.description = "Number of audio channels (Default: 2, Env: MIXER_CHANNELS).";
    channels_ = this->declare_parameter(
      "channels",
      get_env("MIXER_CHANNELS", 2), descriptor);

    descriptor.description = "Chunk size in milliseconds (Default: 20, Env: MIXER_CHUNK_MS).";
    int chunk_ms = this->declare_parameter(
      "chunk_ms",
      get_env("MIXER_CHUNK_MS", 20), descriptor);

    descriptor.description = "Number of input topics (in0, in1, ...) (Default: 4).";
    input_count_ = this->declare_parameter("input_count", 4, descriptor);

    // --- Internal State ---
    samples_per_chunk_ = (sample_rate_ * chunk_ms) / 1000;
    values_per_chunk_ = samples_per_chunk_ * channels_;

    if (!output_fifo_path.empty()) {
      mkfifo(output_fifo_path.c_str(), 0666);
      output_fifo_fd_ = open(output_fifo_path.c_str(), O_RDWR | O_NONBLOCK);
      if (output_fifo_fd_ < 0) {
        RCLCPP_ERROR(
          this->get_logger(), "Failed to open output FIFO: %s",
          output_fifo_path.c_str());
      } else {
        RCLCPP_INFO(
          this->get_logger(), "Output FIFO opened (Non-Blocking): %s",
          output_fifo_path.c_str());
      }
    }


    if (enable_fifo_input) {
      mkfifo(input_fifo_path.c_str(), 0666);
      // Open O_RDWR to prevent EOF when no one is writing, but O_NONBLOCK for the thread
      input_fifo_fd_ = open(input_fifo_path.c_str(), O_RDWR | O_NONBLOCK);
      if (input_fifo_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open input FIFO: %s", input_fifo_path.c_str());
      } else {
        RCLCPP_INFO(this->get_logger(), "Input FIFO opened (Buffered/Non-Blocking): %s", input_fifo_path.c_str());
        fifo_input_thread_ = std::thread(&MixerNode::fifo_input_loop, this);
      }
    }

    if (enable_topic_output) {
      pub_ = this->create_publisher<std_msgs::msg::Int16MultiArray>("out", 10);
    }

    // --- Callback Groups for Multithreading ---
    // Subscribers get a reentrant group (can run in parallel)
    cb_group_subs_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    // Timer gets a mutually exclusive group (prevent overlapping loops)
    cb_group_timer_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    auto sub_options = rclcpp::SubscriptionOptions();
    sub_options.callback_group = cb_group_subs_;

    // Create input subscribers
    for (int i = 0; i < input_count_; ++i) {
      std::string topic = "in" + std::to_string(i);

      descriptor.description = "Number of channels for " + topic + " (Default: same as mixer).";
      int in_ch = this->declare_parameter(topic + "_channels", channels_, descriptor);
      input_topic_channels_.push_back(in_ch);

      subs_.push_back(
        this->create_subscription<std_msgs::msg::Int16MultiArray>(
          topic, 10,
          [this, i](const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
            this->topic_callback(i, msg);
          }, sub_options));
      input_buffers_.emplace_back();
      input_gains_.push_back(1.0f);
      input_active_.push_back(false);
    }

    // Dynamic Control Subscriber
    control_sub_ = this->create_subscription<std_msgs::msg::String>(
      "control", 10,
      std::bind(&MixerNode::control_callback, this, std::placeholders::_1), sub_options);

    // Timer for mixing loop
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(chunk_ms),
      std::bind(&MixerNode::mix_loop, this),
      cb_group_timer_);

    RCLCPP_INFO(
      this->get_logger(), "Mixer Node started. Rate: %dHz, Channels: %d, Chunk: %dms, Inputs: %d",
      sample_rate_, channels_, chunk_ms, input_count_);
  }

  ~MixerNode()
  {
    running_ = false;
    if (fifo_input_thread_.joinable()) {
      fifo_input_thread_.join();
    }
    if (output_fifo_fd_ >= 0) {close(output_fifo_fd_);}
    if (input_fifo_fd_ >= 0) {close(input_fifo_fd_);}
  }

private:
  void fifo_input_loop()
  {
    std::vector<int16_t> read_buffer(2048);
    while (rclcpp::ok() && running_) {
      if (input_fifo_fd_ < 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }

      ssize_t bytes_read = read(input_fifo_fd_, read_buffer.data(), read_buffer.size() * 2);
      if (bytes_read > 0) {
        size_t samples_read = bytes_read / 2;
        std::lock_guard<std::mutex> lock(fifo_mutex_);
        for (size_t i = 0; i < samples_read; ++i) {
          fifo_input_buffer_.push_back(read_buffer[i]);
        }
        // Cap buffer to 300 seconds (prevent OOM)
        size_t max_fifo_samples = sample_rate_ * channels_ * 300.0;
        if (fifo_input_buffer_.size() > max_fifo_samples) {
          fifo_input_buffer_.clear();
          fifo_active_ = false;
          RCLCPP_WARN(this->get_logger(), "FIFO buffer overflow, clearing.");
        }
      } else {
        // Sleep slightly longer when empty to save CPU
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    }
  }

  void topic_callback(int index, const std_msgs::msg::Int16MultiArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & val : msg->data) {
      input_buffers_[index].push_back(val);
    }

    // Safety limit: 300 seconds of audio. Clearing if exceeded (emergency brake).
    size_t max_topic_samples = sample_rate_ * input_topic_channels_[index] * 300.0;
    if (input_buffers_[index].size() > max_topic_samples) {
      input_buffers_[index].clear();
      input_active_[index] = false;
      RCLCPP_WARN(this->get_logger(), "Buffer overflow for input %d, clearing to prevent OOM.", index);
    }
  }

  void control_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    try {
      auto j = nlohmann::json::parse(msg->data);
      std::lock_guard<std::mutex> lock(mutex_);

      for (int i = 0; i < input_count_; ++i) {
        std::string base_key = "in" + std::to_string(i);

        // --- Structured Object Support ---
        // Example: {"in0": {"gain": 0.5, "channels": 1}}
        if (j.contains(base_key) && j[base_key].is_object()) {
          if (j[base_key].contains("gain") && j[base_key]["gain"].is_number()) {
            input_gains_[i] = j[base_key]["gain"].get<float>();
            RCLCPP_INFO(this->get_logger(), "Set gain for in%d to %.2f", i, input_gains_[i]);
          }
          if (j[base_key].contains("channels") && j[base_key]["channels"].is_number()) {
            input_topic_channels_[i] = j[base_key]["channels"].get<int>();
            RCLCPP_INFO(
              this->get_logger(), "Set channels for in%d to %d", i,
              input_topic_channels_[i]);
          }
        }

        // --- Flat Key Support ---
        // Example: {"in0_gain": 0.5, "in0_channels": 1}
        std::string gain_key = base_key + "_gain";
        if (j.contains(gain_key) && j[gain_key].is_number()) {
          input_gains_[i] = j[gain_key].get<float>();
          RCLCPP_INFO(this->get_logger(), "Set gain for in%d to %.2f", i, input_gains_[i]);
        }
        std::string chan_key = base_key + "_channels";
        if (j.contains(chan_key) && j[chan_key].is_number()) {
          input_topic_channels_[i] = j[chan_key].get<int>();
          RCLCPP_INFO(
            this->get_logger(), "Set channels for in%d to %d", i,
            input_topic_channels_[i]);
        }
      }

      // Master and FIFO gains
      if (j.contains("fifo") && j["fifo"].is_number()) {
        fifo_gain_ = j["fifo"].get<float>();
        RCLCPP_INFO(this->get_logger(), "Set gain for fifo to %.2f", fifo_gain_);
      }
      if (j.contains("master") && j["master"].is_number()) {
        master_gain_ = j["master"].get<float>();
        RCLCPP_INFO(this->get_logger(), "Set master gain to %.2f", master_gain_);
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "Failed to parse control JSON: %s", e.what());
    }
  }

  void mix_loop()
  {
    std::vector<int32_t> mixed_data(values_per_chunk_, 0);
    bool has_data = false;

    // 1. Collect from ROS topics (locked only during buffer access)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (int i = 0; i < input_count_; ++i) {
        int in_ch = input_topic_channels_[i];
        float gain = input_gains_[i];
        size_t required = (in_ch == 1 && channels_ == 2) ?
          static_cast<size_t>(samples_per_chunk_) : static_cast<size_t>(values_per_chunk_);

        // Startup Cushion: Wait for some data to accumulate before starting
        if (!input_active_[i]) {
          if (input_buffers_[i].size() >= required * 4) {
            input_active_[i] = true;
          } else {
            continue;
          }
        }

        // Pull samples if enough data is available
        if (input_buffers_[i].size() >= required) {
          has_data = true;
          if (in_ch == 1 && channels_ == 2) {
            // Upmix Mono to Stereo
            for (size_t j = 0; j < required; ++j) {
              int16_t sample = input_buffers_[i].front();
              input_buffers_[i].pop_front();
              int32_t val = static_cast<int32_t>(sample * gain);
              mixed_data[j * 2] += val;
              mixed_data[j * 2 + 1] += val;
            }
          } else {
            // Same channel count
            for (size_t j = 0; j < required; ++j) {
              mixed_data[j] += static_cast<int32_t>(input_buffers_[i].front() * gain);
              input_buffers_[i].pop_front();
            }
          }
        } else {
          // Reset status if truly empty
          if (input_buffers_[i].empty()) {
            input_active_[i] = false;
          }
        }
      }
    }

    // 2. Collect from input FIFO buffer (locked only during buffer access)
    if (input_fifo_fd_ >= 0) {
      std::lock_guard<std::mutex> lock(fifo_mutex_);

      // Startup Cushion for FIFO: 10 chunks (200ms) for music stability
      if (!fifo_active_) {
        if (fifo_input_buffer_.size() >= static_cast<size_t>(values_per_chunk_) * 10) {
          fifo_active_ = true;
        }
      }

      if (fifo_active_) {
        if (fifo_input_buffer_.size() >= static_cast<size_t>(values_per_chunk_)) {
          has_data = true;
          for (size_t j = 0; j < static_cast<size_t>(values_per_chunk_); ++j) {
            mixed_data[j] += static_cast<int32_t>(fifo_input_buffer_.front() * fifo_gain_);
            fifo_input_buffer_.pop_front();
          }
        } else {
          // Soft dropout: if we have some data but not enough for a chunk, 
          // we just wait WITHOUT resetting the active flag immediately.
          // Only reset if truly empty.
          if (fifo_input_buffer_.empty()) {
            fifo_active_ = false;
          }
        }
      }
    }

    // Exit early if no data and no heartbeat to avoid overhead
    if (!has_data && !heartbeat_) {
      return;
    }

    // --- Post-Processing (UNLOCKED) ---
    std::vector<int16_t> final_data(values_per_chunk_);
    for (int i = 0; i < values_per_chunk_; ++i) {
      int32_t val = static_cast<int32_t>(mixed_data[i] * master_gain_);
      if (val > 32767) {val = 32767;} else if (val < -32768) {val = -32768;}
      final_data[i] = static_cast<int16_t>(val);
    }

    // --- Outputs (UNLOCKED) ---
    if (pub_) {
      auto msg = std_msgs::msg::Int16MultiArray();
      msg.data = final_data;
      pub_->publish(msg);
    }

    if (output_fifo_fd_ >= 0) {
      if (write(output_fifo_fd_, final_data.data(), values_per_chunk_ * 2) < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
          RCLCPP_DEBUG(this->get_logger(), "FIFO write error: %s", strerror(errno));
        }
      }
    }

    if (enable_stdout_output_) {
      write(STDOUT_FILENO, final_data.data(), values_per_chunk_ * 2);
    }

    static int loop_count = 0;
    if (++loop_count % 100 == 0) {
      RCLCPP_DEBUG(
        this->get_logger(), "Mixing... Buffer in0: %zu samples",
        input_buffers_[0].size());
    }
  }

  int sample_rate_;
  int channels_;
  int samples_per_chunk_;
  int values_per_chunk_;
  int input_count_;
  bool heartbeat_;

  int output_fifo_fd_ = -1;
  int input_fifo_fd_ = -1;
  bool enable_stdout_output_ = false;

  std::vector<rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr> subs_;
  std::vector<int> input_topic_channels_;
  std::vector<std::deque<int16_t>> input_buffers_;
  std::vector<float> input_gains_;
  std::vector<bool> input_active_;
  float fifo_gain_ = 1.0f;
  bool fifo_active_ = false;
  float master_gain_ = 1.0f;
  std::mutex mutex_;

  rclcpp::CallbackGroup::SharedPtr cb_group_subs_;
  rclcpp::CallbackGroup::SharedPtr cb_group_timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_sub_;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::thread fifo_input_thread_;
  std::deque<int16_t> fifo_input_buffer_;
  std::mutex fifo_mutex_;
  std::atomic<bool> running_{true};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MixerNode>();

  // Use MultiThreadedExecutor with 4 threads for parallel callback processing
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
