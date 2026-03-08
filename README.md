# ROS Package [bob_audio](https://github.com/bob-ros2/bob_audio)

[![ROS 2 CI](https://github.com/bob-ros2/bob_audio/actions/workflows/ros_ci.yml/badge.svg)](https://github.com/bob-ros2/bob_audio/actions/workflows/ros_ci.yml)

A high-performance ROS 2 package for audio mixing and conversion, optimized for Docker and headless environments. It allows combining multiple audio sources (TTS, Music, SFX) into a single master stream without the need for a full audio server like PulseAudio or JACK.

## Features

- **High Performance**: Written in C++ with minimal overhead.
- **Flexible Mixing**: Mix up to 8 ROS topics (via `Int16MultiArray`) and optional FIFO inputs.
- **Real-time Volume Control**: Change gain for each input or master via JSON.
- **Docker Friendly**: No hardware audio device requirements.
- **Multiple Output Sinks**: Stream to ROS topics, FIFO pipes, or directly to stdout (for FFmpeg).
- **Audio Conversion**: Bidirectional conversion between raw streams and ROS messages.

## Installation & Build

### Prerequisites

- ROS 2 Humble (or newer)
- `std_msgs`, `nlohmann_json`

### Build

```bash
# Navigate to your workspace
cd ~/ros2_ws/src
git clone https://github.com/bob-ros2/bob_audio.git
cd ..
colcon build --packages-select bob_audio
source install/setup.bash
```

## ROS API

### 1. Mixer Node (`mixer`)

The mixer node takes multiple audio inputs and aggregates them into one master output using additive mixing with clipping protection.

#### Parameters & Environment Variables

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `input_count` | int | `4` | Number of input topics (`in0`...`inN-1`). |
| `enable_fifo_output` | bool | `false` | Enable writing to a FIFO pipe (Env: `MIXER_ENABLE_FIFO_OUTPUT`) |
| `output_fifo` | string | `/tmp/audio_master_pipe`| Path to the master output FIFO (Env: `MIXER_OUTPUT_FIFO`) |
| `enable_stdout_output`| bool | `false` | Write raw audio to stdout for piping (Env: `MIXER_ENABLE_STDOUT_OUTPUT`) |
| `enable_topic_output` | bool | `true` | Publish mixed audio to ROS topic `out` (Env: `MIXER_ENABLE_TOPIC_OUTPUT`) |
| `enable_fifo_input` | bool | `false` | Read from an additional input FIFO pipe (Env: `MIXER_ENABLE_FIFO_INPUT`) |
| `input_fifo` | string | `/tmp/audio_pipe` | Path to the input FIFO (Env: `MIXER_INPUT_FIFO`) |
| `sample_rate` | int | `44100` | Audio sample rate in Hz (Env: `MIXER_SAMPLE_RATE`) |
| `channels` | int | `2` | Number of audio channels (Env: `MIXER_CHANNELS`) |
| `chunk_ms` | int | `20` | Processing interval per chunk in ms (Env: `MIXER_CHUNK_MS`) |

#### Topics

- **Subscribers**: 
  - `in0` ... `inN-1` (`std_msgs/msg/Int16MultiArray`): Audio input streams.
  - `control` (`std_msgs/msg/String`): JSON-based volume and channel control.
- **Publishers**: 
  - `out` (`std_msgs/msg/Int16MultiArray`): Mixed master output.

#### Dynamic Control (JSON)

You can adjust volumes and input channel counts on the fly by sending a JSON string to the `control` topic.

**Format (Nested Object):**
```json
{
  "in0": {"gain": 0.5, "channels": 1},
  "in1": {"gain": 1.2},
  "master": {"gain": 0.7}
}
```

**Format (Flat Keys):**
```json
{
  "in0_gain": 0.2,
  "in1_channels": 2,
  "fifo": 0.8
}
```

**Example:**
```bash
ros2 topic pub --once /control std_msgs/msg/String "{data: '{\"in0\": {\"gain\": 0.2, \"channels\": 1}, \"master\": 0.5}'}"
```

### 2. Convert Node (`convert`)

Bidirectional conversion between raw audio streams (FIFO/Pipe/stdin) and ROS messages.

#### Parameters & Environment Variables

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `mode` | string | `fifo_to_ros` | Mode: `fifo_to_ros`, `stdin_to_ros`, or `ros_to_fifo` (Env: `CONVERT_MODE`) |
| `fifo_path` | string | `/tmp/audio_pipe`| Path to the FIFO pipe (Env: `CONVERT_FIFO_PATH`) |
| `sample_rate` | int | `44100` | Audio sample rate in Hz (Env: `CONVERT_SAMPLE_RATE`) |
| `channels` | int | `2` | Number of audio channels (Env: `CONVERT_CHANNELS`) |
| `chunk_ms` | int | `20` | Chunk size in ms per ROS message (Env: `CONVERT_CHUNK_MS`) |

#### Topics

- **Subscribers**: `in` (`std_msgs/msg/Int16MultiArray`): Active if mode is `ros_to_fifo`.
- **Publishers**: `out` (`std_msgs/msg/Int16MultiArray`): Active if mode is `*_to_ros`.

## Usage Examples

### Endlessly looping Background Music

```bash
ffmpeg -re -stream_loop -1 -i music.mp3 -f s16le -ar 44100 -ac 2 pipe:1 | \
  ros2 run bob_audio convert --ros-args -p mode:=stdin_to_ros -r out:=/bob/audio_music
```

### Listening to ROS audio (e.g. from Mixer) via ffplay

```bash
# Convert ROS topic to FIFO
ros2 run bob_audio convert --ros-args \
  -r in:=/out \
  -p mode:=ros_to_fifo \
  -p fifo_path:=/tmp/audio_ffplay

# Play the FIFO
ffplay -f s16le -ar 44100 -ac 2 -i /tmp/audio_ffplay
```

### Piping Mixer directly into FFmpeg

```bash
export MIXER_ENABLE_STDOUT_OUTPUT=true
ros2 run bob_audio mixer | ffmpeg -f s16le -ar 44100 -ac 2 -i pipe:0 ...
```

### Full Audio Pipeline Test (FFmpeg -> ROS -> ffplay)

A complete roundtrip to verify the entire system:

```bash
# 1. Create the FIFO for ffplay
mkfifo /tmp/audio_ffplay

# 2. Start the player (waits for data)
ffplay -f s16le -ar 44100 -ac 2 -i /tmp/audio_ffplay

# 3. Start the ROS-to-FIFO converter (Terminal 2)
ros2 run bob_audio convert --ros-args \
  -p mode:=ros_to_fifo \
  -p fifo_path:=/tmp/audio_ffplay \
  -r in:=/audio_stream

# 4. Start the FFmpeg-to-ROS producer (Terminal 3)
ffmpeg -re -i music.mp3 -f s16le -ar 44100 -ac 2 pipe:1 | \
ros2 run bob_audio convert --ros-args \
  -p mode:=stdin_to_ros \
  -r out:=/audio_stream
```

## License

Apache-2.0
