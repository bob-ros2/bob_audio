# ROS Package [bob_audio](https://github.com/bob-ros2/bob_audio)

[![ROS 2 CI](https://github.com/bob-ros2/bob_audio/actions/workflows/ros_ci.yml/badge.svg)](https://github.com/bob-ros2/bob_audio/actions/workflows/ros_ci.yml)

A high-performance ROS 2 package for audio mixing and conversion, optimized for Docker and headless environments. It allows combining multiple audio sources (TTS, Music, SFX) into a single master stream without the need for a full audio server like PulseAudio or JACK.

## Features

- **High Performance**: Written in C++ with minimal overhead.
- **Efficient Mixing**: Mix up to 8 ROS topics (via `Int16MultiArray`) with additive mixing and clipping protection.
- **Docker Friendly**: No hardware audio device requirements.
- **FIFO Output**: Stream mixed audio directly to a named pipe (FIFO).
- **Audio Conversion**: Bidirectional conversion between raw streams and ROS messages.

## Installation & Build

### Prerequisites

- ROS 2 Humble (or newer)
- `std_msgs`, `nlohmann-json`

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

The mixer node takes multiple audio inputs and aggregates them into one master output FIFO.

#### Parameters & Environment Variables

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `enable_fifo_output` | bool | `true` | Enable writing to a FIFO pipe (Env: `MIXER_ENABLE_FIFO_OUTPUT`) |
| `output_fifo` | string | `/tmp/audio_master_pipe`| Path to the master output FIFO (Env: `MIXER_OUTPUT_FIFO`) |
| `sample_rate` | int | `44100` | Audio sample rate in Hz (Env: `MIXER_SAMPLE_RATE`) |
| `channels` | int | `1` | Number of audio channels (Env: `MIXER_CHANNELS`) |
| `chunk_ms` | int | `20` | Processing interval per chunk in ms (Env: `MIXER_CHUNK_MS`) |

#### Topics

- **Subscribers**: 
  - `in0` ... `in7` (`std_msgs/msg/Int16MultiArray`): Audio input streams.

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
