# bob_audio

A high-performance ROS 2 package for audio mixing and conversion, optimized for Docker and headless environments. It allows combining multiple audio sources (TTS, Music, SFX) into a single master stream without the need for a full audio server like PulseAudio or JACK.

## Features

- **High Performance**: Written in C++ with minimal overhead.
- **Flexible Mixing**: Mix up to 8 ROS topics (via `Int16MultiArray`) and optional FIFO inputs.
- **Docker Friendly**: No hardware audio device requirements.
- **Multiple Output Sinks**: Stream to ROS topics, FIFO pipes, or directly to stdout (for FFmpeg).
- **Audio Conversion**: Easily convert between raw pipes/stdin and ROS messages.

## Installation & Build

### Prerequisites

- ROS 2 Humble (or newer)
- `std_msgs`

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

| Parameter | Type | Default | Env Var | Description |
| :--- | :--- | :--- | :--- | :--- |
| `enable_fifo_output` | bool | `false` | `MIXER_ENABLE_FIFO_OUTPUT` | Write to FIFO |
| `output_fifo` | string | `/tmp/audio_master_pipe`| `MIXER_OUTPUT_FIFO` | Path to master FIFO |
| `enable_stdout_output`| bool | `false` | `MIXER_ENABLE_STDOUT_OUTPUT`| Write to stdout |
| `enable_topic_output` | bool | `true` | `MIXER_ENABLE_TOPIC_OUTPUT` | Publish to `out` topic |
| `enable_fifo_input` | bool | `false` | `MIXER_ENABLE_FIFO_INPUT` | Read from an input FIFO |
| `input_fifo` | string | `/tmp/audio_pipe` | `MIXER_INPUT_FIFO` | Path to input FIFO |
| `sample_rate` | int | `44100` | `MIXER_SAMPLE_RATE` | Sample rate in Hz |
| `channels` | int | `2` | `MIXER_CHANNELS` | Number of channels |
| `chunk_ms` | int | `20` | `MIXER_CHUNK_MS` | Delay per chunk |

#### Topics

- **Subscribers**: `in0` ... `in7` (`std_msgs/msg/Int16MultiArray`). Use remapping to connect your sources.
- **Publishers**: `out` (`std_msgs/msg/Int16MultiArray`).

### 2. Convert Node (`convert`)

Converts external raw audio streams into ROS topics.

#### Parameters & Environment Variables

| Parameter | Type | Default | Env Var | Description |
| :--- | :--- | :--- | :--- | :--- |
| `enable_fifo` | bool | `false` | `CONVERT_ENABLE_FIFO` | Read from FIFO |
| `input_fifo` | string | `/tmp/audio_pipe` | `CONVERT_INPUT_FIFO` | Path to input FIFO |
| `enable_stdin` | bool | `false` | `CONVERT_ENABLE_STDIN` | Read from stdin |
| `sample_rate` | int | `44100` | `CONVERT_SAMPLE_RATE` | Sample rate in Hz |
| `channels` | int | `2` | `CONVERT_CHANNELS` | Number of channels |
| `chunk_ms` | int | `20` | `CONVERT_CHUNK_MS` | Chunk size |

#### Topics

- **Publishers**: `out` (`std_msgs/msg/Int16MultiArray`).

## Usage Examples

### Mixing TTS and Music for a Twitch Stream

```bash
# Start the mixer, remapping inputs to specific sources
ros2 run bob_audio mixer --ros-args \
  -r in0:=/bob/audio_speech \
  -r in1:=/bob/audio_music \
  -p enable_fifo_output:=true \
  -p output_fifo:=/tmp/audio_master_pipe
```

### Piping directly into FFmpeg

```bash
export MIXER_ENABLE_STDOUT_OUTPUT=true
ros2 run bob_audio mixer | ffmpeg -f s16le -ar 44100 -ac 2 -i pipe:0 ...
```

### Converting a radio stream to ROS

```bash
ffmpeg -i http://stream.radio.de/live.mp3 -f s16le -ar 44100 -ac 2 pipe:1 | \
  ros2 run bob_audio convert --ros-args -p enable_stdin:=true -r out:=/bob/audio_music
```

## License

Apache-2.0
