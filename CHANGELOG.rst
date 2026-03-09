^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bob_audio
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.1 (2026-03-07)
------------------
* Add bidirectional conversion (FIFO/stdin <-> ROS) with exclusive mode selection
* Multi-threaded Mixer with dynamic input count and Mono-to-Stereo upmixing
* Redesigned 'control' topic for live gain and channel configuration
* Simplified FIFO output (auto-enable) and 'heartbeat' silence generator to keep streams alive
* Full dynamic support for 'ros2 param set' and JSON control to switch channel counts and gains on the fly
* Automatic buffer flushing during channel transitions to prevent pitch and speed distortions
* Significant overhaul of audio stability: high-capacity jitter buffers (300s) for TTS burst handling and non-blocking I/O to eliminate dropouts and node hangs
* Contributors: Bob Ros

0.1.0 (2026-03-05)
------------------
* Add real-time level control via JSON over 'levels' topic
* Initial release of bob_audio
* Contributors: Bob Ros
