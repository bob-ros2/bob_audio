^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bob_audio
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.1 (2026-03-07)
------------------
* Add bidirectional conversion (FIFO/stdin <-> ROS) with exclusive mode selection
* Multi-threaded Mixer with dynamic input count and Mono-to-Stereo upmixing
* Redesigned 'control' topic for live gain and channel configuration
* Simplified FIFO output (auto-enable) and 'heartbeat' silence generator to keep streams alive
* Contributors: Bob Ros

0.1.0 (2026-03-05)
------------------
* Add real-time level control via JSON over 'levels' topic
* Initial release of bob_audio
* Contributors: Bob Ros
