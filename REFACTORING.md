# Refactoring Notes

This document records the implementation review and performance-related refactoring work. It is intentionally separate from the user-facing README files.

## Summary

The refactor focused on removing avoidable dynamic allocation from real-time paths, tightening CAN ring-buffer behavior, fixing command sequence handling, and making the ESP32-oriented math and command state lighter.

## Changes

| Area | Before | After | Effect |
| --- | --- | --- | --- |
| CAN frame reconstruction | One `malloc/free` pair for each verified frame | Fixed stack buffer | 0 heap allocations during CAN frame parsing |
| micro-ROS `chassis_info2` | One `malloc/free` pair per publish | Static storage initialized once | 0 heap allocations per publish |
| micro-ROS `/tf` | Three `malloc/free` pairs per publish | Static storage initialized once | 0 heap allocations per publish |
| micro-ROS wheel + IMU | Two `malloc/free` pairs per update | Static storage initialized once | 0 heap allocations per update |
| micro-ROS joint state | Five `malloc/free` pairs per update | Static storage initialized once | 0 heap allocations per update |
| Velocity command state | `double` fields | `float` fields | Cuts linear/angular command-state storage roughly in half |
| Trigonometry helpers | `cos/sin/atan2` | `cosf/sinf/atan2f` | Avoids promotion to double math on ESP32-oriented code |
| Command sequence counter | Incremented inside byte-copy loops in some commands | Incremented once per generated command | Reduces counter writes from command-length proportional to constant time |
| CAN RX/TX rings | Overflow and unknown-ID behavior was loose | Bounds checks and unknown-ID discard | Avoids buffer corruption and uninitialized CAN ID handling |
| CAN timeout path | Timeout returned before RX processing | Timeout disables velocity but still processes RX | Allows recovery when valid CAN traffic returns |

## Performance Impact

The most important measurable change is the removal of heap allocation from high-frequency paths.

In the micro-ROS sketch, a busy telemetry cycle previously allocated and freed memory repeatedly while publishing:

- `/tf`: 3 allocation/free pairs
- wheel + IMU update: 2 allocation/free pairs
- joint state update: 5 allocation/free pairs
- chassis info publish: 1 allocation/free pair at its lower publish rate

Those runtime allocations are now replaced with static buffers assigned once during micro-ROS setup. For the active publish path, the corresponding loop-time `malloc/free` count is now 0.

CAN frame parsing also no longer allocates per reconstructed frame. At 1 Mbps CAN traffic rates, that matters not only for raw CPU time but also for long-running stability because heap fragmentation is removed from the parser path.

Command generation now increments the sequence counter once per generated command. The move command previously performed roughly 25 counter increments while building a single command frame; it now performs 1. The gimbal command and generic command path follow the same constant-time counter update behavior.

Exact wall-clock speedup depends on the ESP32 board, enabled libraries, CAN traffic volume, Wi-Fi quality, and micro-ROS agent behavior. The reliable quantitative claim from this refactor is the removal of loop-time heap allocation in the paths above, reduced command counter writes, and smaller float-based command state.

## Validation

The shared C modules were checked with:

```bash
cc -std=c11 -Wall -Wextra -fsyntax-only \
  Common/robomaster_s1.c Common/robomaster_s1_crc.c \
  RoboStackS1/robomaster_s1.c RoboStackS1/robomaster_s1_crc.c \
  RoboStackS1_avatar/robomaster_s1.c RoboStackS1_avatar/robomaster_s1_crc.c \
  RoboStackS1_micro-ros/robomaster_s1.c RoboStackS1_micro-ros/robomaster_s1_crc.c
```

The command completed successfully. Full Arduino sketch builds were not run because this environment does not currently provide `arduino-cli` or PlatformIO, and the required Arduino libraries are not available in the active include path.
