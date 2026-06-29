---
name: race-readiness
description: Evaluate SC2026 vehicle, ROS2, perception, mission logic, safety, and logging readiness before a stationary or track test.
disable-model-invocation: true
---

# SC2026 Race Readiness Review

Assess readiness without publishing control commands or starting vehicle motion.

## Required Context
Ask for any missing information instead of assuming:
- Test type: stationary, wheels-off-ground, or track
- Git branch and commit
- Mission or feature being tested
- Expected nodes and topics
- Emergency-stop method
- Current battery and storage state

## Checklist

### Software
- Intended branch and commit are checked out.
- Working tree is understood.
- Affected ROS2 packages built successfully.
- Correct ROS2 and workspace setup files are sourced.
- Required nodes are running without repeated errors.
- Topic types and publisher/subscriber counts match expectations.

### Perception
- Camera stream exists and has stable bounded latency and frame rate.
- Required signal, sign, lane, or obstacle output is visible.
- Confidence thresholds and temporal confirmation are configured.
- Missing, stale, low-confidence, and conflicting inputs have safe behavior.

### Control
- Initial throttle is zero.
- Steering range, trim, clamps, and timeout are known.
- Loss of input returns to a safe neutral or stopped state.
- Autonomous output cannot override emergency stop.
- No unverified high-speed or full-range command is enabled.

### Hardware And Track
- Battery and power connections are secure.
- Wheels, steering linkage, camera, and cables are secure.
- Test area is clear and the assigned track direction is confirmed.
- A person is ready to stop or lift the vehicle.
- The next test begins at the lowest practical speed and duration.

### Observability
- Logs, camera/debug output, and control values can be monitored.
- Storage has enough free space.
- Test objective and success/failure conditions are recorded.
- A rollback commit or known-good branch is available.

## Decision
Return exactly one overall decision:
- `GO`: all required checks are verified.
- `CONDITIONAL GO`: only explicitly listed low-risk conditions remain.
- `NO-GO`: any safety, control, perception, build, or observability requirement is unverified.

List blocking items before recommendations. Never convert an unknown item into a pass.