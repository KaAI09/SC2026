---
paths:
  - "D-Racer-Kit/src/**/*"
  - "D-Racer-Kit/docs/**/*"
---

# ROS2 And D3-G Rules

## Platform
- Target ROS2 Humble on the D3-G Ubuntu 22.04 image.
- Do not assume ROS2 packages or D3-G hardware libraries are available on macOS.
- Use macOS for source inspection and hardware-independent checks only.
- Build and run ROS2 packages from the `D-Racer-Kit` workspace on D3-G.

## Source Changes
- Inspect `package.xml`, `setup.py`, `CMakeLists.txt`, launch files, and YAML configuration before changing package behavior.
- Preserve existing node, topic, message, parameter, and launch interfaces unless an interface change is explicitly required.
- When changing an interface, update every publisher, subscriber, message dependency, launch file, configuration file, and relevant document.
- Keep hardware-dependent code separate from perception and decision logic where the current structure permits.
- Do not hard-code IP addresses, camera device paths, credentials, or machine-specific directories in source code.

## Validation Order
1. Run syntax and static checks.
2. Build only the affected package and its dependencies when possible.
3. Source the ROS2 and workspace setup files.
4. Check node startup and logs.
5. Verify node, topic, message type, publisher count, and subscriber count.
6. Test camera or control output with the vehicle stationary.
7. Perform low-speed track testing only after stationary validation succeeds.

## Vehicle Control Safety
- Never publish to `/control` or start an actuation node without explicit user confirmation.
- State the proposed steering, throttle, rate, and duration before sending a command.
- Begin with zero throttle and conservative steering.
- Use bounded-duration commands and return throttle and steering to a safe neutral state.
- Confirm an emergency-stop method before autonomous testing.
- Stop testing if camera input, control timing, communication, or confidence becomes stale or invalid.
- Do not remove clamps, timeouts, neutral fallback, or emergency-stop behavior merely to improve speed.

## Evidence
- Preserve the exact command, relevant log output, test condition, and observed result for D3-G tests.
- Distinguish simulation, stationary bench testing, wheels-off-ground testing, and track testing.
- Do not describe a feature as verified until it has passed the appropriate D3-G test.