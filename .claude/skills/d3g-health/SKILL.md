---
name: d3g-health
description: Perform a read-only health check of the SC2026 D3-G environment, Git state, ROS2 setup, storage, nodes, topics, and camera pipeline.
disable-model-invocation: true
---

# D3-G Read-Only Health Check

Run only after confirming the current host is the intended D3-G board.

## Safety
- This skill is read-only.
- Never publish a ROS2 topic.
- Never start or stop a node, launch file, motor, steering, or throttle.
- Never install packages, change networking, modify files, build code, commit, or pull.
- If the host cannot be identified as D3-G, stop and report that the check was not run.

## Checks
1. Identify hostname, OS, architecture, current user, and current directory.
2. Confirm `/opt/ros/humble` exists and report `ROS_DISTRO`.
3. Locate the `D-Racer-Kit` Git workspace.
4. Report:
   - current branch
   - working-tree state
   - remote URL
   - latest commit
5. Check storage with `df -h`.
6. Check relevant processes without changing them.
7. Source ROS2 and the workspace setup only if the setup files already exist.
8. Report:
   - `ros2 node list`
   - `ros2 topic list`
   - topic type and publisher/subscriber counts for:
     - `/camera/image/compressed`
     - `/control`
     - `/joystick`
     - `/battery_status`
9. Measure camera topic frequency for a bounded duration only when the topic exists.
10. Do not treat an absent node or topic as a failure unless it was expected to be running.

## Output
Provide a compact table with:
- Component
- Observed state
- Expected state
- Result: PASS, WARN, FAIL, or NOT RUN
- Recommended next action

Clearly separate environment readiness from runtime-node readiness.