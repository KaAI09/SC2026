# SC2026 Project Guide

## Project
- SEA:ME Hackathon 2026 autonomous scale-car project.
- Target hardware: TOPST D3-G based D-Racer Kit.
- Runtime: Ubuntu 22.04, ROS2 Humble, Python and OpenCV.
- Core tasks include lane following, traffic-light recognition, direction-sign recognition, dynamic-obstacle handling, and safe vehicle control.
- Treat official documents under `Notice/` and `D-Racer-Kit/docs/` as the primary technical references.

## System Roles
- Local PCs: edit code, use Claude Code, manage Git branches, and review logs.
- D3-G: pull tested branches, build ROS2 packages, run nodes, and perform vehicle tests.
- Never edit source code or push commits directly from D3-G.
- Linux Server PC: GPU training and model evaluation only when needed.
- Control PC: D3-G network setup, serial access, live logs, and video monitoring.

## Repository
- `D-Racer-Kit/src/`: ROS2 packages and scripts.
- `D-Racer-Kit/docs/`: official hardware and software guides.
- `Env/`: team architecture and workflow documentation.
- `Notice/`: official hackathon mission and rule materials.

## Git Workflow
1. Check `git status --short --branch` before editing.
2. Never develop directly on `main`.
3. Use a task branch such as `feat/<name>`, `fix/<name>`, or `setup/<name>`.
4. Keep changes narrowly scoped and preserve unrelated team changes.
5. Commit and push only after relevant validation.
6. Merge into `main` only after D3-G testing confirms an improvement.
7. Never force-push, reset shared history, or discard another member's work.

## Development Workflow
1. Read the relevant documentation and inspect existing code.
2. State assumptions and propose a short implementation plan.
3. Make the minimum required change.
4. Run local static checks that do not require ROS2 or D3-G hardware.
5. Push the task branch only when explicitly requested.
6. Pull and build that branch on D3-G.
7. Validate ROS2 nodes, topics, logs, camera output, and control output.
8. Record the test result before considering a merge.

## Environment Boundaries
- Do not assume ROS2 Humble or D3-G hardware is available on macOS.
- macOS validation is limited to editing, Git checks, documentation, syntax checks, and hardware-independent tests.
- Run ROS2 builds and hardware-dependent tests on D3-G.
- Run GPU-dependent training on the Linux Server PC.
- Do not silently install system packages or modify board network configuration.

## ROS2 Context
- Camera topic: `/camera/image/compressed`
- Control topic: `/control`
- Joystick topic: `/joystick`
- Battery topic: `/battery_status`
- Main packages include `camera`, `control`, `joystick`, `monitor`, `battery`, `opencv`, `topst_utils`, and custom message packages.
- Confirm actual node, topic, message, and launch names from source before changing interfaces.

## Vehicle Safety
- Never execute actuation, throttle, steering, or autonomous-driving commands without explicit confirmation.
- Before actuation tests, confirm the test area is clear and the vehicle can be stopped immediately.
- Begin tests with zero throttle and conservative steering limits.
- Prefer wheels-off-ground verification before track testing.
- Preserve an emergency-stop path and return to a safe stopped state after failures.
- Do not weaken safety limits merely to improve lap time.

## Vision And Models
- Separate data preparation, training, model artifacts, inference, and vehicle-control logic.
- Record dataset version, split, model version, metrics, and inference latency.
- Do not commit datasets, credentials, or large model files without an agreed storage or Git LFS policy.
- Validate inference compatibility and latency on D3-G before track use.

## Claude Working Style
- Respond in Korean unless English is requested.
- Distinguish verified facts from assumptions.
- Prefer repository conventions over new abstractions.
- Explain commands that can affect Git history, networking, packages, or hardware.
- Do not commit, push, merge, or open a pull request unless explicitly requested.
- After changes, report modified files, validation performed, and remaining risks.