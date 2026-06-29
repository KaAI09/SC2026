---
paths:
  - "D-Racer-Kit/src/opencv/**/*"
  - "training/**/*"
  - "models/**/*"
  - "scripts/**/*"
  - "**/*train*.py"
  - "**/*infer*.py"
---

# Vision And Model Rules

## Approach
- OpenCV rule-based vision and learning-based vision are both permitted.
- YOLO26n is an official recommendation, not an automatic requirement.
- Prefer the simplest method that remains reliable under track lighting, camera vibration, motion blur, and D3-G latency constraints.
- Keep perception output separate from mission-state and vehicle-control logic.

## Dataset
- Record the source, capture condition, class definition, labeling format, and dataset version.
- Prevent leakage by splitting related video frames by source video or sequence rather than randomly by individual frame.
- Keep training, validation, and test sets distinct.
- Inspect class balance and difficult negative samples.
- Never include passwords, personal data, or unrelated recordings.

## Reproducibility
- Fix and record random seeds where supported.
- Record software versions, model configuration, input resolution, preprocessing, augmentation, batch size, and training duration.
- Save the dataset manifest and experiment configuration separately from large artifacts.
- Do not commit datasets or model weights unless the team has explicitly selected Git LFS or external artifact storage.

## Evaluation
- Report task-appropriate accuracy, precision, recall, confusion cases, and confidence thresholds.
- Measure model size, preprocessing time, inference latency, end-to-end latency, memory use, and effective FPS on D3-G.
- Evaluate false positives and false negatives according to their driving consequences.
- Test with track-like lighting, distance, viewing angle, blur, and partial occlusion.

## Runtime Safety
- Do not map a single raw detection directly to throttle or steering.
- Use confidence thresholds, temporal confirmation, hysteresis, timeout handling, and a safe unknown state where appropriate.
- Define behavior for missing frames, stale detections, low confidence, conflicting signs, and inference failure.
- Keep a conservative fallback that stops or maintains a previously verified safe state.
- Validate exported model compatibility on D3-G before integrating it into a ROS2 node.

## Compute Roles
- Use the Linux Server PC for GPU training and large evaluations.
- Use D3-G for final preprocessing, inference, latency, ROS2 integration, and vehicle tests.
- Do not claim deployment success from Server PC metrics alone.