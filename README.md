# Drone Navigation ML

A Unity ML-Agents project training a quadrotor to navigate procedurally generated 3D obstacle corridors using PPO.

## Overview

The agent controls 4 continuous movement axes (lateral, vertical, longitudinal forces and yaw torque) to reach a randomized target zone while avoiding obstacles spawned each episode.

## Observation and Action Space

- **Observations (12 floats):** Relative position to target, normalized target direction, linear velocity, angular velocity.
- **Actions (4 continuous):** X Force, Y Force, Z Force, Yaw Torque.

## Setup and Training

### Requirements

- Unity 2022.3 LTS+ with ML-Agents `v2.0.0+`
- Python 3.9/3.10
- `mlagents` package

### Install

```bash
python -m venv .venv
source .venv/bin/activate  # Windows: .venv\Scripts\activate
pip install mlagents
```

### Run Training

1. Run the trainer from your project root:
   ```bash
   mlagents-learn Config/drone_config.yaml --run-id=Drone_Run_1
   ```
2. Press Play in the Unity Editor.

To view metrics:
```bash
tensorboard --logdir results
```

## Manual Testing

Set **Behavior Type** to `Heuristic Only` on the `DroneAgent` prefab to fly manually in editor:

- **W/S**: Pitch Forward / Back
- **A/D**: Roll Left / Right
- **Space/Left Shift**: Elevate Up / Down
- **Q/E**: Yaw Left / Right
