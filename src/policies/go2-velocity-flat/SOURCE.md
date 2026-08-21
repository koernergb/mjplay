---
tags:
  - reinforcement-learning
  - robotics
  - locomotion
  - unitree
  - go2
  - mujoco
  - ppo
library_name: rsl-rl
license: bsd-3-clause
---

# Unitree Go2 — Velocity Flat (PPO)

RL locomotion policy for the [Unitree Go2](https://www.unitree.com/go2/) quadruped robot, trained on flat terrain using PPO.

## Demo

[![Unitree Go2 RL Locomotion in MuJoCo](https://img.youtube.com/vi/smxh8Uu2Zpo/maxresdefault.jpg)](https://www.youtube.com/watch?v=smxh8Uu2Zpo)

## Training

- **Framework**: [unitree_rl_mjlab](https://github.com/unitreerobotics/unitree_rl_mjlab) (MuJoCo Warp)
- **Task**: `Mjlab-Velocity-Flat-Unitree-Go2`
- **Algorithm**: PPO (RSL-RL)
- **Hardware**: 10× NVIDIA RTX A4000, 56 CPU cores
- **Environments**: 8192 parallel
- **Training time**: ~18 minutes (506 iterations)

## Results

| Metric | Value |
|---|---|
| Mean reward | **52.9** |
| Mean episode length | **1000** (max, no falls) |
| Steps/sec | 628K-738K |

## Files

| File | Description |
|---|---|
| `policy.onnx` + `policy.onnx.data` | ONNX model for deployment (go2_ctrl) |
| `model_500.pt` | Final PyTorch checkpoint (best for fine-tuning) |
| `model_0.pt` ... `model_400.pt` | Intermediate checkpoints every 100 steps |
| `params/deploy.yaml` | Deploy configuration (obs order, action scale, joint mapping) |
| `params/env.yaml` | Environment configuration |
| `params/agent.yaml` | Agent/PPO configuration |
| `events.out.tfevents.*` | TensorBoard training logs |

## Usage

### Deploy in MuJoCo simulator

```bash
# Copy ONNX model + deploy config
cp policy.onnx policy.onnx.data \
  unitree_rl_mjlab/deploy/robots/go2/config/policy/velocity/v0/exported/
cp params/deploy.yaml \
  unitree_rl_mjlab/deploy/robots/go2/config/policy/velocity/v0/params/

# Build controller
cd unitree_rl_mjlab/deploy/robots/go2
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc)

# Run simulator + controller
cd unitree_mujoco/simulate/build && ./unitree_mujoco
cd unitree_rl_mjlab/deploy/robots/go2/build && ./go2_ctrl --network=lo
```

> **Important:** This model was trained **without `gait_phase`** and with **action scale 0.5**. The default `deploy.yaml` in unitree_rl_mjlab may differ — use `params/deploy.yaml` from this repo.

### Fine-tune on rough terrain

```bash
# Place model_500.pt in logs/rsl_rl/go2_velocity/<run_name>/
python scripts/train.py Mjlab-Velocity-Rough-Unitree-Go2 \
  --agent.resume=True \
  --agent.load-run="<run_name>" \
  --agent.load-checkpoint="model_500.pt" \
  --agent.algorithm.learning-rate=1e-4
```

## Known Issues

The upstream `unitree_rl_mjlab` has bugs that crash multi-GPU training on rough terrain — see [Issue #9](https://github.com/unitreerobotics/unitree_rl_mjlab/issues/9) and [PR #8](https://github.com/unitreerobotics/unitree_rl_mjlab/pull/8).
