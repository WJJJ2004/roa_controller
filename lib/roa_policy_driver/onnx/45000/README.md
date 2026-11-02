# 45000 deployment files

Source: `2026-10-31_00-06-26/model_45000.pt`, torso CoM X DR +0.09–+0.16m.

- `policy.onnx`: 45 observations → 12 actions.
- `data/golden_obs.bin`: 45 little-endian float32 (180 bytes).
- `data/golden_action.bin`: 12 little-endian float32 (48 bytes).

Torch/ONNX comparison over146observations and copied-model golden inference PASS. See manifest.json. Controller default model unchanged; hardware not started.
