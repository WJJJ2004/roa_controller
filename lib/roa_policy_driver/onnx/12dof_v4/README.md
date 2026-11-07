# Model 45900 deployment bundle

Source: `2026-11-06_19-45-01/model_45900.pt`. `policy.onnx` maps 45 float32 observations to 12 float32 actions. `data/golden_obs.bin` is one recorded 45-element observation; `data/golden_action.bin` is its 12-element ONNX output. Both binary files are little-endian float32. See `manifest.json` for hashes and export verification.
