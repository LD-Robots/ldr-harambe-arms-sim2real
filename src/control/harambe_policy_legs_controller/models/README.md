# Bundled policy — harambe_policy_legs_controller

`policy.onnx` (+ `policy.onnx.data`, external weights — keep both together) is the
deploy policy this controller loads by default.

- **Model:** harambe_mjx_v7 (legs-only MJX walker)
- **Contract:** 52 obs / 12 act, `action_scale = 0.5`, gait_freq 1.5 Hz
- **Obs normalizer is baked into the ONNX** — feed raw obs.
- **Source:** `pnd_adam_test/models/harambe_mjx_v7/` (run 2026-06-12_11-40-48,
  model_16650 — velocity restored + symmetry-curriculum resume; best velocity
  tracking + straighter walking).

The launch file defaults `onnx_path` to this file
(`share/harambe_policy_legs_controller/models/policy.onnx`). Override with
`onnx_path:=<path>` to load a different policy.

To update: copy the newer `policy.onnx` + `policy.onnx.data` here and rebuild
(`colcon build --packages-select harambe_policy_legs_controller`).
