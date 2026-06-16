# Bundled policy — harambe_policy_legs_controller

`policy.onnx` (+ `policy.onnx.data`, external weights — keep both together) is the
deploy policy this controller loads by default.

- **Model:** harambe_mjx_v8 (legs-only MJX walker) — sha1 `083aae1a7b6b`
- **Contract:** 52 obs / 12 act, `action_scale = 0.5`, gait_freq 1.5 Hz
- **Obs normalizer is baked into the ONNX** — feed raw obs.
- **Source:** `pnd_adam_test/models/harambe_mjx_v8/` (run 2026-06-15_20-04-10,
  model_31150). **sim2real-robust**: trained with random obs LATENCY (0-100 ms on
  base_lin_vel + gyro + gravity) + widened noise, so it tolerates the real VIO
  velocity lag that made v7 thrash on hardware. Deploy with action_scale=0.5,
  ankle kp/kd match-pitch (~0.95/0.07 motor), and IMU calibration.

The launch file defaults `onnx_path` to this file
(`share/harambe_policy_legs_controller/models/policy.onnx`). Override with
`onnx_path:=<path>` to load a different policy.

To update: copy the newer `policy.onnx` + `policy.onnx.data` here and rebuild
(`colcon build --packages-select harambe_policy_legs_controller`).
