# Bundled policy — harambe_policy_legs_controller

`policy.onnx` (+ `policy.onnx.data`, external weights — keep both together) is the
deploy policy this controller loads by default.

* **Model:** harambe_mjx_v10 (legs-only MJX walker) — sha1 `e4132c10c869`
* **Contract:** 52 obs / 12 act, `action_scale = 0.5`, gait_freq 1.5 Hz
* **Obs normalizer is baked into the ONNX** — feed raw obs.
* **Source:** `pnd_adam_test/models/harambe_mjx_v10/` (run 2026-06-18_14-45-44, model_17000).

* **sim2real-robust:** trained with random obs LATENCY (0-100 ms on base_lin_vel + gyro + gravity) + widened noise, so it tolerates the real VIO velocity lag that made v7 thrash on hardware. Deploy with `action_scale=0.5`, ankle kp/kd match-pitch (~0.95/0.07 motor), and IMU calibration.

* **Velocity frame / IMU notes:**
  * During training, `base_lin_vel` was computed using the IMU mounted in the **torso**, not the pelvis IMU.
  * For VIO deployment, the velocity estimate should therefore be derived using the **torso IMU** frame.
  * Gyro and gravity observations (`gyro`, `projected_gravity`) should continue to come from the **pelvis IMU**, matching the training setup.
  * The torso IMU has a different mounting orientation than the pelvis IMU. Do not assume the same frame convention in the VIO implementation.
  * Torso IMU mounting convention:

    * `+X` = up
    * `+Y` = right
    * `+Z` = backward

* **Camera IMU experiment (hypothesis):**
  * During training, `base_lin_vel` was derived from the IMU mounted in the torso.
  * The camera IMU is physically closer to that velocity reference point than the pelvis IMU, since both are located in the upper body region.
  * It may therefore be worth testing a VIO configuration that uses the camera IMU for velocity estimation, even though the sensor is mounted at head level.
  * This is currently only a hypothesis based on sensor placement and distance to the training-time velocity reference point; it has not been experimentally validated.
  * If testing this configuration, carefully verify frame alignment and extrinsics, as the camera IMU, torso IMU, and pelvis IMU do not share identical mounting orientations.



The launch file defaults `onnx_path` to this file
(`share/harambe_policy_legs_controller/models/policy.onnx`). Override with
`onnx_path:=<path>` to load a different policy.

To update: copy the newer `policy.onnx` + `policy.onnx.data` here and rebuild
(`colcon build --packages-select harambe_policy_legs_controller`).
