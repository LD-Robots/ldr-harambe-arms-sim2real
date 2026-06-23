# Bundled policy — harambe_policy_legs_controller

`policy.onnx` (+ `policy.onnx.data`, external weights — keep both together) is the
deploy policy this controller loads by default.

* **Model:** harambe_mjx_v11 (legs-only MJX walker) — sha1 `5d4f55161e5f`
* **Contract:** 52 obs / 12 act, `action_scale = 0.5`, gait_freq 1.5 Hz
* **Obs normalizer is baked into the ONNX** — feed raw obs.
* **Source:** `pnd_adam_test/models/harambe_mjx_v11/` (run 2026-06-22_19-42-35, model_14300).

## ⚠️ NEW default pose — must match `default_positions` in the controller yaml
v11 was trained with a **new, straighter default pose** (CONTRACT change vs v8-v10,
since obs[12:24] = q − default and target = default + 0.5·action):

* legs (this controller): hip_pitch **−0.05**, knee **0.10**, ankle_pitch **−0.05**
  (was −0.15 / 0.30 / −0.15) — already set in
  `config/harambe_policy_legs_controller.yaml` `default_positions`
  (`[-0.05, 0, 0, 0.10, -0.05, 0,  -0.05, 0, 0, 0.10, -0.05, 0]`).
* arms/waist (held by the upper-body controller): shoulder_pitch 0.3845,
  shoulder_roll ±0.1175, shoulder_yaw −0.0323, elbow −1.2686, wrist −0.0168/−0.0052.

A v8-v10 bundle with these new `default_positions` (or vice-versa) is a mismatch
and will walk wrong. Keep model + default_positions on the SAME pose.

* **gait clock** MUST be gated to zero at |cmd|≤0.1 in the controller (mirror the
  training/sim) or standing will micro-step.

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
