#include "HarambePolicyPlugin.hh"

#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Imu.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/Util.hh>
#include <gz/plugin/Register.hh>
#include <gz/math/Quaternion.hh>

#include <kdl_parser/kdl_parser.hpp>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <sstream>
#include <fstream>

using namespace harambe_gz;

// ============================================================================
// Configure
// ============================================================================
void HarambePolicyPlugin::Configure(
    const gz::sim::Entity &entity,
    const std::shared_ptr<const sdf::Element> &sdf,
    gz::sim::EntityComponentManager &ecm,
    gz::sim::EventManager &)
{
  auto model = gz::sim::Model(entity);
  gzmsg << "HarambePolicyPlugin: Configuring for model " << model.Name(ecm) << std::endl;

  // --- Load ONNX model ---
  std::string onnx_path;
  if (sdf->HasElement("onnx_path")) {
    onnx_path = sdf->Get<std::string>("onnx_path");
  } else {
    gzerr << "HarambePolicyPlugin: <onnx_path> not specified!" << std::endl;
    return;
  }

  ort_env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "HarambePolicy");
  Ort::SessionOptions opts;
  opts.SetIntraOpNumThreads(1);
  ort_session_ = std::make_unique<Ort::Session>(*ort_env_, onnx_path.c_str(), opts);
  gzmsg << "HarambePolicyPlugin: ONNX model loaded from " << onnx_path << std::endl;

  // --- Read parameters ---
  if (sdf->HasElement("decimation")) decimation_ = sdf->Get<int>("decimation");
  if (sdf->HasElement("action_scale")) action_scale_ = sdf->Get<double>("action_scale");
  if (sdf->HasElement("clip_actions")) clip_actions_ = sdf->Get<double>("clip_actions");
  if (sdf->HasElement("warmup_steps")) warmup_steps_ = sdf->Get<int>("warmup_steps");
  if (sdf->HasElement("fall_threshold")) fall_threshold_ = sdf->Get<double>("fall_threshold");
  if (sdf->HasElement("policy_debug")) {
    std::string val = sdf->Get<std::string>("policy_debug");
    policy_debug_ = (val == "true" || val == "1");
  }

  // Velocity commands
  commands_.resize(3, 0.0f);
  if (sdf->HasElement("cmd_vel_x")) commands_[0] = sdf->Get<float>("cmd_vel_x");
  if (sdf->HasElement("cmd_vel_y")) commands_[1] = sdf->Get<float>("cmd_vel_y");
  if (sdf->HasElement("cmd_yaw"))   commands_[2] = sdf->Get<float>("cmd_yaw");
  gz_node_.Subscribe("/cmd_vel", &HarambePolicyPlugin::OnCmdVel, this);

  // KDL gravity compensation — get URDF from model SDF
  {
    auto modelSdf = model.ModelByName(ecm, model.Name(ecm));
    // Try to get URDF from the SDF element's parent (the model description)
    // The URDF is embedded when Gazebo spawns the robot from robot_description topic
    std::string urdf_str;
    if (sdf->HasElement("urdf_path")) {
      // Fallback: explicit URDF path
      std::string urdf_path = sdf->Get<std::string>("urdf_path");
      std::ifstream urdf_file(urdf_path);
      if (urdf_file.is_open()) {
        urdf_str.assign((std::istreambuf_iterator<char>(urdf_file)),
                          std::istreambuf_iterator<char>());
      }
    }
    if (urdf_str.empty()) {
      // Try reading from installed package path
      std::string fallback = "/home/alex/Documents/GitHub/ldr-harambe-arms-sim2real/src/robot_description/full_robot_description/urdf/full_robot_isaaclab.urdf";
      std::ifstream f(fallback);
      if (f.is_open()) {
        urdf_str.assign((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
        gzmsg << "HarambePolicyPlugin: Loaded URDF from fallback path" << std::endl;
      }
    }
    // Store URDF string for later InitKDL (after joints are parsed)
    if (!urdf_str.empty()) {
      urdf_str_ = urdf_str;
    } else {
      gzwarn << "HarambePolicyPlugin: No URDF available, gravity compensation disabled" << std::endl;
    }
  }

  // IMU link names (pelvis + torso)
  pelvis_link_name_ = "urdf_base";
  torso_link_name_ = "urdf_simplified_torso";
  if (sdf->HasElement("imu_link")) pelvis_link_name_ = sdf->Get<std::string>("imu_link");
  if (sdf->HasElement("torso_link")) torso_link_name_ = sdf->Get<std::string>("torso_link");

  // --- Read joint configs ---
  auto jointElem = sdf->FindElement("joint");
  while (jointElem) {
    JointConfig jc;
    jc.name = jointElem->Get<std::string>("name");
    jc.kp = jointElem->Get<double>("kp");
    jc.kd = jointElem->Get<double>("kd");
    jc.effort_limit = jointElem->Get<double>("effort_limit");
    jc.vel_limit = jointElem->Get<double>("vel_limit");
    jc.default_pos = jointElem->Get<double>("default_pos", 0.0).first;
    jc.friction = jointElem->Get<double>("friction", 0.0).first;
    joints_.push_back(jc);
    jointElem = jointElem->GetNextElement("joint");
  }

  num_joints_ = joints_.size();
  gzmsg << "HarambePolicyPlugin: " << num_joints_ << " joints configured" << std::endl;

  // Init arrays (93 = 3 base_lin_vel + 3 pelvis_ang_vel + 3 pelvis_gravity
  //                   + 3 torso_ang_vel + 3 torso_gravity + 3 commands
  //                   + 25 joint_pos + 25 joint_vel + 25 prev_actions)
  obs_.resize(93, 0.0f);
  prev_action_.resize(num_joints_, 0.0f);
  target_pos_.resize(num_joints_, 0.0f);
  for (size_t i = 0; i < num_joints_; ++i) {
    target_pos_[i] = joints_[i].default_pos;
  }

  // Init KDL now that num_joints_ is set
  if (!urdf_str_.empty()) {
    InitKDL(urdf_str_);
    urdf_str_.clear();  // free memory
  }

  // Resolve joint entities
  auto jointEntities = ecm.EntitiesByComponents(gz::sim::components::Joint());
  for (auto &jc : joints_) {
    for (auto je : jointEntities) {
      auto nameComp = ecm.Component<gz::sim::components::Name>(je);
      if (nameComp && nameComp->Data() == jc.name) {
        jc.entity = je;
        if (!ecm.Component<gz::sim::components::JointPosition>(je))
          ecm.CreateComponent(je, gz::sim::components::JointPosition({jc.default_pos}));
        else
          ecm.SetComponentData<gz::sim::components::JointPosition>(je, {jc.default_pos});
        if (!ecm.Component<gz::sim::components::JointVelocity>(je))
          ecm.CreateComponent(je, gz::sim::components::JointVelocity({0.0}));
        break;
      }
    }
    if (jc.entity == gz::sim::kNullEntity) {
      gzerr << "HarambePolicyPlugin: Joint '" << jc.name << "' not found!" << std::endl;
    }
  }

  // Resolve IMU link entities (pelvis + torso)
  auto linkEntities = ecm.EntitiesByComponents(gz::sim::components::Link());
  for (auto le : linkEntities) {
    auto nameComp = ecm.Component<gz::sim::components::Name>(le);
    if (!nameComp) continue;
    if (nameComp->Data() == pelvis_link_name_) {
      pelvis_entity_ = le;
      if (!ecm.Component<gz::sim::components::AngularVelocity>(le))
        ecm.CreateComponent(le, gz::sim::components::AngularVelocity());
      if (!ecm.Component<gz::sim::components::LinearVelocity>(le))
        ecm.CreateComponent(le, gz::sim::components::LinearVelocity());
    }
    if (nameComp->Data() == torso_link_name_) {
      torso_entity_ = le;
      if (!ecm.Component<gz::sim::components::AngularVelocity>(le))
        ecm.CreateComponent(le, gz::sim::components::AngularVelocity());
    }
  }
  if (torso_entity_ == gz::sim::kNullEntity) {
    gzwarn << "HarambePolicyPlugin: Torso link '" << torso_link_name_ << "' not found!" << std::endl;
  }

  // Init debug publisher (only if policy_debug enabled)
  if (policy_debug_) {
    // Layout: [efforts(25) | targets(25) | positions(25) | velocities(25) | actions(25) | obs(93)] = 218
    debug_pub_ = gz_node_.Advertise<gz::msgs::Float_V>("/policy/debug");
    gzmsg << "HarambePolicyPlugin: Debug publishing enabled on /policy/debug" << std::endl;
  }
  last_efforts_.resize(num_joints_, 0.0);
  last_positions_.resize(num_joints_, 0.0);
  last_velocities_.resize(num_joints_, 0.0);

  initialized_ = true;
  gzmsg << "HarambePolicyPlugin: Ready! decimation=" << decimation_
        << " action_scale=" << action_scale_
        << " clip_actions=" << clip_actions_ << std::endl;
}

// ============================================================================
// PreUpdate — runs every physics step
// ============================================================================
void HarambePolicyPlugin::PreUpdate(
    const gz::sim::UpdateInfo &info,
    gz::sim::EntityComponentManager &ecm)
{
  if (!initialized_ || info.paused) return;

  // Ramp-up: gradually move targets from 0 to default_pos
  // Set to 0 when warmup=0 to start policy immediately from joints=0
  const int ramp_steps = (warmup_steps_ > 0) ? 200 : 0;
  if (step_counter_ < ramp_steps) {
    double alpha = static_cast<double>(step_counter_ + 1) / ramp_steps;
    for (size_t i = 0; i < num_joints_; ++i) {
      target_pos_[i] = alpha * joints_[i].default_pos;
    }
  }

  // Policy inference at decimated rate
  if (step_counter_ % decimation_ == 0) {
    BuildObservation(ecm);

    // Fall detection: if |pelvis_gravity_z| < threshold, robot has fallen
    float grav_z = obs_[8];  // pelvis projected gravity Z
    if (policy_active_ && !fallen_ && std::abs(grav_z) < fall_threshold_) {
      fallen_ = true;
      gzmsg << "HarambePolicyPlugin: FALL DETECTED at step " << step_counter_
            << " (grav_z=" << grav_z << ", threshold=" << fall_threshold_
            << "). Disabling policy (action_scale=0)." << std::endl;
    }

    // Active policy after both ramp and warmup complete
    if (step_counter_ >= ramp_steps && policy_step_ >= warmup_steps_) {
      RunPolicyInference();
      policy_active_ = true;
    }

    policy_step_++;

    if (policy_step_ == warmup_steps_ + 1) {
      gzmsg << "HarambePolicyPlugin: Warmup complete, policy active!" << std::endl;
      // Dump first observation for debugging
      std::ostringstream oss;
      oss << "OBS_DUMP: ";
      for (size_t i = 0; i < obs_.size(); ++i) oss << obs_[i] << ",";
      gzmsg << oss.str() << std::endl;
    }
  }

  // Isaac Lab uses HOLD: target is applied instantly and held constant
  // throughout the decimation period (no interpolation).

  // Gravity compensation + PD control at every physics step
  ComputeGravityCompensation(ecm);
  ApplyPDControl(ecm);

  // Periodic logging
  if (step_counter_ % 200 == 0 && step_counter_ > 0) {
    float max_vel = 0;
    std::string max_vel_joint;
    for (size_t i = 0; i < num_joints_; ++i) {
      if (joints_[i].entity == gz::sim::kNullEntity) continue;
      auto velComp = ecm.Component<gz::sim::components::JointVelocity>(joints_[i].entity);
      if (velComp && !velComp->Data().empty()) {
        double v = std::abs(velComp->Data()[0]);
        if (v > max_vel) { max_vel = v; max_vel_joint = joints_[i].name; }
      }
    }
    // Action stats
    float max_act = 0, act_sum = 0;
    for (size_t i = 0; i < num_joints_; ++i) {
      act_sum += std::abs(prev_action_[i]);
      if (std::abs(prev_action_[i]) > max_act) max_act = std::abs(prev_action_[i]);
    }
    gzmsg << "[step " << step_counter_
          << " policy_step=" << policy_step_
          << "] cmd=[" << commands_[0] << "," << commands_[1] << "," << commands_[2]
          << "] act_max=" << max_act << " act_sum=" << act_sum
          << " pelvis_grav=[" << obs_[6] << "," << obs_[7] << "," << obs_[8]
          << "] max_vel=" << max_vel << " (" << max_vel_joint
          << ") max_eff=" << *std::max_element(last_efforts_.begin(), last_efforts_.end())
          << " target0=" << target_pos_[13]
          << " pos0=" << last_positions_[13]
          << " policy_active=" << policy_active_ << std::endl;
  }

  step_counter_++;
}

// ============================================================================
// Build 93-dim observation (dual IMU: pelvis + torso)
// ============================================================================
// Layout: [base_lin_vel(3) | pelvis_ang_vel(3) | pelvis_gravity(3)
//        | torso_ang_vel(3) | torso_gravity(3) | commands(3)
//        | joint_pos(25) | joint_vel(25) | prev_actions(25)]
//
// IMU sensor transforms match training (observations.py):
//   Pelvis IMU (rpy="pi -pi/2 0"): sensor=[base_z, -base_y, base_x]
//     roundtrip: base→sensor→base is identity (self-inverse transform)
//   Torso IMU (rpy="0 pi/2 0"):    sensor=[torso_z, torso_y, -torso_x]
//     roundtrip: torso→sensor→torso is identity (self-inverse transform)
// Since base→sensor→base = identity, the deploy path just uses body-frame
// values directly. We do the same here.
// ============================================================================
void HarambePolicyPlugin::BuildObservation(gz::sim::EntityComponentManager &ecm)
{
  // --- Pelvis (urdf_base) ---
  if (pelvis_entity_ != gz::sim::kNullEntity) {
    auto worldPose = gz::sim::worldPose(pelvis_entity_, ecm);
    auto q = worldPose.Rot();

    // [0:3] base_lin_vel: world -> pelvis body frame
    auto linVelComp = ecm.Component<gz::sim::components::LinearVelocity>(pelvis_entity_);
    if (linVelComp) {
      auto v_body = q.RotateVectorReverse(linVelComp->Data());
      obs_[0] = static_cast<float>(v_body.X());
      obs_[1] = static_cast<float>(v_body.Y());
      obs_[2] = static_cast<float>(v_body.Z());
    } else {
      obs_[0] = obs_[1] = obs_[2] = 0.0f;
    }

    // [3:6] pelvis_ang_vel: world -> pelvis body frame
    auto angVelComp = ecm.Component<gz::sim::components::AngularVelocity>(pelvis_entity_);
    if (angVelComp) {
      auto w_body = q.RotateVectorReverse(angVelComp->Data());
      obs_[3] = static_cast<float>(w_body.X());
      obs_[4] = static_cast<float>(w_body.Y());
      obs_[5] = static_cast<float>(w_body.Z());
    } else {
      obs_[3] = obs_[4] = obs_[5] = 0.0f;
    }

    // [6:9] pelvis_projected_gravity: world -> pelvis body frame
    auto grav_body = q.RotateVectorReverse(gz::math::Vector3d(0, 0, -1));
    obs_[6] = static_cast<float>(grav_body.X());
    obs_[7] = static_cast<float>(grav_body.Y());
    obs_[8] = static_cast<float>(grav_body.Z());
  }

  // --- Torso (urdf_simplified_torso) ---
  if (torso_entity_ != gz::sim::kNullEntity) {
    auto torsoWorldPose = gz::sim::worldPose(torso_entity_, ecm);
    auto qt = torsoWorldPose.Rot();

    // [9:12] torso_ang_vel: world -> torso body frame
    auto torsoAngVelComp = ecm.Component<gz::sim::components::AngularVelocity>(torso_entity_);
    if (torsoAngVelComp) {
      auto w_torso = qt.RotateVectorReverse(torsoAngVelComp->Data());
      obs_[9]  = static_cast<float>(w_torso.X());
      obs_[10] = static_cast<float>(w_torso.Y());
      obs_[11] = static_cast<float>(w_torso.Z());
    } else {
      obs_[9] = obs_[10] = obs_[11] = 0.0f;
    }

    // [12:15] torso_projected_gravity: world -> torso body frame
    auto grav_torso = qt.RotateVectorReverse(gz::math::Vector3d(0, 0, -1));
    obs_[12] = static_cast<float>(grav_torso.X());
    obs_[13] = static_cast<float>(grav_torso.Y());
    obs_[14] = static_cast<float>(grav_torso.Z());
  } else {
    // Fallback: copy pelvis values if torso not found
    obs_[9] = obs_[3]; obs_[10] = obs_[4]; obs_[11] = obs_[5];
    obs_[12] = obs_[6]; obs_[13] = obs_[7]; obs_[14] = obs_[8];
  }

  // [15:18] commands
  obs_[15] = commands_[0];
  obs_[16] = commands_[1];
  obs_[17] = commands_[2];

  // [18:43] joint_pos_rel (25) and [43:68] joint_vel (25)
  for (size_t i = 0; i < num_joints_; ++i) {
    double pos = 0.0, vel = 0.0;
    if (joints_[i].entity != gz::sim::kNullEntity) {
      auto posComp = ecm.Component<gz::sim::components::JointPosition>(joints_[i].entity);
      auto velComp = ecm.Component<gz::sim::components::JointVelocity>(joints_[i].entity);
      if (posComp && !posComp->Data().empty()) pos = posComp->Data()[0];
      if (velComp && !velComp->Data().empty()) vel = velComp->Data()[0];
    }
    obs_[18 + i] = static_cast<float>(pos - joints_[i].default_pos);
    obs_[43 + i] = static_cast<float>(vel);
  }

  // [68:93] prev_action (25)
  for (size_t i = 0; i < num_joints_; ++i) {
    obs_[68 + i] = prev_action_[i];
  }
}

// ============================================================================
// Run ONNX inference
// ============================================================================
void HarambePolicyPlugin::RunPolicyInference()
{
  if (!ort_session_) return;

  // ONNX inference
  std::array<int64_t, 2> input_shape = {1, static_cast<int64_t>(obs_.size())};
  auto input_tensor = Ort::Value::CreateTensor<float>(
      ort_mem_info_, obs_.data(), obs_.size(),
      input_shape.data(), input_shape.size());

  const char* input_names[] = {"obs"};
  const char* output_names[] = {"actions"};

  auto output_tensors = ort_session_->Run(
      Ort::RunOptions{nullptr},
      input_names, &input_tensor, 1,
      output_names, 1);

  float* actions = output_tensors[0].GetTensorMutableData<float>();

  // Clip actions and set new target
  double scale = fallen_ ? 0.0 : action_scale_;
  for (size_t i = 0; i < num_joints_; ++i) {
    float a = actions[i];
    a = std::clamp(a, static_cast<float>(-clip_actions_), static_cast<float>(clip_actions_));
    prev_action_[i] = a;
    target_pos_[i] = a * scale + joints_[i].default_pos;
  }

}

// ============================================================================
// Apply PD control + gravity compensation (matching PhysX ImplicitActuator)
// τ = kp*(target - pos) - kd*vel + g(q)
// ============================================================================
void HarambePolicyPlugin::ApplyPDControl(gz::sim::EntityComponentManager &ecm)
{
  for (size_t i = 0; i < num_joints_; ++i) {
    if (joints_[i].entity == gz::sim::kNullEntity) continue;

    auto posComp = ecm.Component<gz::sim::components::JointPosition>(joints_[i].entity);
    auto velComp = ecm.Component<gz::sim::components::JointVelocity>(joints_[i].entity);
    if (!posComp || posComp->Data().empty() || !velComp || velComp->Data().empty()) continue;

    double pos = posComp->Data()[0];
    double vel = velComp->Data()[0];
    last_positions_[i] = pos;
    last_velocities_[i] = vel;
    double target = target_pos_[i];
    double kp = joints_[i].kp;
    double kd = joints_[i].kd;
    double effort_limit = joints_[i].effort_limit;

    // PD torque + gravity compensation
    double grav_comp = (kdl_ready_ && i < gravity_torques_.size()) ? gravity_torques_[i] : 0.0;
    double torque = kp * (target - pos) - kd * vel + grav_comp;

    // Effort clamp
    torque = std::clamp(torque, -effort_limit, effort_limit);

    // No velocity limiting or acceleration clamping — match Isaac Lab ImplicitActuator
    // which only applies effort clamp (done above).

    // Store for debug publishing
    last_efforts_[i] = torque;

    // Apply force
    auto forceComp = ecm.Component<gz::sim::components::JointForceCmd>(joints_[i].entity);
    if (forceComp) {
      forceComp->Data() = {torque};
    } else {
      ecm.CreateComponent(joints_[i].entity, gz::sim::components::JointForceCmd({torque}));
    }
  }

  // Publish all policy data on single topic at policy rate (50 Hz)
  if (policy_debug_ && step_counter_ % decimation_ == 0) {
    gz::msgs::Float_V msg;
    for (size_t i = 0; i < num_joints_; ++i)
      msg.add_data(static_cast<float>(last_efforts_[i]));
    for (size_t i = 0; i < num_joints_; ++i)
      msg.add_data(static_cast<float>(target_pos_[i]));
    for (size_t i = 0; i < num_joints_; ++i)
      msg.add_data(static_cast<float>(last_positions_[i]));
    for (size_t i = 0; i < num_joints_; ++i)
      msg.add_data(static_cast<float>(last_velocities_[i]));
    for (size_t i = 0; i < num_joints_; ++i)
      msg.add_data(prev_action_[i]);
    for (size_t i = 0; i < obs_.size(); ++i)
      msg.add_data(obs_[i]);
    debug_pub_.Publish(msg);
  }
}

// ============================================================================
// Velocity command callback
// ============================================================================
void HarambePolicyPlugin::OnCmdVel(const gz::msgs::Twist &msg)
{
  commands_[0] = static_cast<float>(msg.linear().x());
  commands_[1] = static_cast<float>(msg.linear().y());
  commands_[2] = static_cast<float>(msg.angular().z());
}

// ============================================================================
// Initialize KDL from URDF for gravity compensation
// ============================================================================
bool HarambePolicyPlugin::InitKDL(const std::string &urdf_string)
{
  if (!kdl_parser::treeFromString(urdf_string, kdl_tree_)) {
    gzerr << "HarambePolicyPlugin: Failed to parse URDF into KDL tree" << std::endl;
    return false;
  }

  gravity_torques_.resize(num_joints_, 0.0);

  // Define kinematic chains from base to end-effectors
  // Each chain covers a limb's joints
  struct ChainDef {
    std::string root;
    std::string tip;
    std::vector<std::string> joint_names;  // joints in this chain (in chain order)
  };

  std::vector<ChainDef> chain_defs = {
    {"urdf_base", "urdf_foot_assembly",     // left leg (through ankle_roll)
     {"left_hip_pitch_joint_X8", "left_hip_roll_joint_X8", "left_hip_yaw_joint_X8",
      "left_knee_joint_X8", "left_ankle_pitch_joint_X4", "left_ankle_roll_joint_X4"}},
    {"urdf_base", "urdf_foot_assembly_2",  // right leg (through ankle_roll)
     {"right_hip_pitch_joint_X8", "right_hip_roll_joint_X8", "right_hip_yaw_joint_X8",
      "right_knee_joint_X8", "right_ankle_pitch_joint_X4", "right_ankle_roll_joint_X4"}},
    {"urdf_base", "urdf_l_wrist_assembly", // left arm
     {"left_shoulder_pitch_joint_X6", "left_shoulder_roll_joint_X6", "left_shoulder_yaw_joint_X4",
      "left_elbow_pitch_joint_X6", "left_wrist_yaw_joint_X4", "left_wrist_roll_joint_X4"}},
    {"urdf_base", "urdf_r_wrist_assembly", // right arm
     {"right_shoulder_pitch_joint_X6", "right_shoulder_roll_joint_X6", "right_shoulder_yaw_joint_X4",
      "right_elbow_pitch_joint_X6", "right_wrist_yaw_joint_X4", "right_wrist_roll_joint_X4"}},
  };

  for (auto &def : chain_defs) {
    KDLChainData cd;
    if (!kdl_tree_.getChain(def.root, def.tip, cd.chain)) {
      gzwarn << "HarambePolicyPlugin: KDL chain " << def.root << " -> " << def.tip
             << " not found, skipping gravity compensation for this chain" << std::endl;
      continue;
    }

    // Map KDL chain joints to plugin joint indices
    int kdl_seg_idx = 0;
    for (unsigned s = 0; s < cd.chain.getNrOfSegments(); ++s) {
      const auto &seg = cd.chain.getSegment(s);
      if (seg.getJoint().getType() == KDL::Joint::None) continue;

      std::string jname = seg.getJoint().getName();
      int plugin_idx = -1;
      for (size_t j = 0; j < num_joints_; ++j) {
        if (joints_[j].name == jname) {
          plugin_idx = static_cast<int>(j);
          break;
        }
      }
      cd.joint_map.push_back(plugin_idx);
      kdl_seg_idx++;
    }

    // Create dynamics solver
    KDL::Vector grav(0, 0, -9.81);
    cd.dynamics = std::make_unique<KDL::ChainDynParam>(cd.chain, grav);

    gzmsg << "HarambePolicyPlugin: KDL chain " << def.root << " -> " << def.tip
           << " (" << cd.chain.getNrOfJoints() << " DOF, "
           << cd.joint_map.size() << " mapped)" << std::endl;

    kdl_chains_.push_back(std::move(cd));
  }

  kdl_ready_ = !kdl_chains_.empty();
  gzmsg << "HarambePolicyPlugin: KDL gravity compensation "
        << (kdl_ready_ ? "READY" : "DISABLED") << " (" << kdl_chains_.size() << " chains)" << std::endl;
  return kdl_ready_;
}

// ============================================================================
// Compute gravity compensation torques using KDL
// ============================================================================
void HarambePolicyPlugin::ComputeGravityCompensation(gz::sim::EntityComponentManager &ecm)
{
  if (!kdl_ready_) return;

  // Reset gravity torques
  std::fill(gravity_torques_.begin(), gravity_torques_.end(), 0.0);

  for (auto &cd : kdl_chains_) {
    unsigned n = cd.chain.getNrOfJoints();
    KDL::JntArray q(n), grav_torques(n);

    // Fill joint positions from simulation
    unsigned kdl_idx = 0;
    for (unsigned s = 0; s < cd.chain.getNrOfSegments(); ++s) {
      const auto &seg = cd.chain.getSegment(s);
      if (seg.getJoint().getType() == KDL::Joint::None) continue;

      if (kdl_idx < cd.joint_map.size() && cd.joint_map[kdl_idx] >= 0) {
        int pi = cd.joint_map[kdl_idx];
        auto posComp = ecm.Component<gz::sim::components::JointPosition>(joints_[pi].entity);
        if (posComp && !posComp->Data().empty()) {
          q(kdl_idx) = posComp->Data()[0];
        }
      }
      kdl_idx++;
    }

    // Compute gravity torques: g(q)
    int ret = cd.dynamics->JntToGravity(q, grav_torques);

    // Debug: log once
    static bool grav_debug = false;
    if (!grav_debug && policy_active_) {
      grav_debug = true;
      std::ostringstream dbg;
      dbg << "KDL_GRAV_DEBUG: chain_joints=" << n
          << " chain_segs=" << cd.chain.getNrOfSegments()
          << " ret=" << ret << " q=[";
      for (unsigned j = 0; j < n; ++j) dbg << q(j) << ",";
      dbg << "] torques=[";
      for (unsigned j = 0; j < n; ++j) dbg << grav_torques(j) << ",";
      dbg << "] map=[";
      for (unsigned j = 0; j < cd.joint_map.size(); ++j) dbg << cd.joint_map[j] << ",";
      dbg << "]";
      gzmsg << dbg.str() << std::endl;
    }

    // Map back to plugin joint indices
    for (unsigned k = 0; k < cd.joint_map.size() && k < n; ++k) {
      if (cd.joint_map[k] >= 0) {
        gravity_torques_[cd.joint_map[k]] = grav_torques(k);
      }
    }
  }
}

// ============================================================================
// Plugin registration
// ============================================================================
GZ_ADD_PLUGIN(
    harambe_gz::HarambePolicyPlugin,
    gz::sim::System,
    harambe_gz::HarambePolicyPlugin::ISystemConfigure,
    harambe_gz::HarambePolicyPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(harambe_gz::HarambePolicyPlugin, "harambe_gz::HarambePolicyPlugin")
