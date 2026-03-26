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

  // Velocity commands
  commands_.resize(3, 0.0f);
  if (sdf->HasElement("cmd_vel_x")) commands_[0] = sdf->Get<float>("cmd_vel_x");
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

  // IMU link name
  imu_link_name_ = "urdf_base";
  if (sdf->HasElement("imu_link")) imu_link_name_ = sdf->Get<std::string>("imu_link");

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

  // Init arrays
  obs_.resize(87, 0.0f);
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

  // Resolve IMU link entity
  auto linkEntities = ecm.EntitiesByComponents(gz::sim::components::Link());
  for (auto le : linkEntities) {
    auto nameComp = ecm.Component<gz::sim::components::Name>(le);
    if (nameComp && nameComp->Data() == imu_link_name_) {
      imu_entity_ = le;
      if (!ecm.Component<gz::sim::components::AngularVelocity>(le))
        ecm.CreateComponent(le, gz::sim::components::AngularVelocity());
      if (!ecm.Component<gz::sim::components::LinearVelocity>(le))
        ecm.CreateComponent(le, gz::sim::components::LinearVelocity());
      break;
    }
  }

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

    // TEST: Replace ENTIRE obs with normalizer mean from training.
    // If this produces reasonable actions, the problem is purely obs mismatch.
    // Mean from model_16500.pt actor_obs_normalizer._mean
    static const float norm_mean[87] = {
      0.014560f,0.002344f,-0.020510f,0.007439f,-0.073306f,0.019766f,
      0.098973f,0.007896f,-0.983043f,0.052877f,0.000000f,0.000588f,
      0.004244f,0.000819f,0.028119f,0.007895f,-0.005145f,0.025967f,
      0.003376f,0.000595f,-0.022207f,0.005684f,0.003184f,-0.025144f,
      0.013423f,-0.156746f,0.009333f,0.009215f,-0.019751f,0.078313f,
      0.001897f,-0.121752f,0.002539f,0.010357f,-0.099289f,0.122456f,
      0.007407f,0.030737f,0.018185f,-0.000567f,-0.014012f,0.000053f,
      -0.000357f,0.024137f,-0.012236f,0.000139f,-0.015583f,0.000010f,
      0.000103f,-0.022809f,0.162343f,-0.020397f,-0.005266f,-0.099728f,
      -0.080329f,0.066473f,0.189450f,-0.017626f,-0.000434f,-0.136484f,
      -0.086309f,0.051259f,-1.289845f,1.301810f,0.221760f,-0.944547f,
      -0.550033f,0.620776f,-1.059761f,-1.246093f,0.466370f,-0.885992f,
      0.124125f,0.033217f,-0.024503f,-0.005407f,0.483497f,-0.168474f,
      -1.067385f,-0.111955f,-0.615379f,0.162255f,-0.312931f,-0.092159f,
      -0.676582f,0.191935f,0.044751f
    };
    // No obs override — use real Gazebo observations

    // Active policy after both ramp and warmup complete
    if (step_counter_ >= ramp_steps && policy_step_ >= warmup_steps_) {
      RunPolicyInference();
      policy_active_ = true;
    }

    policy_step_++;

    if (policy_step_ == warmup_steps_ + 1) {
      gzmsg << "HarambePolicyPlugin: Warmup complete, policy active!" << std::endl;
      float min_a = *std::min_element(prev_action_.begin(), prev_action_.end());
      float max_a = *std::max_element(prev_action_.begin(), prev_action_.end());
      float min_t = *std::min_element(target_pos_.begin(), target_pos_.end());
      float max_t = *std::max_element(target_pos_.begin(), target_pos_.end());
      gzmsg << "  First action range: [" << min_a << ", " << max_a
            << "] target range: [" << min_t << ", " << max_t << "]" << std::endl;

      // ONE-TIME: compare TorchScript output for Isaac Lab obs vs Gazebo obs
      // Isaac Lab step 3 — full dynamics, vel non-zero, prev_action non-zero
      std::vector<float> il_obs = {
        0.007458f,0.093524f,-0.049077f,0.129523f,0.051964f,-0.160343f,-0.001306f,-0.008643f,-0.999962f,
        0.0f,0.0f,0.0f,
        -0.027079f,0.085381f,-0.043764f,0.039680f,0.055820f,-0.066059f,
        0.053917f,0.012628f,0.068283f,0.033938f,-0.024699f,-0.033329f,
        0.005063f,
        0.059609f,-0.046991f,-0.080975f,-0.045286f,-0.009825f,0.030576f,
        -0.029107f,0.023486f,-0.041580f,0.015669f,0.013662f,-0.039199f,
        0.552648f,-0.427787f,0.000252f,-0.002744f,0.000318f,-0.000485f,
        -0.765485f,0.162634f,0.000458f,0.534313f,0.000199f,-0.000073f,
        0.082681f,0.267782f,-0.236424f,0.000630f,-0.454495f,-0.245530f,
        -0.088054f,-0.361539f,-0.219966f,0.376734f,0.589851f,-0.544534f,
        0.171338f,
        -2.006419f,-1.771927f,-0.095952f,-3.563867f,-1.569299f,3.644118f,
        -3.786095f,-0.200554f,0.074973f,-3.382048f,1.168368f,0.318866f,
        -1.239805f,0.737682f,1.178721f,2.757662f,-1.306519f,0.606112f,
        -0.215659f,-1.037031f,-1.457611f,1.631150f,-3.866488f,2.683407f,-0.826728f
      };  // 87 values from Isaac Lab FULL_OBS_87 step 3
      // Run ONNX on both IL and GZ obs for comparison
      auto run_onnx = [&](std::vector<float>& input) -> std::vector<float> {
        std::array<int64_t, 2> shape = {1, 87};
        auto tensor = Ort::Value::CreateTensor<float>(
            ort_mem_info_, input.data(), input.size(), shape.data(), shape.size());
        const char* in_n[] = {"obs"};
        const char* out_n[] = {"actions"};
        auto out = ort_session_->Run(Ort::RunOptions{nullptr}, in_n, &tensor, 1, out_n, 1);
        float* d = out[0].GetTensorMutableData<float>();
        return std::vector<float>(d, d + 25);
      };
      auto il_act = run_onnx(il_obs);
      auto gz_act = run_onnx(obs_);

      const char* jnames[] = {"l_sh_p","l_sh_r","l_sh_y","l_elb","l_wr_y","l_wr_r",
        "r_sh_p","r_sh_r","r_sh_y","r_elb","r_wr_y","r_wr_r","waist",
        "l_hip_p","l_hip_r","l_hip_y","l_knee","l_ank_p","l_ank_r",
        "r_hip_p","r_hip_r","r_hip_y","r_knee","r_ank_p","r_ank_r"};
      gzmsg << "=== ONNX: ISAAC vs GAZEBO ===" << std::endl;
      for (int i = 0; i < 25; ++i) {
        gzmsg << "  " << jnames[i]
              << " IL=" << il_act[i] << " GZ=" << gz_act[i]
              << " diff=" << (il_act[i] - gz_act[i]) << std::endl;
      }
    }
  }

  // PD control at every physics step
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
    gzmsg << "[step " << step_counter_
          << " policy_step=" << policy_step_
          << "] gravity=[" << obs_[6] << "," << obs_[7] << "," << obs_[8]
          << "] max_vel=" << max_vel << " (" << max_vel_joint
          << ") policy_active=" << policy_active_ << std::endl;
  }

  step_counter_++;
}

// ============================================================================
// Build 87-dim observation
// ============================================================================
void HarambePolicyPlugin::BuildObservation(gz::sim::EntityComponentManager &ecm)
{
  // base_lin_vel (3)
  if (imu_entity_ != gz::sim::kNullEntity) {
    auto linVelComp = ecm.Component<gz::sim::components::LinearVelocity>(imu_entity_);
    if (linVelComp) {
      auto v = linVelComp->Data();
      obs_[0] = static_cast<float>(v.X());
      obs_[1] = static_cast<float>(v.Y());
      obs_[2] = static_cast<float>(v.Z());
    } else {
      obs_[0] = obs_[1] = obs_[2] = 0.0f;
    }
  } else {
    obs_[0] = obs_[1] = obs_[2] = 0.0f;
  }

  // base_ang_vel (3) and projected_gravity (3)
  if (imu_entity_ != gz::sim::kNullEntity) {
    auto worldPose = gz::sim::worldPose(imu_entity_, ecm);
    auto q = worldPose.Rot();

    auto grav_world = gz::math::Vector3d(0, 0, -1);
    auto grav_body = q.RotateVectorReverse(grav_world);
    obs_[6] = static_cast<float>(grav_body.X());
    obs_[7] = static_cast<float>(grav_body.Y());
    obs_[8] = static_cast<float>(grav_body.Z());

    auto angVelComp = ecm.Component<gz::sim::components::AngularVelocity>(imu_entity_);
    if (angVelComp) {
      auto w = angVelComp->Data();
      obs_[3] = static_cast<float>(w.X());
      obs_[4] = static_cast<float>(w.Y());
      obs_[5] = static_cast<float>(w.Z());
    } else {
      obs_[3] = obs_[4] = obs_[5] = 0.0f;
    }
  }

  // commands (3)
  obs_[9] = commands_[0];
  obs_[10] = commands_[1];
  obs_[11] = commands_[2];

  // joint_pos_rel (25) and joint_vel (25)
  for (size_t i = 0; i < num_joints_; ++i) {
    double pos = 0.0, vel = 0.0;
    if (joints_[i].entity != gz::sim::kNullEntity) {
      auto posComp = ecm.Component<gz::sim::components::JointPosition>(joints_[i].entity);
      auto velComp = ecm.Component<gz::sim::components::JointVelocity>(joints_[i].entity);
      if (posComp && !posComp->Data().empty()) pos = posComp->Data()[0];
      if (velComp && !velComp->Data().empty()) vel = velComp->Data()[0];
    }
    obs_[12 + i] = static_cast<float>(pos - joints_[i].default_pos);
    obs_[37 + i] = static_cast<float>(vel);
  }

  // prev_action (25)
  for (size_t i = 0; i < num_joints_; ++i) {
    obs_[62 + i] = prev_action_[i];
  }

  // Log observations
  bool at_activation = (policy_step_ >= warmup_steps_ - 1 && policy_step_ <= warmup_steps_ + 5);
  if (policy_step_ < 5 || at_activation || policy_step_ % 250 == 0) {
    gzmsg << "[obs step " << policy_step_ << "] "
          << "lin_vel=[" << obs_[0] << "," << obs_[1] << "," << obs_[2] << "] "
          << "ang_vel=[" << obs_[3] << "," << obs_[4] << "," << obs_[5] << "] "
          << "gravity=[" << obs_[6] << "," << obs_[7] << "," << obs_[8] << "] "
          << "cmd=[" << obs_[9] << "," << obs_[10] << "," << obs_[11] << "]"
          << std::endl;
    gzmsg << "  pos_rel_arm=[" << obs_[12] << "," << obs_[13] << "," << obs_[14]
          << "," << obs_[15] << "," << obs_[16] << "," << obs_[17] << "]"
          << " pos_rel_leg=[" << obs_[25] << "," << obs_[26] << "," << obs_[27]
          << "," << obs_[28] << "," << obs_[29] << "," << obs_[30] << "]"
          << std::endl;
    float min_vel = *std::min_element(obs_.begin() + 37, obs_.begin() + 62);
    float max_vel = *std::max_element(obs_.begin() + 37, obs_.begin() + 62);
    gzmsg << "  vel_range=[" << min_vel << "," << max_vel << "]"
          << " action_range=[" << *std::min_element(prev_action_.begin(), prev_action_.end())
          << "," << *std::max_element(prev_action_.begin(), prev_action_.end()) << "]"
          << std::endl;

    if (at_activation && policy_step_ == warmup_steps_) {
      std::ostringstream oss;
      oss << "FULL_OBS_87:";
      for (int j = 0; j < 87; ++j) {
        oss << obs_[j];
        if (j < 86) oss << ",";
      }
      gzmsg << oss.str() << std::endl;
    }
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

  // Clip actions and apply
  for (size_t i = 0; i < num_joints_; ++i) {
    float a = actions[i];
    a = std::clamp(a, static_cast<float>(-clip_actions_), static_cast<float>(clip_actions_));
    prev_action_[i] = a;
    target_pos_[i] = a * action_scale_ + joints_[i].default_pos;
  }

  // Log per-joint details for first 10 active policy steps
  static int active_steps = 0;
  if (active_steps < 10) {
    gzmsg << "[policy step " << active_steps << "] per-joint actions/targets:" << std::endl;
    for (size_t i = 0; i < num_joints_; ++i) {
      gzmsg << "  [" << i << "] " << joints_[i].name
            << " action=" << prev_action_[i]
            << " target=" << target_pos_[i]
            << " default=" << joints_[i].default_pos
            << std::endl;
    }
    active_steps++;
  }
}

// ============================================================================
// Apply PD control + gravity compensation (matching PhysX ImplicitActuator)
// τ = kp*(target - pos) - kd*vel + g(q)
// ============================================================================
void HarambePolicyPlugin::ApplyPDControl(gz::sim::EntityComponentManager &ecm)
{
  // Gravity compensation disabled — KDL returns error -3, and at joints=0
  // gravity torques are ~0 anyway (gravity passes through joint axes when legs straight)
  // ComputeGravityCompensation(ecm);
  // Log gravity torques once at activation
  static bool grav_logged = false;
  if (!grav_logged && policy_active_) {
    grav_logged = true;
    const char* jn[] = {"l_sh_p","l_sh_r","l_sh_y","l_elb","l_wr_y","l_wr_r",
      "r_sh_p","r_sh_r","r_sh_y","r_elb","r_wr_y","r_wr_r","waist",
      "l_hip_p","l_hip_r","l_hip_y","l_knee","l_ank_p","l_ank_r",
      "r_hip_p","r_hip_r","r_hip_y","r_knee","r_ank_p","r_ank_r"};
    std::ostringstream oss;
    oss << "GRAV_TORQUES(size=" << gravity_torques_.size() << " kdl_ready=" << kdl_ready_ << "):";
    for (size_t i = 0; i < std::min(gravity_torques_.size(), (size_t)25); ++i) {
      oss << jn[i] << "=" << gravity_torques_[i] << " ";
    }
    gzmsg << oss.str() << std::endl;
  }

  for (size_t i = 0; i < num_joints_; ++i) {
    if (joints_[i].entity == gz::sim::kNullEntity) continue;

    auto posComp = ecm.Component<gz::sim::components::JointPosition>(joints_[i].entity);
    auto velComp = ecm.Component<gz::sim::components::JointVelocity>(joints_[i].entity);
    if (!posComp || posComp->Data().empty() || !velComp || velComp->Data().empty()) continue;

    double pos = posComp->Data()[0];
    double vel = velComp->Data()[0];
    double target = target_pos_[i];
    double kp = joints_[i].kp;
    double kd = joints_[i].kd;
    double effort_limit = joints_[i].effort_limit;
    double vel_limit = joints_[i].vel_limit;

    // PD torque
    double pd_torque = kp * (target - pos) - kd * vel;

    // Friction compensation: counteract URDF Coulomb friction partially.
    // URDF applies τ_friction = -friction * sign(vel) which opposes motion.
    // We add friction * sign(error) scaled by how much error remains
    // (full comp at small error where friction dominates, less at large error).
    double error = target - pos;
    double friction_comp = 0.0;
    if (std::abs(error) > 0.001 && joints_[i].friction > 0) {
      // Scale: 100% comp when |error|<0.05, 50% when |error|>0.15
      double scale = std::clamp(1.0 - (std::abs(error) - 0.05) / 0.10, 0.5, 1.0);
      friction_comp = joints_[i].friction * scale * (error > 0 ? 1.0 : -1.0);
    }

    double torque = pd_torque + friction_comp;

    // Effort clamp
    torque = std::clamp(torque, -effort_limit, effort_limit);

    // Velocity limiting
    if (vel > vel_limit && torque > 0.0) {
      torque = -effort_limit;
    } else if (vel < -vel_limit && torque < 0.0) {
      torque = effort_limit;
    }

    // Apply force
    auto forceComp = ecm.Component<gz::sim::components::JointForceCmd>(joints_[i].entity);
    if (forceComp) {
      forceComp->Data() = {torque};
    } else {
      ecm.CreateComponent(joints_[i].entity, gz::sim::components::JointForceCmd({torque}));
    }
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
