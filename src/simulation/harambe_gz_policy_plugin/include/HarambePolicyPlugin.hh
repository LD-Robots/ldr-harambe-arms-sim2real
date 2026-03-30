#ifndef HARAMBE_POLICY_PLUGIN_HH_
#define HARAMBE_POLICY_PLUGIN_HH_

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/float_v.pb.h>
#include <onnxruntime_cxx_api.h>

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntarray.hpp>

#include <vector>
#include <string>
#include <memory>
#include <map>

namespace harambe_gz
{

/// Gazebo system plugin that runs an ONNX RL policy directly in the physics loop.
/// Uses KDL for gravity compensation to match PhysX ImplicitActuator behavior.
class HarambePolicyPlugin :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  HarambePolicyPlugin() = default;
  ~HarambePolicyPlugin() override = default;

  void Configure(
      const gz::sim::Entity &entity,
      const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &ecm,
      gz::sim::EventManager &eventMgr) override;

  void PreUpdate(
      const gz::sim::UpdateInfo &info,
      gz::sim::EntityComponentManager &ecm) override;

private:
  // ONNX model
  std::unique_ptr<Ort::Env> ort_env_;
  std::unique_ptr<Ort::Session> ort_session_;
  Ort::MemoryInfo ort_mem_info_{Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)};

  // Joint config
  struct JointConfig {
    std::string name;
    double kp, kd;
    double effort_limit, vel_limit;
    double default_pos;
    double friction{0.0};  // Coulomb friction from URDF for compensation
    gz::sim::Entity entity{gz::sim::kNullEntity};
    int kdl_index{-1};
  };
  std::vector<JointConfig> joints_;
  size_t num_joints_ = 0;

  // IMU entity
  gz::sim::Entity imu_entity_{gz::sim::kNullEntity};
  std::string imu_link_name_;

  // State
  std::vector<float> obs_;
  std::vector<float> prev_action_;
  std::vector<float> target_pos_;       // current target (held constant between policy steps)
  std::vector<float> commands_;

  // KDL gravity compensation
  struct KDLChainData {
    KDL::Chain chain;
    std::unique_ptr<KDL::ChainDynParam> dynamics;
    std::vector<int> joint_map;  // maps KDL joint index -> plugin joint index
  };
  std::vector<KDLChainData> kdl_chains_;
  KDL::Tree kdl_tree_;
  bool kdl_ready_ = false;
  std::vector<double> gravity_torques_;  // gravity compensation per joint

  // Timing
  int decimation_ = 4;
  int step_counter_ = 0;
  double action_scale_ = 0.25;
  double clip_actions_ = 5.0;
  bool initialized_ = false;
  bool policy_active_ = false;
  int warmup_steps_ = 0;
  int policy_step_ = 0;

  // URDF string for deferred KDL init
  std::string urdf_str_;

  // Velocity command subscriber
  gz::transport::Node gz_node_;

  // Debug publisher: single topic with all policy data (enabled by <policy_debug>true</policy_debug>)
  bool policy_debug_ = false;
  gz::transport::Node::Publisher debug_pub_;
  std::vector<double> last_efforts_;
  std::vector<double> last_positions_;
  std::vector<double> last_velocities_;

  void RunPolicyInference();
  void BuildObservation(gz::sim::EntityComponentManager &ecm);
  void ApplyPDControl(gz::sim::EntityComponentManager &ecm);
  void ComputeGravityCompensation(gz::sim::EntityComponentManager &ecm);
  bool InitKDL(const std::string &urdf_string);
  void OnCmdVel(const gz::msgs::Twist &msg);
};

}  // namespace harambe_gz

#endif
