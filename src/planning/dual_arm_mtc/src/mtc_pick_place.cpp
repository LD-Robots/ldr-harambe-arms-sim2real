#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <mutex>
#include <stdexcept>
#include <std_msgs/msg/empty.hpp>
#if __has_include(<moveit/planning_scene/planning_scene.hpp>)
  #include <moveit/planning_scene/planning_scene.hpp>
  #include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#elif __has_include(<moveit/planning_scene/planning_scene.h>)
  #include <moveit/planning_scene/planning_scene.h>
  #include <moveit/planning_scene_interface/planning_scene_interface.h>
#else
  #error "MoveIt planning_scene header not found"
#endif
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/cost_terms.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

#if __has_include(<moveit/robot_model/robot_model.hpp>)
  #include <moveit/robot_model/robot_model.hpp>
#else
  #include <moveit/robot_model/robot_model.h>
#endif

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_pick_place");
namespace mtc = moveit::task_constructor;

namespace
{
constexpr std::size_t ARM_DOF = 7;

bool validateArmGroup(const moveit::core::RobotModelConstPtr& model,
                      const std::string& group_name,
                      const std::string& waist_joint)
{
  const moveit::core::JointModelGroup* arm_group = model->getJointModelGroup(group_name);
  if (!arm_group) {
    RCLCPP_ERROR(LOGGER, "Planning group '%s' does not exist", group_name.c_str());
    return false;
  }

  const std::vector<std::string>& variables = arm_group->getVariableNames();
  RCLCPP_INFO(LOGGER, "Planning group '%s' contains %zu DOF:", group_name.c_str(), variables.size());
  for (std::size_t index = 0; index < variables.size(); ++index) {
    RCLCPP_INFO(LOGGER, "  [%zu] %s", index + 1, variables[index].c_str());
  }

  const bool has_waist = std::find(variables.begin(), variables.end(), waist_joint) != variables.end();
  if (variables.size() != ARM_DOF || !has_waist) {
    RCLCPP_ERROR(LOGGER,
                 "Expected a %zu-DOF arm group containing '%s', but the loaded robot model does not match",
                 ARM_DOF, waist_joint.c_str());
    return false;
  }
  return true;
}
}  // namespace

// Collect every link WITH collision geometry in the kinematic subtree rooted at
// `root_link` (inclusive). The hand's intermediate/distal finger links are driven by
// MIMIC joints that are NOT part of the "hand" planning group, so
// getJointModelGroup("hand")->getLinkModelNamesWithCollisionGeometry() omits them.
// Those omitted links (e.g. left_thumb_distal) then collide with the table and stop
// the Cartesian approach short ("min_fraction not met"). Walking the subtree from
// left_hand_base_link catches the whole hand — palm, proximals, intermediates, distals.
static std::vector<std::string>
handCollisionLinks(const moveit::core::RobotModelConstPtr& model, const std::string& root_link)
{
  std::vector<std::string> links;
  const moveit::core::LinkModel* root = model->getLinkModel(root_link);
  if (!root) {
    RCLCPP_WARN(LOGGER, "handCollisionLinks: link '%s' not found", root_link.c_str());
    return links;
  }
  std::vector<const moveit::core::LinkModel*> stack{ root };
  while (!stack.empty()) {
    const moveit::core::LinkModel* link = stack.back();
    stack.pop_back();
    if (!link->getShapes().empty())  // has collision geometry
      links.push_back(link->getName());
    for (const moveit::core::JointModel* joint : link->getChildJointModels())
      if (const moveit::core::LinkModel* child = joint->getChildLinkModel())
        stack.push_back(child);
  }
  return links;
}

// One arm and the hand on it. The task runs the same pick-and-place twice, once per
// arm: the first carries the object to the far table, the second brings it back.
struct ArmSpec
{
  std::string arm_group;
  std::string hand_group;
  std::string hand_frame;       // IK target frame
  std::string hand_root_link;   // subtree root for collision allowances
  std::vector<std::string> arm_joints;
  double grasp_tilt{ 0.18 };
  std::vector<double> pre_grasp_offset{ 0.035, 0.045, -0.03 };
  // Extra rotation of the grasp frame, in its own axes, after the standard
  // RotY(pi) * RotX(pi - tilt).
  std::vector<double> grasp_frame_rpy{ 0.0, 0.0, 0.0 };
};

// Everything the task reads from config/mtc_task.yaml. Defaults mirror that file so
// the node still runs standalone; the YAML remains the source of truth.
struct TaskConfig
{
  // robot.*
  std::string arm_group{ "arm" };
  std::string hand_group{ "hand" };
  std::string hand_frame{ "left_tcp_link" };
  std::string hand_root_link{ "left_hand_base_link" };
  std::string planning_frame{ "urdf_base" };
  std::string waist_joint{ "waist_yaw_joint_X8" };
  std::vector<std::string> arm_joints{ "left_shoulder_pitch_joint_X6", "left_shoulder_roll_joint_X6",
                                       "left_shoulder_yaw_joint_X4",   "left_elbow_pitch_joint_X6",
                                       "left_wrist_yaw_joint_X4",      "left_wrist_roll_joint_X4" };

  // poses.*
  std::string pose_arm_home{ "home" };
  std::string pose_hand_open{ "open" };
  std::string pose_hand_pregrasp{ "03_hand_cylinder_pregrasp" };
  std::string pose_hand_grasp{ "04_hand_cylinder_grasp" };
  std::string pose_hand_release_partial{ "07_hand_cylinder_release" };
  std::string pose_hand_release_full{ "08_hand_cylinder_release" };

  // scene.* -- only the names the task needs for collision allowances; the geometry
  // itself belongs to planning_scene_publisher.py.
  std::string object_id{ "target_cylinder" };
  std::string source_table_name{ "table" };
  std::string destination_table_name{ "destination_table" };

  // task.*
  std::vector<double> place_pose{ 0.45, -0.35, 0.175 };        // where the first arm puts it
  std::vector<double> second_place_pose{ 0.45, 0.35, 0.175 };  // where the second arm puts it back
  double grasp_tilt{ 0.18 };
  std::vector<double> pre_grasp_offset{ 0.035, 0.045, -0.03 };
  double grasp_angle_delta{ M_PI / 6 };
  int grasp_max_ik_solutions{ 4 };
  double grasp_min_solution_distance{ 0.1 };
  double approach_min{ 0.0 }, approach_max{ 0.05 };
  double lift_min{ 0.005 }, lift_max{ 0.1 };
  double retreat_min{ 0.02 }, retreat_max{ 0.1 };
  int place_n_angles{ 6 };
  int place_max_ik_solutions{ 2 };
  double place_min_solution_distance{ 0.1 };
  std::vector<double> grasp_frame_rpy{ 0.0, 0.0, 0.0 };

  // The second arm, which brings the object back.
  std::string second_arm_group{ "right_arm" };
  std::string second_hand_group{ "right_hand" };
  std::string second_hand_frame{ "right_tcp_link" };
  std::string second_hand_root_link{ "right_hand_base_link" };
  std::vector<std::string> second_arm_joints{
    "right_shoulder_pitch_joint_X6", "right_shoulder_roll_joint_X6",
    "right_shoulder_yaw_joint_X4",   "right_elbow_pitch_joint_X6",
    "right_wrist_yaw_joint_X4",      "right_wrist_roll_joint_X4" };
  double second_grasp_tilt{ 0.18 };
  // Mirrored against the left hand, each in the way its own quantity requires -- see
  // the second_grasp block in config/mtc_task.yaml for why they differ.
  std::vector<double> second_pre_grasp_offset{ 0.035, 0.045, 0.03 };
  std::vector<double> second_grasp_frame_rpy{ 0.0, M_PI, 0.0 };

  double cost_waist_weight{ 15.0 };
  std::vector<double> cost_arm_weights{ 7.0, 7.0, 5.0, 3.0, 1.0, 1.0 };
  int max_solutions{ 5 };
  double goal_joint_tolerance{ 0.00001 };
  double cartesian_step_size{ 0.01 };
  double scene_wait_timeout{ 10.0 };
  std::string trigger_topic{ "/mtc_next_arm" };
};

class MTCPickPlace
{
public:
  MTCPickPlace(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  bool doTask();

  bool executesAutomatically() const
  {
    return execute_automatically_;
  }

  bool loadTaskConfig();

  // Block until planning_scene_publisher.py has published the object, or time out.
  bool waitForPlanningScene();

  // Diagnostic: log the exact link pairs colliding at the final hand-grasp pose.
  void dumpHandGraspContacts();

private:
  // One task per arm. The second is only built after the first has run, so it
  // plans from the state the robot is actually in rather than a predicted one.
  mtc::Task createTask(const std::string& name, const ArmSpec& arm,
                       const std::vector<double>& place_target);
  ArmSpec firstArm() const;
  ArmSpec secondArm() const;
  bool runArm(const std::string& name, const ArmSpec& arm,
              const std::vector<double>& place_target);

  // Block until someone publishes on config_.trigger_topic. Returns false if the node
  // is shut down while waiting.
  bool waitForTrigger(const std::string& what);

  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
  TaskConfig config_;
  mtc::Stage* hand_grasp_ptr_{ nullptr };  // final hand-grasp stage, used for failure diagnostics
  bool execute_automatically_{ false };
  bool wait_for_trigger_{ true };

  // The trigger arrives on the executor thread while doTask() sits in waitForTrigger().
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr trigger_sub_;
  std::mutex trigger_mutex_;
  std::condition_variable trigger_cv_;
  bool triggered_{ false };
};

MTCPickPlace::MTCPickPlace(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_pick_place", options) }
{
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCPickPlace::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

bool MTCPickPlace::loadTaskConfig()
{
  // Read one key from config/mtc_task.yaml, keeping the struct default when absent so
  // the node still runs without a config file. The struct defaults deliberately mirror
  // the YAML, which makes "loaded" and "fell back" indistinguishable from the values
  // alone -- so count which keys actually came from the parameter file.
  std::size_t from_params = 0;
  std::vector<std::string> missing;
  auto load = [&](const std::string& name, auto& target) {
    using T = std::decay_t<decltype(target)>;
    if (node_->has_parameter(name)) {  // auto-declared from the parameter overrides
      ++from_params;
    } else {
      missing.push_back(name);
      node_->declare_parameter<T>(name, target);
    }
    target = node_->get_parameter(name).get_value<T>();
  };

  load("robot.arm_group", config_.arm_group);
  load("robot.hand_group", config_.hand_group);
  load("robot.hand_frame", config_.hand_frame);
  load("robot.hand_root_link", config_.hand_root_link);
  load("robot.planning_frame", config_.planning_frame);
  load("robot.waist_joint", config_.waist_joint);
  load("robot.arm_joints", config_.arm_joints);

  load("poses.arm_home", config_.pose_arm_home);
  load("poses.hand_open", config_.pose_hand_open);
  load("poses.hand_pregrasp", config_.pose_hand_pregrasp);
  load("poses.hand_grasp", config_.pose_hand_grasp);
  load("poses.hand_release_partial", config_.pose_hand_release_partial);
  load("poses.hand_release_full", config_.pose_hand_release_full);

  load("scene.object.id", config_.object_id);
  load("scene.source_table.name", config_.source_table_name);
  load("scene.destination_table.name", config_.destination_table_name);

  load("task.place_pose", config_.place_pose);
  load("task.second_place_pose", config_.second_place_pose);
  load("task.grasp.frame_rpy", config_.grasp_frame_rpy);
  load("robot.second_arm.arm_group", config_.second_arm_group);
  load("robot.second_arm.hand_group", config_.second_hand_group);
  load("robot.second_arm.hand_frame", config_.second_hand_frame);
  load("robot.second_arm.hand_root_link", config_.second_hand_root_link);
  load("robot.second_arm.arm_joints", config_.second_arm_joints);
  load("task.second_grasp.tilt", config_.second_grasp_tilt);
  load("task.second_grasp.pre_grasp_offset", config_.second_pre_grasp_offset);
  load("task.second_grasp.frame_rpy", config_.second_grasp_frame_rpy);
  load("task.grasp.tilt", config_.grasp_tilt);
  load("task.grasp.pre_grasp_offset", config_.pre_grasp_offset);
  load("task.grasp.angle_delta", config_.grasp_angle_delta);
  load("task.grasp.max_ik_solutions", config_.grasp_max_ik_solutions);
  load("task.grasp.min_solution_distance", config_.grasp_min_solution_distance);
  load("task.approach.min_distance", config_.approach_min);
  load("task.approach.max_distance", config_.approach_max);
  load("task.lift.min_distance", config_.lift_min);
  load("task.lift.max_distance", config_.lift_max);
  load("task.retreat.min_distance", config_.retreat_min);
  load("task.retreat.max_distance", config_.retreat_max);
  load("task.place.n_angles", config_.place_n_angles);
  load("task.place.max_ik_solutions", config_.place_max_ik_solutions);
  load("task.place.min_solution_distance", config_.place_min_solution_distance);
  load("task.cost.waist_weight", config_.cost_waist_weight);
  load("task.cost.arm_weights", config_.cost_arm_weights);
  load("task.planning.max_solutions", config_.max_solutions);
  load("task.planning.goal_joint_tolerance", config_.goal_joint_tolerance);
  load("task.planning.cartesian_step_size", config_.cartesian_step_size);
  load("task.planning.scene_wait_timeout", config_.scene_wait_timeout);
  load("task.trigger_topic", config_.trigger_topic);

  if (!node_->has_parameter("execute"))
    node_->declare_parameter("execute", false);
  execute_automatically_ = node_->get_parameter("execute").as_bool();

  if (!node_->has_parameter("wait_for_trigger"))
    node_->declare_parameter("wait_for_trigger", true);
  wait_for_trigger_ = node_->get_parameter("wait_for_trigger").as_bool();

  // Default QoS on purpose: `ros2 topic pub` is volatile, and a transient-local
  // subscription would silently never match it. The subscriber exists from here on, so
  // a trigger sent while arm 1 is still planning is queued and sets the flag early
  // rather than being lost.
  trigger_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
      config_.trigger_topic, 10,
      [this](const std_msgs::msg::Empty::SharedPtr) {
        {
          std::lock_guard<std::mutex> lock(trigger_mutex_);
          triggered_ = true;
        }
        trigger_cv_.notify_all();
      });

  // The cost map pairs these element-wise; a mismatch would silently drop weights.
  if (config_.arm_joints.size() != config_.cost_arm_weights.size()) {
    RCLCPP_ERROR(LOGGER,
                 "robot.arm_joints has %zu entries but task.cost.arm_weights has %zu; they must match",
                 config_.arm_joints.size(), config_.cost_arm_weights.size());
    return false;
  }
  if (config_.place_pose.size() != 3 || config_.pre_grasp_offset.size() != 3) {
    RCLCPP_ERROR(LOGGER, "task.place_pose and task.grasp.pre_grasp_offset must each have 3 entries");
    return false;
  }

  const std::size_t total = from_params + missing.size();
  if (from_params == 0) {
    RCLCPP_WARN(LOGGER,
                "No task parameters were supplied -- all %zu keys fell back to built-in defaults. "
                "Is config/mtc_task.yaml being passed to this node?",
                total);
  } else {
    RCLCPP_INFO(LOGGER, "Task config: %zu/%zu keys from parameters", from_params, total);
    for (const std::string& name : missing)
      RCLCPP_WARN(LOGGER, "  '%s' not in the parameter file, using built-in default", name.c_str());
  }

  RCLCPP_INFO(LOGGER, "Task config: object='%s', place=(%.3f, %.3f, %.3f) in '%s'",
              config_.object_id.c_str(), config_.place_pose[0], config_.place_pose[1],
              config_.place_pose[2], config_.planning_frame.c_str());
  RCLCPP_INFO(LOGGER, "Grasp: tilt=%.3f rad, angle_delta=%.4f rad, max_ik=%d; planning max_solutions=%d",
              config_.grasp_tilt, config_.grasp_angle_delta, config_.grasp_max_ik_solutions,
              config_.max_solutions);
  return true;
}

// The collision scene is owned by planning_scene_publisher.py, which publishes on the
// /collision_object topic. That is asynchronous, so wait for the object to actually land
// in the monitored scene rather than racing it.
bool MTCPickPlace::waitForPlanningScene()
{
  moveit::planning_interface::PlanningSceneInterface psi;
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration<double>(config_.scene_wait_timeout);

  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    const std::vector<std::string> known = psi.getKnownObjectNames();
    if (std::find(known.begin(), known.end(), config_.object_id) != known.end()) {
      RCLCPP_INFO(LOGGER, "Planning scene contains '%s' (%zu objects total)",
                  config_.object_id.c_str(), known.size());
      return true;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(200));
  }

  RCLCPP_ERROR(LOGGER,
               "Timed out after %.1f s waiting for '%s' in the planning scene. "
               "Is planning_scene_publisher running?",
               config_.scene_wait_timeout, config_.object_id.c_str());
  return false;
}

// Diagnostic: take the REAL state at which the final hand-grasp stage failed (its failure carries
// the true arm configuration and every MTC allowance applied so far), drive the hand to the named
// cylinder-grasp pose, and list the contacts that remain. Using the stage's failure rather than a
// synthetic proxy avoids reporting collisions caused by unrelated default robot poses.
void MTCPickPlace::dumpHandGraspContacts()
{
  if (!hand_grasp_ptr_ || hand_grasp_ptr_->failures().empty()) {
    RCLCPP_WARN(LOGGER, "dumpHandGraspContacts: no 'grasp cylinder' failures to inspect");
    return;
  }

  collision_detection::CollisionRequest req;
  req.contacts = true;
  req.max_contacts = 200;
  req.max_contacts_per_pair = 1;

  int idx = 0;
  for (const auto& failure : hand_grasp_ptr_->failures()) {
    const moveit::task_constructor::InterfaceState* start = failure->start();
    if (!start || !start->scene())
      continue;
    // Editable copy of the real pre-grasp scene, with all upstream MTC allowances intact.
    planning_scene::PlanningScenePtr scene = start->scene()->diff();
    moveit::core::RobotState& state = scene->getCurrentStateNonConst();
    if (const auto* hand = state.getJointModelGroup(config_.hand_group))
      state.setToDefaultValues(hand, config_.pose_hand_grasp);
    state.update();

    collision_detection::CollisionResult res;
    scene->checkCollision(req, res, state);  // uses the scene's (MTC) ACM => only real blockers remain

    RCLCPP_WARN(LOGGER, "=== grasp cylinder failure #%d: %zu BLOCKING contact(s) ===",
                idx++, res.contacts.size());
    for (const auto& entry : res.contacts)
      RCLCPP_WARN(LOGGER, "  BLOCKER: %s  <->  %s",
                  entry.first.first.c_str(), entry.first.second.c_str());
  }
}

ArmSpec MTCPickPlace::firstArm() const
{
  return { config_.arm_group, config_.hand_group, config_.hand_frame, config_.hand_root_link,
           config_.arm_joints, config_.grasp_tilt, config_.pre_grasp_offset,
           config_.grasp_frame_rpy };
}

ArmSpec MTCPickPlace::secondArm() const
{
  return { config_.second_arm_group, config_.second_hand_group, config_.second_hand_frame,
           config_.second_hand_root_link, config_.second_arm_joints, config_.second_grasp_tilt,
           config_.second_pre_grasp_offset, config_.second_grasp_frame_rpy };
}

// Plan one arm's pick-and-place and, if asked, run it. Returns false only on a real
// failure; a successful plan left for manual execution also returns true.
bool MTCPickPlace::runArm(const std::string& name, const ArmSpec& arm,
                          const std::vector<double>& place_target)
{
  try
  {
    task_ = createTask(name, arm, place_target);
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, name << ": initialization failed: " << e);
    return false;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(LOGGER, "%s: task creation failed: %s", name.c_str(), e.what());
    return false;
  }

  RCLCPP_INFO(LOGGER, "%s: planning (max %d solutions)...", name.c_str(), config_.max_solutions);
  if (!task_.plan(static_cast<std::size_t>(config_.max_solutions)))
  {
    RCLCPP_ERROR(LOGGER, "%s: planning failed", name.c_str());
    dumpHandGraspContacts();  // name the exact links blocking the final hand-grasp pose
    return false;
  }

  task_.introspection().publishSolution(*task_.solutions().front());
  RCLCPP_INFO(LOGGER, "%s: %zu solutions found.", name.c_str(), task_.solutions().size());

  if (!execute_automatically_)
    return true;

  RCLCPP_INFO(LOGGER, "%s: executing...", name.c_str());
  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, name << ": execution failed with error code: " << result.val);
    return false;
  }
  RCLCPP_INFO(LOGGER, "%s: executed.", name.c_str());
  return true;
}

bool MTCPickPlace::waitForTrigger(const std::string& what)
{
  std::unique_lock<std::mutex> lock(trigger_mutex_);
  RCLCPP_INFO(LOGGER, "Waiting for the go-ahead before %s.", what.c_str());
  RCLCPP_INFO(LOGGER, "  ros2 topic pub --once %s std_msgs/msg/Empty {}",
              config_.trigger_topic.c_str());

  while (!triggered_)
  {
    if (!rclcpp::ok())
    {
      RCLCPP_WARN(LOGGER, "Shut down while waiting on %s.", config_.trigger_topic.c_str());
      return false;
    }
    // Timed wait so Ctrl+C is noticed even if the trigger never arrives.
    trigger_cv_.wait_for(lock, std::chrono::milliseconds(200));
  }

  triggered_ = false;  // consumed: a later pause needs its own trigger
  RCLCPP_INFO(LOGGER, "Triggered.");
  return true;
}

bool MTCPickPlace::doTask()
{
  // Arm 1 carries the object to the far table and goes home.
  if (!runArm("arm 1 (" + config_.arm_group + ")", firstArm(), config_.place_pose))
    return false;

  // Arm 2's task is built only now, against the scene as it actually ended up rather
  // predicted one -- which is the whole reason the two arms are separate tasks. That
  // pause below load-bearing when nothing executes arm 1's task for us: the object
  // is still on the near table and the second arm would have nothing to pick.
  if (wait_for_trigger_)
  {
    if (!execute_automatically_)
    {
      RCLCPP_INFO(LOGGER, "Arm 1 is planned but not executed (execute:=false).");
      RCLCPP_INFO(LOGGER, "Run it from RViz under 'Motion Planning Tasks' first, otherwise");
      RCLCPP_INFO(LOGGER, "arm 2 will plan against an object that never moved.");
    }
    if (!waitForTrigger("planning arm 2"))
      return false;
  }

  return runArm("arm 2 (" + config_.second_arm_group + ")", secondArm(), config_.second_place_pose);
}

mtc::Task MTCPickPlace::createTask(const std::string& name, const ArmSpec& arm,
                                   const std::vector<double>& place_target)
{
  mtc::Task task;
  task.stages()->setName(name);
  task.loadRobotModel(node_);

  // Everything below comes from config/mtc_task.yaml -- see loadTaskConfig().
  const std::string& arm_group_name = arm.arm_group;
  const std::string& hand_group_name = arm.hand_group;
  const std::string& hand_frame = arm.hand_frame;

  if (!validateArmGroup(task.getRobotModel(), arm_group_name, config_.waist_joint)) {
    throw std::runtime_error("The MTC arm group is not configured as the expected 7-DOF waist-arm chain");
  }

  // Weighted PathLength map shared by both Connect stages: waist plus one entry per arm joint.
  std::map<std::string, double> connect_cost_weights{ { config_.waist_joint, config_.cost_waist_weight } };
  for (std::size_t i = 0; i < config_.arm_joints.size(); ++i)
    connect_cost_weights.emplace(config_.arm_joints[i], config_.cost_arm_weights[i]);

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

  // Create planners
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  sampling_planner->setProperty("goal_joint_tolerance", config_.goal_joint_tolerance);

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(config_.cartesian_step_size);

  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  // Forward current_state to later stages
  mtc::Stage* current_state_ptr = nullptr;

  // Weighted PathLength for an arm's Connect stages. The waist is shared by both
  // chains, so it carries the same weight whichever arm is moving.
  auto connect_cost_for = [&](const ArmSpec& arm) {
    std::map<std::string, double> weights{ { config_.waist_joint, config_.cost_waist_weight } };
    for (std::size_t i = 0; i < arm.arm_joints.size() && i < config_.cost_arm_weights.size(); ++i)
      weights.emplace(arm.arm_joints[i], config_.cost_arm_weights[i]);
    return std::make_shared<mtc::cost::PathLength>(weights);
  };

  // Where on the hand the object ends up, and how that frame is oriented.
  // RotY(pi)*RotX(pi - tilt) tilts the approach off straight down, which a pure top
  // grasp cannot reach: the arm gets to ~81 deg where 90 would be needed.
  //
  // grasp_frame_rpy is applied FIRST, to the hand frame itself, and describes how that
  // hand's TCP sits relative to the left one. The right hand really is a mirror image:
  // both TCPs have +Y toward the fingertips, but the thumb sits at +Z on the left and
  // -Z on the right. Feeding both through one grasp formula therefore rolled the right
  // hand 180 deg about its approach axis -- it reached the object and closed from the
  // wrong side. [0, pi, 0] undoes that: a half turn about Y leaves the approach axis
  // alone and flips only X and Z, which is exactly the mirror.
  //
  // It rotates the frame ONLY. pre_grasp_offset is a point in the palm and is mirrored
  // in the config instead, because the two are not the same mirror: the hands are
  // reflections (measured: anatomy maps exactly (x,y,z) -> (x,y,-z) in hand-base
  // coords), while Ry(pi) is a rotation and flips X as well. Running the offset
  // through it puts the right hand's grasp point 70 mm off, on the wrong side of
  // the palm.
  auto grasp_frame_for = [](const ArmSpec& arm) {
    const Eigen::Matrix3d mirror =
        (Eigen::AngleAxisd(arm.grasp_frame_rpy[0], Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxisd(arm.grasp_frame_rpy[1], Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(arm.grasp_frame_rpy[2], Eigen::Vector3d::UnitZ())).toRotationMatrix();
    Eigen::Isometry3d frame = Eigen::Isometry3d::Identity();
    frame.linear() = mirror *
                     (Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(M_PI - arm.grasp_tilt, Eigen::Vector3d::UnitX()))
                         .toRotationMatrix();
    frame.translation().x() = arm.pre_grasp_offset[0];
    frame.translation().y() = arm.pre_grasp_offset[1];
    frame.translation().z() = arm.pre_grasp_offset[2];
    return frame;
  };

  // Where to put the object an arm is ALREADY holding: sweep yaw around the
  // (symmetric) cylinder axis and IK every candidate, so Alternatives can try them all.
  auto held_object_pose_alternatives =
    [&](const std::string& label, const ArmSpec& arm, mtc::Stage* monitored,
        const std::vector<double>& target, const std::string& target_frame, int n_angles,
        int max_ik_solutions, double min_solution_distance) {
      auto alternatives = std::make_unique<mtc::Alternatives>(label + " pose alternatives");
      // exposeTo DECLARES eef/group/ik_frame on the container; they are then set to
      // THIS arm rather than inherited, because the second arm differs from the task.
      task.properties().exposeTo(alternatives->properties(), { "eef", "group", "ik_frame" });
      alternatives->setProperty("group", arm.arm_group);
      alternatives->setProperty("eef", arm.hand_group);
      alternatives->setProperty("ik_frame", arm.hand_frame);

      for (int i = 0; i < n_angles; ++i) {
        const double yaw = i * (2.0 * M_PI / n_angles);

        auto generator = std::make_unique<mtc::stages::GeneratePlacePose>("generate " + label + " pose");
        generator->properties().configureInitFrom(mtc::Stage::PARENT);
        generator->properties().set("marker_ns", label + "_pose");
        generator->setObject(config_.object_id);

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = target_frame;
        pose.pose.position.x = target[0];
        pose.pose.position.y = target[1];
        pose.pose.position.z = target[2];
        pose.pose.orientation.z = std::sin(yaw / 2.0);  // pure yaw quaternion
        pose.pose.orientation.w = std::cos(yaw / 2.0);
        generator->setPose(pose);
        generator->setMonitoredStage(monitored);  // scene where the object is held

        auto wrapper = std::make_unique<mtc::stages::ComputeIK>(label + " pose IK", std::move(generator));
        wrapper->setMaxIKSolutions(max_ik_solutions);
        wrapper->setMinSolutionDistance(min_solution_distance);
        // Do NOT ignore collisions: these IK results are Connect goals, and an
        // in-collision goal is rejected by OMPL as GOAL_STATE_INVALID.
        wrapper->setIgnoreCollisions(false);
        wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
        wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
        // Same frame as the grasp: the object is held in exactly that relationship,
        // so any pose for it must mirror the grasp hand pose or there is no IK.
        wrapper->setIKFrame(grasp_frame_for(arm), arm.hand_frame);

        alternatives->insert(std::move(wrapper));
      }
      return alternatives;
    };

  // ========== STAGE 1: Current State ==========
  {
    auto current_state = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = current_state.get();
    task.add(std::move(current_state));
  }

  // ========== STAGE 1b: Allow object<->table collisions for the whole task ==========
  // The object rests on one table and ends up on the other, so both pairs are
  // expected contacts. Hand<->table is deliberately NOT allowed anywhere.
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (object,tables)");
    stage->allowCollisions(config_.object_id, config_.source_table_name, true);
    stage->allowCollisions(config_.object_id, config_.destination_table_name, true);
    task.add(std::move(stage));
  }

  // ========== STAGE 2: This arm home, its hand open ==========
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("move to home", sampling_planner);
    stage->setGroup(arm.arm_group);
    stage->setGoal(config_.pose_arm_home);
    task.add(std::move(stage));
  }
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
    stage->setGroup(arm.hand_group);
    stage->setGoal(config_.pose_hand_open);
    task.add(std::move(stage));
  }

  // Pick the object up with one arm. `monitored` is the stage whose scene shows where
  // the object currently sits; `attach_ptr` comes back pointing at the attach stage,
  // which the matching place sequence has to monitor.
  auto add_pick = [&](const ArmSpec& arm, mtc::Stage* monitored, mtc::Stage*& attach_ptr) {
    const std::string label = "grasp";
    {
      auto connect = std::make_unique<mtc::stages::Connect>(
        "move to " + label + " grasp",
        mtc::stages::Connect::GroupPlannerVector{ { arm.arm_group, sampling_planner } });
      connect->setCostTerm(connect_cost_for(arm));
      task.add(std::move(connect));
    }

    auto grasp = std::make_unique<mtc::SerialContainer>(label + " grasp");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->setProperty("group", arm.arm_group);
    grasp->setProperty("eef", arm.hand_group);
    grasp->setProperty("ik_frame", arm.hand_frame);

    const auto hand_links = handCollisionLinks(task.getRobotModel(), arm.hand_root_link);

    // Propagates BACKWARD, so it never reaches the stages after the generator.
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
        "allow collision (" + label + " hand,object)");
      stage->allowCollisions(config_.object_id, hand_links, true);
      grasp->insert(std::move(stage));
    }
    {
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate " + label + " grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", label + "_grasp_pose");
      stage->setPreGraspPose(config_.pose_hand_open);
      stage->setObject(config_.object_id);
      stage->setAngleDelta(config_.grasp_angle_delta);
      stage->setMonitoredStage(monitored);

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>(label + " grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(config_.grasp_max_ik_solutions);
      wrapper->setMinSolutionDistance(config_.grasp_min_solution_distance);
      // A dexterous hand always contacts the object at the grasp pose, and the
      // allowance above propagates backward so it never reaches this check.
      wrapper->setIgnoreCollisions(true);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      wrapper->setIKFrame(grasp_frame_for(arm), arm.hand_frame);
      grasp->insert(std::move(wrapper));
    }
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>(label + " approach", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(config_.approach_min, config_.approach_max);
      stage->setIKFrame(arm.hand_frame);
      stage->properties().set("marker_ns", label + "_approach");
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = arm.hand_frame;
      vec.vector.x = 1.0;   // +X of the hand frame points at the object
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }
    // Forward-propagating twin of the allowance above: grasping drives the fingers
    // into the object and into each other.
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
        "allow hand collisions (" + label + ")");
      stage->allowCollisions(config_.object_id, hand_links, true);
      stage->allowCollisions(hand_links, hand_links, true);
      grasp->insert(std::move(stage));
    }
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("prepare " + label + " hand", interpolation_planner);
      stage->setGroup(arm.hand_group);
      stage->setGoal(config_.pose_hand_pregrasp);
      grasp->insert(std::move(stage));
    }
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>(label + " close on cylinder", interpolation_planner);
      stage->setGroup(arm.hand_group);
      stage->setGoal(config_.pose_hand_grasp);
      hand_grasp_ptr_ = stage.get();   // failure diagnostics
      grasp->insert(std::move(stage));
    }
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach to " + label);
      stage->attachObject(config_.object_id, arm.hand_frame);
      grasp->insert(std::move(stage));
    }
    // The world-object allowance does not carry over once the object is attached.
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
        "allow attached object<->tables (" + label + ")");
      stage->allowCollisions(config_.object_id, config_.source_table_name, true);
      stage->allowCollisions(config_.object_id, config_.destination_table_name, true);
      attach_ptr = stage.get();   // the place generator monitors this
      grasp->insert(std::move(stage));
    }
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>(label + " lift", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(config_.lift_min, config_.lift_max);
      stage->setIKFrame(arm.hand_frame);
      stage->properties().set("marker_ns", label + "_lift");
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = config_.planning_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }
    task.add(std::move(grasp));
  };

  // Put the object down at `target`, then back the hand out and go home.
  auto add_place = [&](const ArmSpec& arm, const std::vector<double>& target,
                       mtc::Stage* monitored, mtc::Stage*& detach_ptr) {
    const std::string label = "place";
    {
      auto connect = std::make_unique<mtc::stages::Connect>(
        "move to " + label + " place",
        mtc::stages::Connect::GroupPlannerVector{ { arm.arm_group, sampling_planner } });
      connect->setCostTerm(connect_cost_for(arm));
      task.add(std::move(connect));
    }

    auto place = std::make_unique<mtc::SerialContainer>(label + " place");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    place->setProperty("group", arm.arm_group);
    place->setProperty("eef", arm.hand_group);
    place->setProperty("ik_frame", arm.hand_frame);

    const auto hand_links = handCollisionLinks(task.getRobotModel(), arm.hand_root_link);

    place->insert(held_object_pose_alternatives(
      label + " place", arm, monitored, target, config_.planning_frame,
      config_.place_n_angles, config_.place_max_ik_solutions,
      config_.place_min_solution_distance));

    // The place IK spawns fresh states that do not carry the task-level allowance.
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
        "allow collision (object,tables) " + label + " place");
      stage->allowCollisions(config_.object_id, config_.source_table_name, true);
      stage->allowCollisions(config_.object_id, config_.destination_table_name, true);
      place->insert(std::move(stage));
    }
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("prepare " + label + " release", interpolation_planner);
      stage->setGroup(arm.hand_group);
      stage->setGoal(config_.pose_hand_release_partial);
      place->insert(std::move(stage));
    }
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>(label + " release cylinder", interpolation_planner);
      stage->setGroup(arm.hand_group);
      stage->setGoal(config_.pose_hand_release_full);
      place->insert(std::move(stage));
    }
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
        "forbid collision (" + label + " hand,object)");
      stage->allowCollisions(config_.object_id, hand_links, false);
      place->insert(std::move(stage));
    }
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach from " + label);
      stage->detachObject(config_.object_id, arm.hand_frame);
      detach_ptr = stage.get();   // the next arm's grasp generator monitors this
      place->insert(std::move(stage));
    }
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>(label + " retreat", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(config_.retreat_min, config_.retreat_max);
      stage->setIKFrame(arm.hand_frame);
      stage->properties().set("marker_ns", label + "_retreat");
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = config_.planning_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }
    task.add(std::move(place));

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>(label + " return home", sampling_planner);
      stage->setGroup(arm.arm_group);
      stage->setGoal(config_.pose_arm_home);
      task.add(std::move(stage));
    }
  };

  // Pick the object up wherever it currently is, put it down at place_target, go home.
  mtc::Stage* attach_ptr = nullptr;
  mtc::Stage* detach_ptr = nullptr;
  add_pick(arm, current_state_ptr, attach_ptr);
  add_place(arm, place_target, attach_ptr, detach_ptr);

  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_node = std::make_shared<MTCPickPlace>(options);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(mtc_node->getNodeBaseInterface());

  // Spin in background for service calls
  auto spin_thread = std::make_unique<std::thread>([&executor]() {
    executor.spin();
  });

  // Wait for system initialization
  RCLCPP_INFO(LOGGER, "Waiting for system initialization (3 seconds)...");
  rclcpp::sleep_for(std::chrono::seconds(3));

  auto fail = [&executor, &spin_thread](const char* message) {
    RCLCPP_ERROR(LOGGER, "%s", message);
    executor.cancel();
    spin_thread->join();
    rclcpp::shutdown();
    return 1;
  };

  // Load the task configuration (config/mtc_task.yaml) from parameters
  if (!mtc_node->loadTaskConfig())
    return fail("Failed to load the task configuration");

  // The collision scene is published by planning_scene_publisher.py, not by this node
  if (!mtc_node->waitForPlanningScene())
    return fail("Planning scene is not ready");

  bool success = mtc_node->doTask();

  if (success && !mtc_node->executesAutomatically()) {
    RCLCPP_INFO(LOGGER, "Keeping the MTC node alive for manual execution. Press Ctrl+C to stop.");
    spin_thread->join();
    rclcpp::shutdown();
    return 0;
  }

  // Task complete - shutdown cleanly
  RCLCPP_INFO(LOGGER, "Task complete, shutting down...");
  executor.cancel();
  spin_thread->join();
  rclcpp::shutdown();
  return success ? 0 : 1;
}
