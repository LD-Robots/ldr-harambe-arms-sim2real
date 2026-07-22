#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>
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

  // task.*
  std::vector<double> place_pose{ 0.35, 0.46, 0.175 };
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
  double cost_waist_weight{ 15.0 };
  std::vector<double> cost_arm_weights{ 7.0, 7.0, 5.0, 3.0, 1.0, 1.0 };
  int max_solutions{ 5 };
  double goal_joint_tolerance{ 0.00001 };
  double cartesian_step_size{ 0.01 };
  double scene_wait_timeout{ 10.0 };
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
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
  TaskConfig config_;
  mtc::Stage* hand_grasp_ptr_{ nullptr };  // final hand-grasp stage, used for failure diagnostics
  bool execute_automatically_{ false };
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

  load("task.place_pose", config_.place_pose);
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

  if (!node_->has_parameter("execute"))
    node_->declare_parameter("execute", false);
  execute_automatically_ = node_->get_parameter("execute").as_bool();

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

bool MTCPickPlace::doTask()
{
  try
  {
    task_ = createTask();
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task initialization failed: " << e);
    return false;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(LOGGER, "Task creation failed: %s", e.what());
    return false;
  }

  RCLCPP_INFO(LOGGER, "Starting task planning (max %d solutions)...", config_.max_solutions);
  if (!task_.plan(static_cast<std::size_t>(config_.max_solutions)))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    dumpHandGraspContacts();  // name the exact links blocking the final hand-grasp pose
    return false;
  }

  task_.introspection().publishSolution(*task_.solutions().front());
  RCLCPP_INFO(LOGGER, "Task planning succeeded! %zu solutions found.", task_.solutions().size());

  if (!execute_automatically_)
  {
    RCLCPP_INFO(LOGGER, "Automatic execution is disabled.");
    RCLCPP_INFO(LOGGER,
                "Select a solution in RViz under 'Motion Planning Tasks' and execute it manually.");
    return true;
  }

  // Execute the task
  RCLCPP_INFO(LOGGER, "Attempting to execute the task...");
  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed with error code: " << result.val);
    return false;
  }

  RCLCPP_INFO(LOGGER, "Task executed successfully!");
  return true;
}

mtc::Task MTCPickPlace::createTask()
{
  mtc::Task task;
  task.stages()->setName("Pick and Place Cylinder");
  task.loadRobotModel(node_);

  // Everything below comes from config/mtc_task.yaml -- see loadTaskConfig().
  const std::string& arm_group_name = config_.arm_group;
  const std::string& hand_group_name = config_.hand_group;
  const std::string& hand_frame = config_.hand_frame;

  if (!validateArmGroup(task.getRobotModel(), arm_group_name, config_.waist_joint)) {
    throw std::runtime_error("The MTC arm group is not configured as the expected 7-DOF waist-arm chain");
  }

  // Pre-grasp offset: IK-frame translation from the hand frame. x must exceed finger
  // length (~0.05) so the OPEN hand sits in front of the object, not inside it.
  const double pre_grasp_offset_x = config_.pre_grasp_offset[0];
  const double pre_grasp_offset_y = config_.pre_grasp_offset[1];
  const double pre_grasp_offset_z = config_.pre_grasp_offset[2];

  // Approach: from pre-grasp toward the object. Final grasp = offset - approach distance.
  const double approach_min_dist = config_.approach_min;
  const double approach_max_dist = config_.approach_max;

  const double grasp_tilt = config_.grasp_tilt;  // off straight-down; shared by grasp + place

  const double lift_min_dist = config_.lift_min;
  const double lift_max_dist = config_.lift_max;
  const double retreat_min_dist = config_.retreat_min;
  const double retreat_max_dist = config_.retreat_max;

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
  mtc::Stage* attach_object_ptr = nullptr;  // For GeneratePlacePose to monitor

  // ========== STAGE 1: Current State ==========
  {
    auto current_state = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = current_state.get();
    task.add(std::move(current_state));
  }

  // ========== STAGE 1b: Allow object<->table collision for the whole task ==========
  // The object starts on the table and is placed back on it, so this contact is expected.
  // Allowing it persistently keeps the "move to place" goal state (object on table) valid;
  // otherwise OMPL rejects it as GOAL_STATE_INVALID.
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (object,table)");
    stage->allowCollisions(config_.object_id, config_.source_table_name, true);
    // NOTE: hand<->table is intentionally NOT allowed task-wide. A persistent hand<->table allowance
    // made the palm/fingers clip through the table during approach/lift/transport. The hand must
    // respect the table everywhere; only object<->table (the cylinder resting on it) is expected.
    task.add(std::move(stage));
  }

  // ========== STAGE 2: Move to Home Position ==========
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("move to home", sampling_planner);
    stage->setGroup(arm_group_name);
    stage->setGoal(config_.pose_arm_home);
    task.add(std::move(stage));
  }

  // ========== STAGE 3: Open Gripper ==========
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
    stage->setGroup(hand_group_name);
    stage->setGoal(config_.pose_hand_open);
    task.add(std::move(stage));
  }

  // ========== STAGE 4: Move to Ready Pose (disabled) ==========
  // {
  //   auto stage = std::make_unique<mtc::stages::MoveTo>("move to ready", sampling_planner);
  //   stage->setGroup(arm_group_name);
  //   stage->setGoal("ready");
  //   task.add(std::move(stage));
  // }

  // ========== STAGE 5: Connect to Grasp Pose ==========
  // Use Connect stage to bridge from current state to the grasp pose generator
  {
    auto connect = std::make_unique<mtc::stages::Connect>(
      "move to grasp",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
    connect->properties().configureInitFrom(mtc::Stage::PARENT);
    connect->setCostTerm(std::make_shared<mtc::cost::PathLength>(connect_cost_weights));
    task.add(std::move(connect));
  }

  // ========== STAGE 6: Grasp (SerialContainer with Generator) ==========
  {
    auto grasp = std::make_unique<mtc::SerialContainer>("grasp");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    // 6.1: Allow collision FIRST (needed for grasp pose IK to succeed)
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      // Whole hand subtree (palm + every finger link, incl. mimic-driven distals) vs the object.
      const auto hand_links = handCollisionLinks(task.getRobotModel(), config_.hand_root_link);
      stage->allowCollisions(config_.object_id, hand_links, true);
      grasp->insert(std::move(stage));
    }

    // 6.2: Generate grasp poses (GENERATOR)
    {
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose(config_.pose_hand_open);
      stage->setObject(config_.object_id);
      stage->setAngleDelta(config_.grasp_angle_delta);  // sweep around the cylinder axis
      stage->setMonitoredStage(current_state_ptr);

      // Wrap with ComputeIK
      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(config_.grasp_max_ik_solutions);
      wrapper->setMinSolutionDistance(config_.grasp_min_solution_distance);
      // A dexterous hand always contacts the object at the grasp pose, and the upstream
      // "allow collision" stage does NOT reach the grasp-IK check (generator uses the
      // current_state scene). Skip collision checking for the grasp IK itself; the approach /
      // hand-pregrasp / hand-grasp / attach stages downstream run with hand-object collisions allowed.
      wrapper->setIgnoreCollisions(true);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });

      // Keep all 7 DOF available, but prefer grasp IK solutions with the waist close to zero.
      // This is a soft cost rather than a constraint: the waist can still move when required.
      wrapper->setCostTerm(std::make_shared<mtc::cost::DistanceToReference>(
          std::map<std::string, double>{ { config_.waist_joint, 0.0 } },
          mtc::TrajectoryCostTerm::Mode::START_INTERFACE,
          std::map<std::string, double>{ { config_.waist_joint, config_.cost_waist_weight } }));

      // Pre-grasp frame transform: offset + orientation of the IK frame.
      // GenerateGraspPose orients the IK frame with Z along the object axis (cylinder = vertical)
      // and sweeps around it. With no rotation the hand approaches HORIZONTALLY (side grasp),
      // which has no IK at this low object pose. RotX(+90deg) makes the TCP +Y (approach) point
      // along the object's -Z (downward) => a TOP-DOWN grasp, which the arm can reach.
      Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
      // grasp_frame_transform = RotY(pi)*RotX(alpha). alpha=pi => straight-down top grasp, but that
      // sits at the kinematic edge (arm reaches ~81deg, grasp needs 90deg) -> no IK for all poses.
      // Tilt off vertical (alpha = pi - tilt) so the approach is down-forward, inside the reachable set.
      grasp_frame_transform.linear() =
        (Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(M_PI - grasp_tilt, Eigen::Vector3d::UnitX())).toRotationMatrix();
      grasp_frame_transform.translation().x() = pre_grasp_offset_x;
      grasp_frame_transform.translation().y() = pre_grasp_offset_y;
      grasp_frame_transform.translation().z() = pre_grasp_offset_z;
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);

      grasp->insert(std::move(wrapper));
    }

    // 6.3: Approach object (move gripper toward object along X-axis of hand frame)
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(approach_min_dist, approach_max_dist);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "approach");

      // Move along Y-axis of hand_frame (toward the object)
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.x = 1.0;  // Positive X = toward object (based on your gripper orientation)
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    // 6.3b: Allow hand<->object AND hand self-collision for the grasp — in a FORWARD-propagating
    //        stage. Stage 6.1 ("allow collision (hand,object)") sits BEFORE the grasp-pose-IK
    //        generator, so it only propagates BACKWARD and never reaches the hand-grasp stages.
    //        Grasping drives the fingers into the cylinder and potentially into each other; both
    //        collision classes must be allowed here, after the generator.
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow hand collisions (grasp)");
      const auto hand_links = handCollisionLinks(task.getRobotModel(), config_.hand_root_link);
      stage->allowCollisions(config_.object_id, hand_links, true);  // fingers grip INTO the cylinder
      stage->allowCollisions(hand_links, hand_links, true);         // fingers touch each other
      grasp->insert(std::move(stage));
    }

    // 6.4a: Prepare the hand for a multi-directional cylinder grasp. This SRDF pose rotates the
    //       thumb around the cylinder while keeping the other fingers open.
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("prepare hand for cylinder grasp", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal(config_.pose_hand_pregrasp);
      grasp->insert(std::move(stage));
    }

    // 6.4b: Close the prepared hand around the cylinder using the dedicated SRDF grasp pose.
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("grasp cylinder", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal(config_.pose_hand_grasp);
      hand_grasp_ptr_ = stage.get();  // for failure diagnostics
      grasp->insert(std::move(stage));
    }

    // 6.5: Attach the object after the dedicated hand-grasp pose has completed.
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject(config_.object_id, hand_frame);
      grasp->insert(std::move(stage));
    }

    // 6.5b: Allow the now-ATTACHED object to rest on the table. The world-object allowance (stage 1b)
    //        does NOT carry over once the object is attached to the robot, so re-allow it here.
    //        GeneratePlacePose monitors THIS stage, so the place IK sees that the placed cylinder may
    //        touch the table. hand<->table is deliberately NOT allowed — the hand must clear the table.
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow attached object<->table");
      stage->allowCollisions(config_.object_id, config_.source_table_name, true);
      attach_object_ptr = stage.get();  // place IK monitors this (attached object<->table allowed)
      grasp->insert(std::move(stage));
    }

    // 6.6: Lift object (move up in base_link Z)
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(lift_min_dist, lift_max_dist);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift");

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = config_.planning_frame;
      vec.vector.z = 1.0;  // Lift upward
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    task.add(std::move(grasp));
  }

  // ========== STAGE 7: Connect to Place Position ==========
  {
    auto connect = std::make_unique<mtc::stages::Connect>(
      "move to place",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
    connect->properties().configureInitFrom(mtc::Stage::PARENT);
    connect->setCostTerm(std::make_shared<mtc::cost::PathLength>(connect_cost_weights));
    task.add(std::move(connect));
  }

  // ========== STAGE 8: Place (SerialContainer) ==========
  {
    auto place = std::make_unique<mtc::SerialContainer>("place");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    // 8.1: Generate place pose — sweep yaw around the (symmetric) cylinder axis, like the grasp
    //      sweep at pick. Each yaw candidate gets its own IK; Alternatives tries all of them.
    {
      auto alternatives = std::make_unique<mtc::Alternatives>("place pose alternatives");
      // Pass eef/group/ik_frame down through the Alternatives container to its child IK stages:
      // exposeTo DECLARES the props on the container, configureInitFrom INITS them from the parent.
      place->properties().exposeTo(alternatives->properties(), { "eef", "group", "ik_frame" });
      alternatives->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });
      const int n_place_angles = config_.place_n_angles;  // yaw candidates around the cylinder axis
      for (int i = 0; i < n_place_angles; ++i)
      {
        const double yaw = i * (2.0 * M_PI / n_place_angles);

        auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        stage->properties().set("marker_ns", "place_pose");
        stage->setObject(config_.object_id);

        geometry_msgs::msg::PoseStamped place_pose;
        place_pose.header.frame_id = config_.planning_frame;
        place_pose.pose.position.x = config_.place_pose[0];
        place_pose.pose.position.y = config_.place_pose[1];
        place_pose.pose.position.z = config_.place_pose[2];
        place_pose.pose.orientation.z = std::sin(yaw / 2.0);  // pure yaw quaternion
        place_pose.pose.orientation.w = std::cos(yaw / 2.0);
        stage->setPose(place_pose);
        stage->setMonitoredStage(attach_object_ptr);  // object is attached here

        auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
        wrapper->setMaxIKSolutions(config_.place_max_ik_solutions);
        wrapper->setMinSolutionDistance(config_.place_min_solution_distance);
        // Do NOT ignore collisions here: the place IK must produce COLLISION-FREE goals (expected
        // object/hand<->table contacts are already allowed via stage 1b) so the "move to place"
        // Connect can actually reach them. With ignore=true it generated in-collision goals that
        // OMPL rejected as GOAL_STATE_INVALID.
        wrapper->setIgnoreCollisions(false);
        wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
        wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });

        // Use the SAME IK frame as the grasp — same rotation AND same translation offsets. The object
        // is held by the hand in exactly that relationship, so the place hand pose must mirror the grasp
        // hand pose for IK to be feasible. (Previously only y was set, which is a different hand<->object
        // relationship than how the object is actually grasped, hence "no IK found".)
        Eigen::Isometry3d place_frame_transform = Eigen::Isometry3d::Identity();
        place_frame_transform.linear() =
          (Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(M_PI - grasp_tilt, Eigen::Vector3d::UnitX())).toRotationMatrix();
        place_frame_transform.translation().x() = pre_grasp_offset_x;
        place_frame_transform.translation().y() = pre_grasp_offset_y;
        place_frame_transform.translation().z() = pre_grasp_offset_z;
        wrapper->setIKFrame(place_frame_transform, hand_frame);

        alternatives->insert(std::move(wrapper));
      }
      place->insert(std::move(alternatives));
    }

    // 8.1b: Re-allow object<->table for the FORWARD stages (release/forbid/detach/retreat).
    //        The place IK generator produces fresh states that don't carry the task-level allowance,
    //        so re-assert it here (the persistent stage 1b keeps the move-to-place goal valid).
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (object,table) place");
      stage->allowCollisions(config_.object_id, config_.source_table_name, true);
      place->insert(std::move(stage));
    }

    // 8.2a: Relax the thumb pitch first while the other fingers continue to
    //       support the cylinder, as defined by the dedicated SRDF state.
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("prepare cylinder release", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal(config_.pose_hand_release_partial);
      place->insert(std::move(stage));
    }

    // 8.2b: Fully open the hand and release the cylinder.
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("fully release cylinder", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal(config_.pose_hand_release_full);
      place->insert(std::move(stage));
    }

    // 8.3: Forbid collision
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions(
        config_.object_id,
        handCollisionLinks(task.getRobotModel(), config_.hand_root_link),
        false);
      place->insert(std::move(stage));
    }

    // 8.4: Detach object
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject(config_.object_id, hand_frame);
      place->insert(std::move(stage));
    }

    // 8.5: Retreat from place (move UP in world frame to avoid stacked objects)
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(retreat_min_dist, retreat_max_dist);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "retreat");

      // Move UP in world frame (avoids collision with stacked cylinders)
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = config_.planning_frame;
      vec.vector.z = 1.0;  // Retreat upward
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    task.add(std::move(place));
  }

  // ========== STAGE 9: Return Home ==========
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", sampling_planner);
    stage->setGroup(arm_group_name);
    stage->setGoal(config_.pose_arm_home);
    task.add(std::move(stage));
  }

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
