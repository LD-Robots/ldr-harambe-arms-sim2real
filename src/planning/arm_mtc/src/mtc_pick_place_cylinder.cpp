#include <rclcpp/rclcpp.hpp>
#include <cmath>
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

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_pick_place_cylinder");
namespace mtc = moveit::task_constructor;

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

// Object configuration structure
struct ObjectConfig {
  std::string id;
  double radius;
  double height;
  double pick_x, pick_y, pick_z;
  double place_x, place_y, place_z;
};

class MTCPickPlaceCylinder
{
public:
  MTCPickPlaceCylinder(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  bool doTask();

  void setupPlanningScene();
  
  bool loadObjectConfig();

  // Diagnostic: log the exact link pairs colliding at the close-gripper pose.
  void dumpCloseGripperContacts();

private:
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
  ObjectConfig object_config_;
  mtc::Stage* close_gripper_ptr_{ nullptr };  // for failure diagnostics
};

MTCPickPlaceCylinder::MTCPickPlaceCylinder(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_pick_place_cylinder", options) }
{
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCPickPlaceCylinder::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

bool MTCPickPlaceCylinder::loadObjectConfig()
{
  // Declare parameters if not already declared (handles both command-line and default cases)
  auto declare_if_not_declared = [this](const std::string& name, const auto& default_value) {
    if (!node_->has_parameter(name)) {
      node_->declare_parameter(name, default_value);
    }
  };

  declare_if_not_declared("object_id", std::string("target_cylinder"));
  // Defaults match the Gazebo lab-mtc.sdf scene, transformed world -> urdf_base
  // (robot spawns at world z=0.85, so subtract 0.85 from world Z).
  declare_if_not_declared("object_radius", 0.03035);   // = Gazebo lever radius
  declare_if_not_declared("object_height", 0.15);      // = Gazebo lever length
  declare_if_not_declared("pick_x", 0.45);             // test_table / lever world (0.45, 0.35, 1.025)
  declare_if_not_declared("pick_y", 0.35);
  declare_if_not_declared("pick_z", 0.175);            // 1.025 - 0.85
  declare_if_not_declared("place_x", 0.40);            // 5 cm toward the robot from the pick
  declare_if_not_declared("place_y", 0.35);            // same Y/Z as the pick
  declare_if_not_declared("place_z", 0.175);

  object_config_.id = node_->get_parameter("object_id").as_string();
  object_config_.radius = node_->get_parameter("object_radius").as_double();
  object_config_.height = node_->get_parameter("object_height").as_double();
  object_config_.pick_x = node_->get_parameter("pick_x").as_double();
  object_config_.pick_y = node_->get_parameter("pick_y").as_double();
  object_config_.pick_z = node_->get_parameter("pick_z").as_double();
  object_config_.place_x = node_->get_parameter("place_x").as_double();
  object_config_.place_y = node_->get_parameter("place_y").as_double();
  object_config_.place_z = node_->get_parameter("place_z").as_double();

  RCLCPP_INFO(LOGGER, "Loaded object config: id=%s, radius=%.3f, height=%.3f",
              object_config_.id.c_str(), object_config_.radius, object_config_.height);
  RCLCPP_INFO(LOGGER, "Pick pose: (%.3f, %.3f, %.3f)",
              object_config_.pick_x, object_config_.pick_y, object_config_.pick_z);
  RCLCPP_INFO(LOGGER, "Place pose: (%.3f, %.3f, %.3f)",
              object_config_.place_x, object_config_.place_y, object_config_.place_z);

  return true;
}

void MTCPickPlaceCylinder::setupPlanningScene()
{
  moveit::planning_interface::PlanningSceneInterface psi;
  
  // Helper to declare parameter if not already declared
  auto declare_if_not_declared = [this](const std::string& name, bool default_value) {
    if (!node_->has_parameter(name)) {
      node_->declare_parameter(name, default_value);
    }
  };
  
  // Check if we should spawn the object (external script may have already done it)
  declare_if_not_declared("spawn_object", true);
  bool spawn_object = node_->get_parameter("spawn_object").as_bool();
  
  if (spawn_object) {
    moveit_msgs::msg::CollisionObject cylinder;
    cylinder.id = object_config_.id;
    cylinder.header.frame_id = "urdf_base";
    cylinder.primitives.resize(1);
    cylinder.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    cylinder.primitives[0].dimensions = { object_config_.height, object_config_.radius };

    geometry_msgs::msg::Pose cylinder_pose;
    cylinder_pose.position.x = object_config_.pick_x;
    cylinder_pose.position.y = object_config_.pick_y;
    cylinder_pose.position.z = object_config_.pick_z;
    cylinder_pose.orientation.w = 1.0;
    cylinder.pose = cylinder_pose;

    psi.applyCollisionObject(cylinder);
    RCLCPP_INFO(LOGGER, "Spawned object '%s' at (%.3f, %.3f, %.3f)",
                object_config_.id.c_str(), object_config_.pick_x, 
                object_config_.pick_y, object_config_.pick_z);
  } else {
    RCLCPP_INFO(LOGGER, "Skipping object spawn (spawn_object=false)");
  }
  
  // Check if table should be added
  declare_if_not_declared("spawn_table", true);
  if (node_->get_parameter("spawn_table").as_bool()) {
    moveit_msgs::msg::CollisionObject table;
    table.id = "table";
    table.header.frame_id = "urdf_base";
    table.primitives.resize(1);
    table.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    table.primitives[0].dimensions = { 0.5, 0.3, 0.95 };  // = Gazebo test_table size

    geometry_msgs::msg::Pose table_pose;
    table_pose.position.x = 0.45;     // test_table world (0.45, 0.35, 0.475)
    table_pose.position.y = 0.35;
    table_pose.position.z = -0.375;   // 0.475 - 0.85; top at -0.375 + 0.475 = 0.1 = cylinder bottom
    table_pose.orientation.w = 1.0;
    table.pose = table_pose;

    psi.applyCollisionObject(table);
    RCLCPP_INFO(LOGGER, "Spawned table");

    // Destination table (= Gazebo destination_table), world (0.45, -0.35, 0.475)
    moveit_msgs::msg::CollisionObject dest_table;
    dest_table.id = "destination_table";
    dest_table.header.frame_id = "urdf_base";
    dest_table.primitives.resize(1);
    dest_table.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    dest_table.primitives[0].dimensions = { 0.5, 0.3, 0.95 };  // = Gazebo destination_table size

    geometry_msgs::msg::Pose dest_table_pose;
    dest_table_pose.position.x = 0.45;
    dest_table_pose.position.y = -0.35;
    dest_table_pose.position.z = -0.375;   // 0.475 - 0.85; top at 0.1
    dest_table_pose.orientation.w = 1.0;
    dest_table.pose = dest_table_pose;

    psi.applyCollisionObject(dest_table);
    RCLCPP_INFO(LOGGER, "Spawned destination table");
  }

  RCLCPP_INFO(LOGGER, "Planning scene setup complete");
}

// Diagnostic: take the REAL state at which "close gripper" failed (its own failure carries the true
// grasp arm config, the real right-arm pose, and every MTC allowance applied so far), drive the hand
// to the "close" pose, and list the contacts that remain. Whatever remains is exactly what blocks the
// close. Using the stage's failure (not a synthetic proxy) avoids fake collisions from default poses.
void MTCPickPlaceCylinder::dumpCloseGripperContacts()
{
  if (!close_gripper_ptr_ || close_gripper_ptr_->failures().empty()) {
    RCLCPP_WARN(LOGGER, "dumpCloseGripperContacts: no 'close gripper' failures to inspect");
    return;
  }

  collision_detection::CollisionRequest req;
  req.contacts = true;
  req.max_contacts = 200;
  req.max_contacts_per_pair = 1;

  int idx = 0;
  for (const auto& failure : close_gripper_ptr_->failures()) {
    const moveit::task_constructor::InterfaceState* start = failure->start();
    if (!start || !start->scene())
      continue;
    // Editable copy of the real pre-close scene (grasp config, hand open, MTC allowances intact).
    planning_scene::PlanningScenePtr scene = start->scene()->diff();
    moveit::core::RobotState& state = scene->getCurrentStateNonConst();
    if (const auto* hand = state.getJointModelGroup("hand"))
      state.setToDefaultValues(hand, "close");  // drive to the full close pose
    state.update();

    collision_detection::CollisionResult res;
    scene->checkCollision(req, res, state);  // uses the scene's (MTC) ACM => only real blockers remain

    RCLCPP_WARN(LOGGER, "=== close gripper failure #%d: %zu BLOCKING contact(s) ===",
                idx++, res.contacts.size());
    for (const auto& entry : res.contacts)
      RCLCPP_WARN(LOGGER, "  BLOCKER: %s  <->  %s",
                  entry.first.first.c_str(), entry.first.second.c_str());
  }
}

bool MTCPickPlaceCylinder::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task initialization failed: " << e);
    return false;
  }

  RCLCPP_INFO(LOGGER, "Starting task planning (max 5 solutions)...");
  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    dumpCloseGripperContacts();  // name the exact links blocking "close gripper"
    return false;
  }

  task_.introspection().publishSolution(*task_.solutions().front());
  RCLCPP_INFO(LOGGER, "Task planning succeeded! %zu solutions found.", task_.solutions().size());

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

mtc::Task MTCPickPlaceCylinder::createTask()
{
  mtc::Task task;
  task.stages()->setName("Pick and Place Cylinder");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "arm";
  const auto& hand_group_name = "hand";
  const auto& hand_frame = "left_tcp_link";

  // ========== CONFIGURABLE PARAMETERS ==========
  // Pre-grasp offset: distance from palm origin for IK (must be > 0 to avoid collision)
  // This is where the gripper will be BEFORE the approach

  const double pre_grasp_offset_x = 0.05;  // stand-off > finger length (~0.05) so the OPEN hand sits
                                         // just in front of the object, not penetrating it
  const double pre_grasp_offset_y = 0.02;   // stand-off in y-direction
  const double pre_grasp_offset_z = -0.06;   // stand-off in z-direction

  // Approach: move from pre_grasp position toward the object
  // Final grasp position = pre_grasp_offset_x - approach_distance
  const double approach_min_dist = 0.01;  // 1cm minimum approach
  const double approach_max_dist = 0.05;  // bring the hand all the way in to the grasp
  
  const double grasp_tilt = 0.2;     // tilt of the grasp off straight-down (~28 deg); shared by grasp + place
  
  const double lift_min_dist = 0.02;
  const double lift_max_dist = 0.10;
  const double retreat_min_dist = 0.02;
  const double retreat_max_dist = 0.10;

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

  // Create planners
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  sampling_planner->setProperty("goal_joint_tolerance", 1e-5);

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(0.01);

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
    stage->allowCollisions(object_config_.id, "table", true);
    // The closed hand wraps the cylinder down to its base (= table top), so the fingers/palm
    // touch the table at the place pose. Allow hand<->table too, else "move to place" rejects the
    // goal (GOAL_STATE_INVALID). At the pick the fingers are open and clear, so this is harmless there.
    const auto hand_links = handCollisionLinks(task.getRobotModel(), "left_hand_base_link");
    stage->allowCollisions("table", hand_links, true);
    task.add(std::move(stage));
  }

  // ========== STAGE 2: Move to Home Position ==========
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("move to home", sampling_planner);
    stage->setGroup(arm_group_name);
    stage->setGoal("home");
    task.add(std::move(stage));
  }

  // ========== STAGE 3: Open Gripper ==========
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
    stage->setGroup(hand_group_name);
    stage->setGoal("open");
    task.add(std::move(stage));
  }

  // ========== STAGE 4: Move to Ready Pose ==========
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("move to ready", sampling_planner);
    stage->setGroup(arm_group_name);
    stage->setGoal("ready");
    task.add(std::move(stage));
  }

  // ========== STAGE 5: Connect to Grasp Pose ==========
  // Use Connect stage to bridge from current state to the grasp pose generator
  {
    auto connect = std::make_unique<mtc::stages::Connect>(
      "move to grasp",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
    connect->properties().configureInitFrom(mtc::Stage::PARENT);
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
      const auto hand_links = handCollisionLinks(task.getRobotModel(), "left_hand_base_link");
      stage->allowCollisions(object_config_.id, hand_links, true);
      grasp->insert(std::move(stage));
    }

    // 6.2: Generate grasp poses (GENERATOR)
    {
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("open");
      stage->setObject(object_config_.id);
      stage->setAngleDelta(M_PI / 6);  // 12 poses around cylinder (30 degrees apart)
      stage->setMonitoredStage(current_state_ptr);

      // Wrap with ComputeIK
      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(4);
      wrapper->setMinSolutionDistance(0.1);
      // A dexterous hand always contacts the object at the grasp pose, and the upstream
      // "allow collision" stage does NOT reach the grasp-IK check (generator uses the
      // current_state scene). Skip collision checking for the grasp IK itself; the approach /
      // close / attach stages downstream run with hand-object collisions allowed.
      wrapper->setIgnoreCollisions(true);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });

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

    // 6.3b: Allow hand<->object AND hand self-collision for the close — in a FORWARD-propagating
    //        stage. Stage 6.1 ("allow collision (hand,object)") sits BEFORE the grasp-pose-IK
    //        generator, so it only propagates BACKWARD and never reaches "close gripper". Closing
    //        drives the fingers INTO the cylinder (the intended grasp) and into each other; both must
    //        be allowed here, after the generator, so the forward close stage sees them.
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow hand collisions (close)");
      const auto hand_links = handCollisionLinks(task.getRobotModel(), "left_hand_base_link");
      stage->allowCollisions(object_config_.id, hand_links, true);  // fingers grip INTO the cylinder
      stage->allowCollisions(hand_links, hand_links, true);         // fingers touch each other
      grasp->insert(std::move(stage));
    }

    // 6.4: Close gripper (use named pose from SRDF)
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("close");
      close_gripper_ptr_ = stage.get();  // for failure diagnostics
      grasp->insert(std::move(stage));
    }

    // 6.5: Attach object after gripper closed
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject(object_config_.id, hand_frame);
      grasp->insert(std::move(stage));
    }

    // 6.5b: Allow the now-ATTACHED object + closed hand to touch the table. The world-object
    //        allowance (stage 1b) does NOT carry over once the object is attached to the robot,
    //        so re-allow it here. GeneratePlacePose monitors THIS stage, so the place IK collision
    //        check sees the allowance and produces collision-free (reachable) goals.
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow attached object<->table");
      stage->allowCollisions(object_config_.id, "table", true);
      const auto hand_links = handCollisionLinks(task.getRobotModel(), "left_hand_base_link");
      stage->allowCollisions("table", hand_links, true);
      attach_object_ptr = stage.get();  // place IK monitors this (object attached + table allowed)
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
      vec.header.frame_id = "urdf_base";
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
      const int n_place_angles = 6;  // yaw candidates, 60 deg apart
      for (int i = 0; i < n_place_angles; ++i)
      {
        const double yaw = i * (2.0 * M_PI / n_place_angles);

        auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        stage->properties().set("marker_ns", "place_pose");
        stage->setObject(object_config_.id);

        geometry_msgs::msg::PoseStamped place_pose;
        place_pose.header.frame_id = "urdf_base";
        place_pose.pose.position.x = object_config_.place_x;
        place_pose.pose.position.y = object_config_.place_y;
        place_pose.pose.position.z = object_config_.place_z;
        place_pose.pose.orientation.z = std::sin(yaw / 2.0);  // pure yaw quaternion
        place_pose.pose.orientation.w = std::cos(yaw / 2.0);
        stage->setPose(place_pose);
        stage->setMonitoredStage(attach_object_ptr);  // object is attached here

        auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
        wrapper->setMaxIKSolutions(2);
        wrapper->setMinSolutionDistance(0.1);
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
      stage->allowCollisions(object_config_.id, "table", true);
      place->insert(std::move(stage));
    }

    // 8.2: Open gripper to release
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("release gripper", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("open");
      place->insert(std::move(stage));
    }

    // 8.3: Forbid collision
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions(
        object_config_.id,
        handCollisionLinks(task.getRobotModel(), "left_hand_base_link"),
        false);
      place->insert(std::move(stage));
    }

    // 8.4: Detach object
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject(object_config_.id, hand_frame);
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
      vec.header.frame_id = "urdf_base";
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
    stage->setGoal("home");
    task.add(std::move(stage));
  }

  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_node = std::make_shared<MTCPickPlaceCylinder>(options);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(mtc_node->getNodeBaseInterface());

  // Spin in background for service calls
  auto spin_thread = std::make_unique<std::thread>([&executor]() {
    executor.spin();
  });

  // Wait for system initialization
  RCLCPP_INFO(LOGGER, "Waiting for system initialization (3 seconds)...");
  rclcpp::sleep_for(std::chrono::seconds(3));

  // Load object configuration from parameters
  if (!mtc_node->loadObjectConfig()) {
    RCLCPP_ERROR(LOGGER, "Failed to load object configuration");
    executor.cancel();
    spin_thread->join();
    rclcpp::shutdown();
    return 1;
  }

  mtc_node->setupPlanningScene();

  RCLCPP_INFO(LOGGER, "Waiting 1 second for planning scene to update...");
  rclcpp::sleep_for(std::chrono::seconds(1));

  bool success = mtc_node->doTask();

  // Task complete - shutdown cleanly
  RCLCPP_INFO(LOGGER, "Task complete, shutting down...");
  executor.cancel();
  spin_thread->join();
  rclcpp::shutdown();
  return success ? 0 : 1;
}
