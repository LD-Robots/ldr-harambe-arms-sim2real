# arm_mtc

MoveIt Task Constructor pick-and-place for the 7-DOF waist + left-arm chain.

The `arm` planning group is `waist_yaw_joint_X8` plus the six left-arm joints. That
redundancy is the point of the package and also its main difficulty: the same grasp is
reachable with the waist near zero or swung 100°, and preferring the former is not a
matter of setting a weight — see [`task.cost`](#taskcost--weighted-path-length).

## Quick start

From a clean workspace:

```bash
source setup_shortcuts.bash          # exports COLCON_DEFAULTS_FILE, so builds are Release
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch arm_mtc mtc_demo.launch.py
```

`colcon` does not read `colcon.defaults.yaml` on its own. In a shell that has not
sourced the shortcuts, pass `--cmake-args -DCMAKE_BUILD_TYPE=Release` yourself.

That brings up Gazebo with `lab-mtc.sdf`, controllers, `move_group`, RViz, the scene
publisher and the task node, then plans. It does **not** execute: pick a solution in
the RViz *Motion Planning Tasks* panel and run it from there.

```bash
ros2 launch arm_mtc mtc_demo.launch.py execute:=true       # plan and run immediately
ros2 launch arm_mtc mtc_demo.launch.py gazebo_gui:=false   # headless simulation
ros2 launch arm_mtc mtc_demo.launch.py use_rviz:=false
ros2 launch arm_mtc mtc_demo.launch.py --show-args         # every argument, includes and all
```

To iterate on the task without paying the ~20 s bringup each time, leave the demo
running and re-plan in a second terminal:

```bash
ros2 launch arm_mtc mtc_node_only.launch.py
```

## Contents

| Path | Purpose |
|---|---|
| `src/mtc_pick_place.cpp` | the task: builds and plans the MTC stage tree |
| `scripts/planning_scene_publisher.py` | owns the collision objects |
| `launch/mtc_demo.launch.py` | full stack; includes the two below |
| `launch/mtc_node_only.launch.py` | task node alone, against a running system |
| `launch/publish_planning_scene.launch.py` | collision scene alone |
| `config/mtc_task.yaml` | scene geometry and task tuning |
| `config/mtc.rviz` | RViz layout with the Motion Planning Tasks panel |

## Configuration

Everything tunable lives in [`config/mtc_task.yaml`](config/mtc_task.yaml). Point
`task_config:=` at your own copy to override it without editing the package:

```bash
ros2 launch arm_mtc mtc_node_only.launch.py task_config:=/path/to/my_task.yaml
```

### Two nodes, one file

The collision scene is **not** built by the task node. `planning_scene_publisher.py`
owns it and applies it once through the `/apply_planning_scene` service;
`mtc_pick_place` blocks in `waitForPlanningScene()` until the object shows up, then
plans against whatever is there. Splitting it that way means the scene can be
republished, inspected or replaced without touching the task, and the task can be
re-run repeatedly against a scene that is already in place.

Both nodes load the same file through the `/**` wildcard and each declares only the
keys it uses:

| Node | Reads |
|---|---|
| `planning_scene_publisher.py` | `robot.planning_frame`, all of `scene.*` |
| `mtc_pick_place` | all of `robot.*`, `poses.*`, `task.*`, plus `scene.object.id` and `scene.source_table.name` |

The two `scene.*` keys the task node reads are names, not geometry — it needs them to
allow the object↔table and object↔hand collision pairs.

The node's C++ defaults mirror the file so it runs standalone. Because they mirror it,
the logged values alone cannot tell "loaded" from "fell back", so startup reports
provenance explicitly:

```
[mtc_pick_place]: Task config: 36/36 keys from parameters
```

A `WARN` naming individual keys, or `all N keys fell back to built-in defaults`, means
the file is not reaching the node.

### `robot` and `poses` — model names

Group names, the IK target frame (`hand_frame`), the subtree root used to collect hand
collision links (`hand_root_link`), the planning frame, and the SRDF named states the
task drives to.

`robot.arm_joints` is **parallel to `task.cost.arm_weights`** — same order, same
length. The node refuses to start if the lengths disagree rather than silently
dropping weights.

### `scene` — collision objects

Geometry is expressed in `robot.planning_frame` (`urdf_base`). The Gazebo world spawns
the robot at world z = 0.85, so **every Z here is the world value minus 0.85**. Gazebo
physics and the MoveIt collision scene are two independent worlds kept in sync by hand;
changing the SDF does not change what the planner sees.

| Key | Notes |
|---|---|
| `object.id` | also used by the task node for collision allowances and attach/detach |
| `object.pose` | explicit, in `urdf_base` — not derived from the table |
| `source_table` / `destination_table` | `pose` is the box **centre**; with `size[2] = 0.95` and `pose[2] = -0.375` the top sits at 0.1 |
| `spawn_object`, `spawn_tables` | drop either from the scene without editing geometry |
| `republish_period` | `0` = apply once. The service call is acknowledged, so re-asserting is unnecessary and only makes `move_group` log an update every cycle. Raise it only if something else clears the scene. |

### `task.grasp` and the motion distances

| Key | Notes |
|---|---|
| `grasp.tilt` | radians off straight-down. A pure top grasp (`0`) sits at the kinematic edge — the arm reaches ~81° where the grasp needs 90° — so the approach is tilted down-forward into the reachable set. |
| `grasp.pre_grasp_offset` | IK-frame translation from `hand_frame`. `x` must exceed finger length (~0.05) or the open hand is generated inside the object. |
| `grasp.angle_delta` | grasp poses are swept around the cylinder axis; count is `2π / angle_delta`. Lowering it is the cheapest way to get more candidates. |
| `grasp.max_ik_solutions` | above 1 is often ineffective: MTC seeds only the first attempt from the current state and randomises all seven joints for every retry, which rarely converges within `kinematics_solver_timeout`. |
| `approach`, `lift`, `retreat` | Cartesian min/max distances. `lift.min_distance` is sensitive — at 0.02 it failed `min_fraction` and pruned grasp branches before the Connect stage could bridge to them. |
| `place.n_angles` | yaw candidates around the symmetric cylinder axis. Unlike the grasp IK, place IK does **not** ignore collisions, so these must be reachable and collision-free. |

### `task.cost` — weighted path length

`cost.waist_weight` and `cost.arm_weights` build the `PathLength` map used by both
`Connect` stages, and `waist_weight` also drives a `DistanceToReference` on the grasp
IK that pulls the waist toward zero. Grading the arm 7/7/5/3/1/1 from shoulder to
wrist makes swinging a proximal joint expensive and a wrist motion cheap.

**These rank solutions; they do not steer the search.** MTC orders `InterfaceState`s by
**depth before cost** — a partial solution already several stages deep outranks a
cheaper but shallower alternative, and cost only breaks ties at equal depth. Combined
with `planning.max_solutions` stopping at the first N complete solutions, the weights
sort whatever happened to finish rather than directing the search toward better
candidates. Raising the weight does not make the planner try harder; raising
`max_solutions` gives the ranking more to choose from.

One useful side effect: because the grasp IK carries only the waist term, its cost in
the RViz panel is literally `waist_weight × |waist|` — divide to read the waist angle
of each grasp candidate straight off the list.

### `task.planning`

| Key | Notes |
|---|---|
| `max_solutions` | `Task::plan` stops here. Low values end the search before the cost ranking has alternatives to compare. |
| `goal_joint_tolerance`, `cartesian_step_size` | handed to the OMPL and Cartesian solvers |
| `scene_wait_timeout` | how long to wait for the scene publisher before giving up |

## The task pipeline

Stages in execution order; the name is what appears in the RViz panel.

| Stage | Type |
|---|---|
| `current` | CurrentState |
| `allow collision (object,table)` | ModifyPlanningScene |
| `move to home` | MoveTo, OMPL |
| `open gripper` | MoveTo, interpolation |
| `move to grasp` | **Connect**, OMPL |
| `grasp` | SerialContainer ↓ |
| ⤷ `allow collision (hand,object)` | ModifyPlanningScene, propagates **backward** |
| ⤷ `grasp pose IK` | ComputeIK(GenerateGraspPose), collisions ignored |
| ⤷ `approach object` | MoveRelative, Cartesian |
| ⤷ `allow hand collisions (grasp)` | ModifyPlanningScene |
| ⤷ `prepare hand for cylinder grasp` → `grasp cylinder` | MoveTo ×2, interpolation |
| ⤷ `attach object` → `allow attached object<->table` | ModifyPlanningScene ×2 |
| ⤷ `lift` | MoveRelative, Cartesian |
| `move to place` | **Connect**, OMPL |
| `place` | SerialContainer ↓ |
| ⤷ `place pose alternatives` | Alternatives of ComputeIK(GeneratePlacePose) |
| ⤷ `allow collision (object,table) place` | ModifyPlanningScene |
| ⤷ `prepare cylinder release` → `fully release cylinder` | MoveTo ×2, interpolation |
| ⤷ `forbid collision (hand,object)` → `detach object` | ModifyPlanningScene ×2 |
| ⤷ `retreat` | MoveRelative, Cartesian |
| `return home` | MoveTo, OMPL |

## Reading the Motion Planning Tasks panel

The tree shows successes / failures / time per stage; the right pane lists that stage's
solutions with their cost. Two things are easy to misread:

**The highlighted solution is not the chosen one.** Clicking a stage selects its first
solution by default. To see what the task actually picked, click the **root** and
select a solution there — the panel then highlights the stage solutions making up that
chain.

**Per-stage costs are not comparable.** Only the two `Connect` stages and the grasp IK
carry weighted cost terms; everything else uses the unweighted default. The weighted
stages therefore dominate the total, and a cheap `return home` loses to a cheap
`move to place` every time. The task ranks by the **sum** over the whole chain.

## Requirements

- The world must contain the objects the task expects; `lab-mtc.sdf` is the default.
- `trac_ik_kinematics_plugin` — declared as an `exec_depend` of `arm_moveit_config`,
  installed by `rosdep`. KDL fails on this 7-DOF chain even where a solution exists.
- The RViz panel comes from `moveit_task_constructor_visualization`, and execution from
  RViz needs `move_group` started with `ExecuteTaskSolutionCapability` — both already
  wired in `arm_moveit_config`.

Introspection topics are published at the root: `/description`, `/statistics`,
`/solution`. Confirm with `ros2 topic list | grep -E "description|statistics|solution"`.

## References

- [MTC documentation](https://moveit.picknik.ai/main/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html)
- [MTC source](https://github.com/moveit/moveit_task_constructor)
