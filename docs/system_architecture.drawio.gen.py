#!/usr/bin/env python3
# Whole-system architecture — draw.io, A0/A1, STANDARD LIGHT THEME (white bg).
# Exhaustive + explained: every package, node, controller, plugin, topic, config.
import html, math

# ---- standard draw.io light swatches: (fill, stroke, font) ----
BLUE  =("#dae8fc","#6c8ebf","#13243f")
GREEN =("#d5e8d4","#82b366","#24401a")
ORANGE=("#ffe6cc","#d79b00","#6e4400")
YELLOW=("#fff2cc","#d6b656","#5f4d00")
RED   =("#f8cecc","#b85450","#5e1c18")
PURPLE=("#e1d5e7","#9673a6","#42285a")
TEAL  =("#d6e9e6","#4d9b94","#16433f")
GRAY  =("#f5f5f5","#999999","#333333")
WHITE =("#ffffff","#cccccc","#222222")
NOTE  =("#fbfbf7","#b0b0a8","#3a3a34")
INK="#222222"; DIM="#666666"; BG="#ffffff"; MONO="Courier New"

cells=[]; _id=[10]
def nid(): _id[0]+=1; return f"n{_id[0]}"
def esc(s): return s.replace("&","&amp;").replace("<","&lt;").replace(">","&gt;").replace('"',"&quot;")
def _lbl(title,sub,fc,tsize,ssize):
    s=f'<b style="font-size:{tsize}px;color:{fc}">{html.escape(title)}</b>'
    if sub:
        sub=html.escape(sub).replace("\n","<br>")
        s+=f'<br><span style="font-size:{ssize}px;color:{DIM};font-family:{MONO}">{sub}</span>'
    return s
def box(x,y,w,h,title,sub=None,c=GRAY,dashed=0,rounded=1,tsize=12,ssize=9):
    i=nid(); fill,stroke,fc=c
    st=(f"rounded={rounded};whiteSpace=wrap;html=1;fillColor={fill};strokeColor={stroke};fontColor={fc};"
        f"dashed={dashed};arcSize=5;strokeWidth=1.3;fontFamily=Helvetica;spacing=3;spacingLeft=7;spacingRight=6;"
        f"align=left;verticalAlign=top;")
    cells.append(f'<mxCell id="{i}" value="{esc(_lbl(title,sub,fc,tsize,ssize))}" style="{st}" vertex="1" parent="1">'
                 f'<mxGeometry x="{x}" y="{y}" width="{w}" height="{h}" as="geometry"/></mxCell>'); return i
def note(x,y,w,h,heading,text,accent):
    i=nid(); fill,stroke,fc=NOTE
    body=f'<b style="color:{accent};font-size:11px;letter-spacing:1px">{html.escape(heading)}</b><br>'\
         f'<span style="font-size:11px;color:#3a3a34;line-height:1.35">{html.escape(text)}</span>'
    st=(f"rounded=1;whiteSpace=wrap;html=1;fillColor={fill};strokeColor={accent};fontColor={fc};dashed=0;"
        f"arcSize=3;strokeWidth=1;fontFamily=Helvetica;spacing=4;spacingLeft=9;spacingRight=9;align=left;verticalAlign=top;")
    cells.append(f'<mxCell id="{i}" value="{esc(body)}" style="{st}" vertex="1" parent="1">'
                 f'<mxGeometry x="{x}" y="{y}" width="{w}" height="{h}" as="geometry"/></mxCell>'); return i
def chip(x,y,w,h,text,c=GRAY,bold=False,fs=9):
    i=nid(); fill,stroke,fc=c
    st=(f"rounded=1;whiteSpace=wrap;html=1;fillColor={fill};strokeColor={stroke};fontColor={fc};arcSize=25;"
        f"strokeWidth=1;fontFamily={MONO};fontSize={fs};align=center;verticalAlign=middle;spacing=1;")
    v=f"&lt;b&gt;{esc(text)}&lt;/b&gt;" if bold else esc(text)
    cells.append(f'<mxCell id="{i}" value="{v}" style="{st}" vertex="1" parent="1">'
                 f'<mxGeometry x="{x}" y="{y}" width="{w}" height="{h}" as="geometry"/></mxCell>'); return i
def frame(x,y,w,h,title,accent):
    i=nid()
    st=(f"rounded=0;whiteSpace=wrap;html=1;fillColor=none;strokeColor={accent};dashed=0;verticalAlign=top;"
        f"align=left;fontColor={accent};fontFamily={MONO};fontSize=15;fontStyle=1;spacingLeft=12;spacingTop=8;strokeWidth=2;")
    cells.append(f'<mxCell id="{i}" value="{esc(title)}" style="{st}" vertex="1" parent="1">'
                 f'<mxGeometry x="{x}" y="{y}" width="{w}" height="{h}" as="geometry"/></mxCell>'); return i
def subhdr(x,y,w,text,accent,h=24):
    i=nid()
    st=(f"rounded=0;whiteSpace=wrap;html=1;fillColor={accent};strokeColor={accent};fontColor=#ffffff;"
        f"fontFamily={MONO};fontSize=11;fontStyle=1;align=left;verticalAlign=middle;spacingLeft=9;strokeWidth=1;")
    cells.append(f'<mxCell id="{i}" value="{esc(text)}" style="{st}" vertex="1" parent="1">'
                 f'<mxGeometry x="{x}" y="{y}" width="{w}" height="{h}" as="geometry"/></mxCell>'); return i
def edge(src,tgt,label="",color="#888888",dashed=0,width=1.4,exitX=None,exitY=None,entryX=None,entryY=None,extra=""):
    i=nid()
    st=(f"edgeStyle=orthogonalEdgeStyle;rounded=1;html=1;strokeColor={color};strokeWidth={width};dashed={dashed};"
        f"fontColor=#444444;fontFamily={MONO};fontSize=9;endArrow=block;endFill=1;endSize=6;labelBackgroundColor=#ffffff;{extra}")
    if exitX is not None: st+=f"exitX={exitX};exitY={exitY};exitDx=0;exitDy=0;"
    if entryX is not None: st+=f"entryX={entryX};entryY={entryY};entryDx=0;entryDy=0;"
    cells.append(f'<mxCell id="{i}" value="{esc(label)}" style="{st}" edge="1" parent="1" source="{src}" target="{tgt}">'
                 f'<mxGeometry relative="1" as="geometry"/></mxCell>'); return i
def flow(items,x0,y0,colW,rowH,gapX,gapY,ncols,c=GRAY,tsize=11,ssize=8):
    ids=[]
    for k,it in enumerate(items):
        t,s=it[0],it[1]; col=it[2] if len(it)>2 else c; dsh=it[3] if len(it)>3 else 0
        r,cc=divmod(k,ncols); x=x0+cc*(colW+gapX); y=y0+r*(rowH+gapY)
        ids.append(box(x,y,colW,rowH,t,s,c=col,dashed=dsh,tsize=tsize,ssize=ssize))
    return ids
def bottom(n,y0,rowH,gapY,ncols):
    rows=math.ceil(n/ncols); return y0+rows*rowH+(rows-1)*gapY

LX=40; RX=3360; W=RX-LX

# ===================== TITLE + LEGEND =====================
i=nid()
st=(f"rounded=0;whiteSpace=wrap;html=1;fillColor=#ffffff;strokeColor=#333333;fontColor={INK};align=left;"
    f"verticalAlign=middle;spacingLeft=20;strokeWidth=2;")
tval=esc('<span style="font-family:Courier New;font-size:11px;color:#666;letter-spacing:3px">LDR · HARAMBE · SIM2REAL</span><br>'
         '<b style="font-size:25px;color:#111">Whole-System Architecture</b>'
         '<span style="font-size:13px;color:#555">&nbsp;&nbsp;— ROS 2 Jazzy dual-arm humanoid · 25 EtherCAT DOF · sim ↔ real · DDS middleware · 48 packages</span>')
cells.append(f'<mxCell id="{i}" value="{tval}" style="{st}" vertex="1" parent="1"><mxGeometry x="{LX}" y="24" width="2160" height="60" as="geometry"/></mxCell>')
i=nid()
st=(f"rounded=0;whiteSpace=wrap;html=1;fillColor=#ffffff;strokeColor=#bbbbbb;fontColor=#444;align=left;"
    f"verticalAlign=middle;spacingLeft=12;fontFamily={MONO};fontSize=9;")
def sw(c): return c[0]
lgval=esc('<b style="color:#222">LAYER COLOURS</b>&nbsp; '
          '<span style="background:%s;color:#13243f">&nbsp;UI&nbsp;</span> '
          '<span style="background:%s;color:#6e4400">&nbsp;planning&nbsp;</span> '
          '<span style="background:%s;color:#5f4d00">&nbsp;DDS&nbsp;</span> '
          '<span style="background:%s;color:#24401a">&nbsp;control&nbsp;</span> '
          '<span style="background:%s;color:#42285a">&nbsp;model/RL&nbsp;</span> '
          '<span style="background:%s;color:#16433f">&nbsp;sim&nbsp;</span> '
          '<span style="background:%s;color:#5e1c18">&nbsp;real-HW/safety&nbsp;</span><br>'
          '<span style="color:#555">—⟶ DDS topic / action / service&nbsp;&nbsp;&nbsp;⟹ in-process plugin (read/write, no DDS)&nbsp;&nbsp;&nbsp;– – offline / optional&nbsp;&nbsp;&nbsp;'
          'motor type X4 / X6 / X8</span>'
          %(sw(BLUE),sw(ORANGE),sw(YELLOW),sw(GREEN),sw(PURPLE),sw(TEAL),sw(RED)))
cells.append(f'<mxCell id="{i}" value="{lgval}" style="{st}" vertex="1" parent="1"><mxGeometry x="2220" y="24" width="1140" height="60" as="geometry"/></mxCell>')

# ============================================================ §01 UI
Ay=104; Ah=486
frame(LX,Ay,W,Ah,"§ 01 · USER INTERFACE  ·  OPERATOR & DIAGNOSTIC TOOLS",BLUE[1])
note(LX+16,Ay+40,W-32,64,"WHAT THIS LAYER DOES",
 "Everything a human touches. Operators visualise robot state in RViz2, drive the arms by hand through teleop, and watch drive health on PyQt5 / Rich dashboards. "
 "Every item here is an ordinary ROS 2 node: it only ever reaches the rest of the system over DDS topics, actions and services — never in-process. "
 "full_system_launcher offers three modes — Full-Local (sim+control on one PC), Robot (real hardware), Client (remote viewer on the same ROS_DOMAIN_ID).",BLUE[1])
sy=Ay+114
subhdr(LX+16,sy,1056,"VISUALISATION  ·  RViz2 + remote view",BLUE[1])
subhdr(LX+16+1072,sy,1052,"arm_gui_tools  (PyQt5 / Rich) — 11 nodes",BLUE[1])
subhdr(LX+16+2144,sy,W-32-2144,"diagnostics · calibration · gamepad",BLUE[1])
viz=[("RViz2","RobotModel·MotionPlanning·TF·sensors\nreads /robot_description(TL)·/joint_states·/tf·/planning_scene",BLUE),
 ("remote_view  (robot_bringup)","headless RViz, any PC on same ROS_DOMAIN_ID\nsubs /robot_description·/tf·/joint_states · domain_id arg",BLUE),
 ("moveit.rviz · _with_octomap","MotionPlanning panel + octomap collision",BLUE),
 ("mtc.rviz","MoveIt Task-Constructor solution stages",BLUE),
 ("view_robot.rviz ×3","arm · dual_arm · full_robot model viewers",BLUE),
 ("skeleton_teleop.rviz","RealSense operator-pose overlay",BLUE)]
viz_ids=flow(viz,LX+16,sy+30,512,78,16,8,2,c=BLUE,tsize=11,ssize=8)
gui=[("full_system_launcher","orchestrator · modes full_local/robot/client\nworld dropdown · dependency-locked start/stop",BLUE),
 ("action_position_sender","per-controller sliders → FollowJointTrajectory",BLUE),
 ("joint_state_monitor · _tui","live pos/vel/eff table · Rich multi-tab TUI",BLUE),
 ("joint_monitor · joint_controller_gui","single-joint inspect + command",BLUE),
 ("ethercat_monitor","subs /diagnostics·/drive_status · CiA402 · PDO",BLUE),
 ("actuator_status · motor_monitor_gui","per-drive temp · fault · CiA402 state",BLUE),
 ("power_monitor_gui","bus voltage / power draw",BLUE),
 ("trajectory_recorder_gui","record + replay joint trajectories",BLUE)]
gui_ids=flow(gui,LX+16+1072,sy+30,512,78,16,8,2,c=BLUE,tsize=11,ssize=8)
tl=[("arm_teleop  (9 nodes)","keyboard·joystick·cartesian(/compute_ik)·servo\n·visual(/hand_pose)·skeleton·calib·joint_teleop(C++)·gui",BLUE),
 ("ethercat_tools demo_*","ethercat_status·joint_state(auto SIM/REAL)·joint_offset",BLUE),
 ("joint_calibration suite","calibration_window·apply_offsets·reverse_direction·robot_viewer",BLUE),
 ("pdo_formulas · chain_debug","CiA402 raw↔SI · URDF chain diagnose",BLUE),
 ("gravcomp_from_urdf · recompute_torque_factors","extract mgl/q_eq · per-family τ factors",BLUE),
 ("robot_tools","pvt_dashboard · pvt_tuner · bus_voltage_viewer",BLUE),
 ("dualsense_node (dualsense_tools)","PS5 pad → /dualsense_joy (sensor_msgs/Joy)",BLUE)]
tl_ids=flow(tl,LX+16+2144,sy+30,(W-32-2144-16)//2,78,16,8,2,c=BLUE,tsize=11,ssize=8)

# ============================================================ §02 PLANNING / RL
By=610; Bh=420
frame(LX,By,W,Bh,"§ 02 · APPLICATION  ·  MOTION PLANNING (MoveIt 2)  ·  LEARNING (RL)",ORANGE[1])
note(LX+16,By+42,W-32,60,"WHAT THIS LAYER DOES",
 "Turns goals into joint trajectories. MoveIt 2's move_group plans collision-free paths (OMPL / Pilz) and solves IK (KDL), then streams them to the controllers as a FollowJointTrajectory action via the simple-controller-manager. "
 "In parallel, the learning path trains a balancing/locomotion policy in Isaac Gym/Lab, exports it to ONNX, and deploys it through harambe_policy_controller — the same actuator PD model is used in sim and on hardware.",ORANGE[1])
sy=By+112
subhdr(LX+16,sy,2036,"MoveIt 2  ·  move_group + 5 config packages + MTC",ORANGE[1])
subhdr(LX+16+2052,sy,W-32-2052,"REINFORCEMENT LEARNING  ·  sim-to-real policy",PURPLE[1])
mv=[("move_group  (MoveIt 2)","planning-scene monitor · pipelines:\nOMPL · Pilz industrial · CHOMP",ORANGE),
 ("KDL kinematics","kinematics.yaml · IK/FK · 0.005 res · 0.2s timeout",ORANGE),
 ("moveit_simple_controller_manager","moveit_controllers.yaml →\nFollowJointTrajectory per arm/hand",ORANGE),
 ("moveit_py","python planning + execution API",ORANGE),
 ("arm_moveit_config","SRDF groups arm(chain base→wrist)+hand\nposes home/ready/open/close · joint_limits · ompl",ORANGE),
 ("dual_arm_moveit_config","groups left/right/dual_arm + hands",ORANGE),
 ("left_arm_plannig","dedicated left-arm planning config",ORANGE),
 ("arm_gripper / arm_hand _moveit_config","gripper · 6-DOF hand planning groups",ORANGE),
 ("arm_mtc  (Task Constructor)","mtc_node · mtc_node2 · pick_place_task (C++)",ORANGE),
 ("  mtc_pick_place_cylinder","grasp-generator → IK → Cartesian stages",ORANGE),
 ("planning scene","/planning_scene · collision objects · octomap",ORANGE),
 ("motion_player","record / playback whole trajectories",ORANGE)]
mv_ids=flow(mv,LX+16,sy+30,(2036-3*16)//4,84,16,10,4,c=ORANGE,tsize=11,ssize=8)
rl=[("Isaac Gym / Isaac Lab","massively-parallel RL · domain randomisation\n(PD, friction, latency, mass)",PURPLE,1),
 ("ActuatorCfg PD model","per-joint kp/kd/effort/friction/default-q\nSAME gains used in deploy (kp_scale=1.0)",PURPLE,1),
 ("ONNX policy","exported actor net · obs→action",PURPLE,1),
 ("dual_arm_isaac","ROS↔Isaac bridge\n/isaac_joint_states ↔ /isaac_joint_commands",PURPLE,1),
 ("→ harambe_policy_controller","loads ONNX as ros2_control plugin (see §04)",PURPLE),
 ("mc_rtc_controller","whole-body QP — COLCON_IGNORE (WIP)",PURPLE,1)]
rl_ids=flow(rl,LX+16+2052,sy+30,(W-32-2052-2*16)//3,84,16,10,3,c=PURPLE,tsize=11,ssize=8)

# ============================================================ §03 MIDDLEWARE / DDS
Cy=1050; Ch=560
frame(LX,Cy,W,Ch,"§ 03 · MIDDLEWARE  —  ROS 2 client libraries  ·  rmw  ·  DDS transport",YELLOW[1])
note(LX+16,Cy+42,W-32,76,"WHAT THIS LAYER DOES  (the glue — this is the 'how things interconnect')",
 "Every arrow that crosses a layer boundary above or below is really a DDS exchange here. ROS 2 nodes call rclcpp/rclpy, which sit on rcl, which sits on the rmw abstraction, which is implemented by a DDS vendor. This workspace runs rmw_cyclonedds_cpp "
 "(RMW_IMPLEMENTATION is set); rmw_fastrtps_cpp is the drop-in alternative. DDS carries pub/sub topics, request/response services and goal-based actions over RTPS (UDP + shared-memory). Nodes find each other by ROS_DOMAIN_ID; QoS contracts "
 "(reliability, durability, history, deadline) decide delivery — e.g. /robot_description is latched transient_local so late-joining RViz still gets the model. NOTE: ros2_control ↔ hardware is the ONE link that is NOT DDS — it is an in-process C++ call.",YELLOW[1])
sy=Cy+126
# left stack
stk=[("rclcpp · rclpy","nodes · executors · callback groups · timers",YELLOW),
 ("rcl","C client-library core",YELLOW),
 ("rmw  (abstraction)","ROS middleware interface",YELLOW),
 ("rmw_cyclonedds_cpp","ACTIVE · Eclipse Cyclone DDS",YELLOW),
 ("rmw_fastrtps_cpp","alt · eProsima Fast DDS",YELLOW),
 ("RTPS / DDS wire","UDP + shared-memory (Iceoryx)",YELLOW)]
stk_ids=flow(stk,LX+16,sy,300,42,14,6,1,c=YELLOW,tsize=11,ssize=8)
# QoS / discovery column
qos=[("Discovery","ROS_DOMAIN_ID · simple multicast / discovery-server",YELLOW),
 ("QoS: reliability","reliable (control) · best-effort (high-rate state)",YELLOW),
 ("QoS: durability","transient_local = latched /robot_description, /tf_static",YELLOW),
 ("QoS: history/depth","keep-last N · deadline · liveliness",YELLOW),
 ("graph services","node params · lifecycle · /rosout logging",YELLOW),
 ("clock","/clock — sim time when use_sim_time",YELLOW)]
qos_ids=flow(qos,LX+16+318,sy,330,42,14,6,1,c=YELLOW,tsize=11,ssize=8)
# DDS bus + topic chips
busx=LX+16+318+330+24; busw=RX-16-busx
dds=box(busx,sy,busw,44,"DDS DATA BUS  —  typed messages between every node",
   "publish/subscribe topics · request/response services · goal/feedback/result actions",c=YELLOW,tsize=13,ssize=10)
groups=[("STATE",GREEN,["/joint_states  JointState","/joint_states_raw  1kHz","/joint_states_filtered  100Hz",
   "/tf  TFMessage","/tf_static  (latched)","/robot_description  String(TL)","/drive_status  Float64MA","/clock  Clock"]),
 ("SENSORS·CMD",BLUE,["/pelvis/imu  Imu","/torso/imu  Imu","/pelvis/grav  Vector3","/torso/grav  Vector3",
   "/cmd_vel  Twist","/joy  Joy","/dualsense_joy  Joy","/hand_pose  PoseStamped"]),
 ("CONTROL",ORANGE,["/mode_controller/commands  F64MA","/<grp>/joint_trajectory","/robot_pvt_controller/setpoint",
   "~/hold ~/free ~/damp  Trigger","/ankle_method_compare  JointState","/servo_target_twist"]),
 ("SAFETY",RED,["/safety/status  Bool","/diagnostics  DiagnosticArray","/safety/estop  Trigger","/safety/reset  Trigger"]),
 ("ACTIONS",PURPLE,["FollowJointTrajectory","/move_group  MoveGroup","/execute_trajectory"]),
 ("SERVICES",GRAY,["/controller_manager/switch|load|list","/compute_ik  GetPositionIK","/get_sdo·/set_sdo  ethercat_msgs","/plan_kinematic_path"]),
]
cy=sy+58; cw=250; ch=26; gx=8; gy=7; x0=busx; maxx=RX-16
for cat,col,topics in groups:
    chip(x0,cy,120,ch,cat,c=col,bold=True)
    xc=x0+120+gx
    for t in topics:
        if xc+cw>maxx: cy+=ch+gy; xc=x0+120+gx
        chip(xc,cy,cw,ch,t,c=GRAY); xc+=cw+gx
    cy+=ch+gy

# ============================================================ §04 CONTROL
Dy=1630; Dh=640
frame(LX,Dy,W,Dh,"§ 04 · CONTROL  —  ros2_control  (controller_manager)",GREEN[1])
note(LX+16,Dy+42,W-32,76,"WHAT THIS LAYER DOES",
 "The real-time heart. controller_manager (ros2_control_node) runs a periodic update loop (100 Hz CSP/CST · 200 Hz policy · 1 kHz PVT) on a SCHED_FIFO thread with mlockall and CPU pinning. Each cycle it calls hardware read(), runs the active "
 "controllers, then hardware write() — the hardware is an IN-PROCESS C++ plugin, NOT a DDS peer. Controllers are composable and claim DISJOINT interfaces on the same joints, so e.g. a trajectory controller (position), an effort controller and the "
 "mode_controller (mode_of_operation) coexist without conflict; this is what makes runtime CSP↔CST↔PVT mode switching possible. Broadcasters publish state outward to DDS; two independent safety supervisors can deactivate controllers on breach.",GREEN[1])
cm=box(LX+16,Dy+124,W-32,46,"controller_manager  (ros2_control_node)",
   "update_rate 100/200/1000 Hz · SCHED_FIFO prio 80 · lock_memory(mlockall) · cpu_affinity 3 (isolcpus) · loads URDF ros2_control plugin, spawns controllers, exposes /controller_manager/* services",
   c=GREEN,tsize=13,ssize=9)
sy=Dy+182
subhdr(LX+16,sy,820,"BROADCASTERS → DDS",GREEN[1])
subhdr(LX+16+836,sy,1320,"CONTROLLERS (disjoint interfaces · sim + real)",GREEN[1])
subhdr(LX+16+2172,sy,W-32-2172,"SAFETY SUPERVISORS",RED[1])
bc=[("robot_state_publisher","URDF → /tf · publishes /robot_description",GREEN),
 ("joint_state_broadcaster","primary raw feed → /joint_states_raw (1 kHz)",GREEN),
 ("robot_filtered_joint_state_broadcaster","IIR 30 Hz vel/eff → /joint_states_filtered @100 Hz",GREEN),
 ("robot_drive_status_broadcaster","TxPDO 0x1A02 temp/busV/err → /drive_status @50 Hz",GREEN)]
bc_ids=flow(bc,LX+16,sy+30,(820-16)//2,80,16,10,2,c=GREEN,tsize=11,ssize=8)
ct=[("left/right_arm_trajectory_controller","JointTrajectoryController · position · CSP",GREEN),
 ("waist_controller","JTC · waist_yaw_X8 · position",GREEN),
 ("left/right_leg_trajectory_controller","JTC · position",GREEN),
 ("body/legs/whole_body_controller","JointGroupPositionController (sim)",GREEN),
 ("*_effort_controller (arm/leg/waist)","JointGroupEffortController · CST · inactive",GREEN),
 ("mode_controller","ForwardCommandController · mode_of_operation",GREEN),
 ("robot_pvt_controller  +upper/lower","RobotPVTController · CiA402 mode 5",GREEN),
 ("harambe_policy_controller","ONNX RL · effort · 200/50 Hz",PURPLE),
 ("implicit_actuator_controller","Isaac ImplicitActuator PD replica",PURPLE)]
ct_ids=flow(ct,LX+16+836,sy+30,(1320-2*14)//3,80,14,10,3,c=GREEN,tsize=11,ssize=8)
sf=[("arm_ethercat_safety  (safety_monitor_node)","subs /joint_states → /safety/status·/diagnostics",RED),
 ("  watchdog · estop_handler","heartbeat timeout · /safety/estop·/reset",RED),
 ("  joint_limit_monitor · fault_handler","pos/vel/eff limits · CiA402 0x603F faults",RED),
 ("robot_safety  (supervisor_node)","whole-body supervisor · librobot_safety",RED),
 ("  breach · thermal · clamp","Kp thermal-derate · position clamp",RED),
 ("  sustained_effort_monitor","overload integral guard",RED)]
sf_ids=flow(sf,LX+16+2172,sy+30,(W-32-2172-14)//2,52,14,8,2,c=RED,tsize=11,ssize=8)
# detail panels: PVT + policy + interfaces
py=sy+30+max(bottom(4,0,80,10,2),bottom(9,0,80,10,3),bottom(6,0,52,8,2))+16
box(LX+16,py,1480,118,"robot_pvt_controller  ·  PVT (CiA 402 mode 5) parameters",
 "drive_side_pd:true (drive runs PD law)  ·  body_group full | upper | lower (split plugin instances, disjoint rosters)\n"
 "cmd per joint: q_des, v_des, τ_ff, Kp, Kd  →  τ = Kp·Δq + Kd·Δv + τ_ff in firmware @1 kHz\n"
 "gravity FF τ_g = mgl[j]·sin(q[j]−q_eq[j]) (arms only, bench-tuned per joint, ff_gravity false at boot)\n"
 "Kd_damp braking ramp · lag governor lag_free 0.04s / lag_pause 0.14s / alpha_slew 2.0 · spawned INACTIVE",
 c=GRAY,tsize=12,ssize=9)
box(LX+16+1496,py,1000,118,"harambe_policy_controller  ·  RL deploy parameters",
 "update 200 Hz · policy 50 Hz (decimation 4) · effort command (drive-side PD)\n"
 "obs: joint q/qd, IMU ω+grav (/pelvis,/torso), /cmd_vel, last action · action_scale 0.25 · clip 5.0\n"
 "kp Isaac ActuatorCfg: shoulder≈114 · hip≈466 · ankle≈996 · 25 joints HARDWARE_JOINT_ORDER\n"
 "warmup 200 steps (boosted gains) · fall guard |grav_z|<0.5 · ~/enable gate",
 c=GRAY,tsize=12,ssize=9)
box(LX+16+2512,py,W-32-2512,118,"ros2_control interfaces",
 "command:\n  position · velocity · effort\n  kp · kd · mode_of_operation\n"
 "state:\n  position · velocity · effort · status_word\n"
 "CiA402 modes: 8 CSP · 9 CSV · 10 CST · 5 PVT",
 c=GREEN,tsize=12,ssize=9)

# ============================================================ §05 ROBOT MODEL
Ey=2290; Eh=300
frame(LX,Ey,W,Eh,"§ 05 · ROBOT MODEL  —  URDF / xacro  (single source of truth)",PURPLE[1])
note(LX+16,Ey+42,W-32,68,"WHAT THIS LAYER DOES",
 "One parametric model feeds every layer. full_robot_description assembles links + joints (exact, never-rounded origins from urdf/joints/*) into the robot. The use_sim xacro arg is the master sim/real switch: true loads the Gazebo "
 "gz_ros2_control hardware tag, false loads the EtherCAT hardware tag with per-joint <ec_module> slave configs. Further args select PVT vs CSP, read-only vs gravcomp config dirs, ankle driver v1/v2, fixed_legs, only_left. "
 "robot_state_publisher turns this model into /tf; MoveIt reads it as /robot_description; the controllers read joint limits and effort caps from it.",PURPLE[1])
sy=Ey+120
rm=[("full_robot_description","25 DOF + hands · full_robot.urdf.xacro\nview_robot.rviz · full_robot_gazebo.xacro",PURPLE),
 ("urdf/joints/*.xacro","body·left/right arm·left/right foot\nexact joint origins (source of truth)",PURPLE),
 ("urdf/links/*.xacro","masses · inertias · meshes · collision",PURPLE),
 ("arm · dual_arm _description","single + dual-arm sub-models",PURPLE),
 ("hand_description","Inspire 6-DOF hand ×2 (12 joints)",PURPLE),
 ("camera_description · imu_description","Orbbec Gemini 335/336L · BNO055 ×2",PURPLE),
 ("actuator_properties.xacro","damping · friction · effort/vel limits",PURPLE),
 ("ros2_control macros","real · real_pvt · effort · position · isaac · (sim)",PURPLE),
 ("materials · gazebo_extensions","visual colours + gz sensor/plugin tags",PURPLE),
 ("camera.xacro · depth_camera.xacro","RGB + depth sensor macros",PURPLE),
 ("use_sim switch","true→Gazebo plugin · false→EtherCAT <ec_module>",PURPLE),
 ("xacro args","pvt_mode·readonly·gravcomp·ankle_driver v1/v2\nfixed_legs·only_left",PURPLE)]
rm_ids=flow(rm,LX+16,sy,(W-32-5*16)//6,74,16,10,6,c=PURPLE,tsize=11,ssize=8)

# ============================================================ SIM-TO-REAL BANNER
Sy=2610
i=nid()
st=(f"rounded=1;whiteSpace=wrap;html=1;fillColor=#fff7e6;strokeColor=#d79b00;fontColor=#5a3d00;dashed=0;arcSize=3;"
    f"strokeWidth=2;align=center;verticalAlign=middle;fontFamily=Helvetica;fontSize=12;")
sval=esc('<b>SIM-TO-REAL CONTRACT</b>&nbsp;&nbsp; identical ros2_control graph &amp; <b>hardware_interface::SystemInterface</b> — only the hardware plugin changes via <b>use_sim</b>.'
         '&nbsp;&nbsp; Pipeline:&nbsp; Isaac Gym/Lab (domain-randomised PD) ⟶ ONNX policy ⟶ Gazebo validation ⟶ Real hardware.'
         '&nbsp;&nbsp; Gap priority: PD-mismatch ▸ actuator-lag ▸ friction ▸ latency ▸ obs-normalisation.')
cells.append(f'<mxCell id="{i}" value="{sval}" style="{st}" vertex="1" parent="1"><mxGeometry x="{LX}" y="{Sy}" width="{W}" height="48" as="geometry"/></mxCell>')

# ============================================================ §06 HW ABSTRACTION (sim | real)
Fy=2682; Fh=636
LW=1610; RXX=1700; RW=RX-RXX
frame(LX,Fy,LW,Fh,"§ 06A · SIMULATION PATH   (use_sim:=true)",TEAL[1])
note(LX+16,Fy+40,LW-32,86,"HOW IT CONNECTS",
 "Gazebo loads the gz_ros2_control plugin, which presents the SAME hardware_interface::SystemInterface that controller_manager expects — so identical controllers run. ros_gz_bridge mirrors /clock, sensors and TF between Gazebo and ROS. "
 "Physics (DART) closes the loop: commands move sim joints, sim encoders/IMU/camera feed state back. dual_arm_isaac is an alternate physics backend via an Isaac joint-state bridge.",TEAL[1])
sy=Fy+132
subhdr(LX+16,sy,LW-32,"HARDWARE ABSTRACTION (sim)",TEAL[1])
sim=[("gz_ros2_control","GazeboSimROS2ControlPlugin\nlibgz_ros2_control-system.so",TEAL),
 ("GazeboSimSystem","per-joint sim hardware · position/effort",TEAL),
 ("ros_gz_bridge","/clock · sensors · TF bridge",TEAL),
 ("ros2_control_position/effort.xacro","sim hardware tags (control_mode)",TEAL)]
sim_ids=flow(sim,LX+16,sy+28,(LW-32-16)//2,66,16,10,2,c=TEAL,tsize=11,ssize=8)
sy2=sy+28+bottom(4,0,66,10,2)+10
subhdr(LX+16,sy2,LW-32,"PHYSICAL — Gazebo Harmonic (gz sim)",TEAL[1])
simp=[("Gazebo Harmonic (gz sim)","DART physics · sensors-system · ogre2",TEAL),
 ("arm_gazebo worlds","lab-ldr · lab-mtc · lab · whole_arm_workspace",TEAL),
 ("dual_arm/full_robot_gazebo","lab-ldr · empty.sdf",TEAL),
 ("world models","table·walls·chair·mug·trash·visitor",TEAL),
 ("simulated sensors","RGB-D camera · IMU · joint encoders",TEAL),
 ("dual_arm_isaac","Isaac Sim bridge (alt physics)",TEAL,1)]
simp_ids=flow(simp,LX+16,sy2+28,(LW-32-16)//2,64,16,10,2,c=TEAL,tsize=11,ssize=8)

frame(RXX,Fy,RW,Fh,"§ 06B · REAL HARDWARE PATH   (use_sim:=false · EtherCAT)",RED[1])
note(RXX+16,Fy+40,RW-32,86,"HOW IT CONNECTS",
 "ethercat_driver's EthercatDriver implements SystemInterface; its read()/write() exchange PDO process-images with the EtherLab IgH master each cycle. Per joint an EcCiA402Drive module runs the CiA 402 state machine and maps SI↔raw via a "
 "source-controlled YAML (factors, offsets, SDO init). The two ankles aren't direct-drive: harambe_ankle_ethercat_driver (v1 analytic / v2 calibrated-cubic) wraps the driver and converts joint pitch/roll ↔ the two pushrod motors with IK/FK + Jacobian.",RED[1])
sy=Fy+132
subhdr(RXX+16,sy,RW-32,"HARDWARE ABSTRACTION (real)  ·  ethercat_driver_ros2 + ankle drivers",RED[1])
real=[("EthercatDriver  (SystemInterface)","ethercat_driver · read()/write() · domains\n+ EthercatSafetyDriver variant",RED),
 ("harambe_ankle_ethercat_driver v1/v2","AnkleLinkageDriver · IK/FK + Jacobian\nHarambeEthercatDriver / HarambePvtDriver",RED),
 ("EcCiA402Drive  (per joint)","ethercat_generic_cia402_drive\nCiA402 state machine · PDO 0x1600/0x1A00",RED),
 ("GenericEcSlave","ethercat_generic_slave · YAML PDO map",RED),
 ("EcMaster / ethercat_interface","ecrt.h · domains · PDO · SDO · sync-mgrs",RED),
 ("ethercat_manager","EcMasterAsync · /dev/EtherCAT ioctl",RED),
 ("ethercat_msgs","GetSdo · SetSdo srv",RED),
 ("EtherLab IgH master","kernel real-time master · DC SYNC0/1",RED)]
real_ids=flow(real,RXX+16,sy+28,(RW-32-3*14)//4,68,14,10,4,c=RED,tsize=10,ssize=8)
sy3=sy+28+bottom(8,0,68,10,4)+10
# PDO byte map + config families
box(RXX+16,sy3,880,150,"CiA 402 PDO byte-map  (MyActuator RMD X V4 · 16 B)",
 "RxPDO 0x1600 master→drive:\n 0x6040 ctrl_word · 0x607A target_pos(factor·rad→cnt, offset) · 0x60FF target_vel · 0x6071 target_torque(‰ rated-I) · 0x6072 max_torque · 0x6060 mode_op · pad\n"
 "TxPDO 0x1A00 drive→master:\n 0x6041 status_word · 0x6064 pos · 0x606C vel · 0x6077 torque · 0x603F error · 0x6061 mode_disp · pad\n"
 "PVT 22 B adds Rx 0x2000 Kp / 0x2001 Kd · Tx 0x2009 motor_temp / 0x200C drive_temp / 0x200A bus_V\n"
 "SDO@boot: 0x6072 max_torque(500=50%) · 0x60C2 interp 10 ms(100 Hz)/1 ms(PVT) · assign_activate 0x0300",
 c=GRAY,tsize=11,ssize=9)
subhdr(RXX+16+896,sy3,RW-32-896,"slave config families (per-joint YAML, source-controlled)",RED[1])
cfg=[("ethercat/ (25)","arm_real_bringup · CSP mode 8",RED),
 ("ethercat_pvt/ (25)","robot_bringup · PVT mode 5 · 1 ms",RED),
 ("ethercat_readonly/","auto_state_transitions:false (viewer)",RED),
 ("ethercat_gravcomp/","CST mode 10 · torque",RED),
 ("factors / offsets","X4/X6/X8 pos·vel·τ · dir=−1 negate · zero-offset",RED),
 ("recompute_torque_factors","‰ rated-current derivation (never hand-edit)",RED)]
cfg_ids=flow(cfg,RXX+16+896,sy3+28,(RW-32-896-14)//2,56,14,8,2,c=RED,tsize=10,ssize=8)

# ============================================================ §07 PHYSICAL
Gy=3338; Gh=470
frame(LX,Gy,W,Gh,"§ 07 · PHYSICAL ROBOT  —  EtherCAT bus  ·  25 MyActuator RMD X V4 drives  ·  sensors",RED[1])
igh=box(LX+16,Gy+38,W-32,42,"EtherLab IgH master · /dev/EtherCAT0 → single EtherCAT ring (bus pos 0–24)",
   "DC distributed clock SYNC0/1 · 100 Hz (CSP/CST) / 1 kHz (PVT) · state machine PREOP → SAFEOP → OP · single domain process-image",
   c=RED,tsize=13,ssize=9)
MX={"X4":BLUE,"X6":ORANGE,"X8":RED}
joints=[("00 L_shoulder_pitch","X6"),("01 L_shoulder_roll","X6"),("02 L_shoulder_yaw","X4"),
 ("03 L_elbow_pitch","X6"),("04 L_wrist_yaw","X4"),("05 L_wrist_roll","X4"),
 ("06 R_shoulder_pitch","X6"),("07 R_shoulder_roll","X6"),("08 R_shoulder_yaw","X4"),
 ("09 R_elbow_pitch","X6"),("10 R_wrist_yaw","X4"),("11 R_wrist_roll","X4"),
 ("12 waist_yaw","X8"),
 ("13 L_hip_pitch","X8"),("14 L_hip_roll","X8"),("15 L_hip_yaw","X8"),("16 L_knee","X8"),
 ("17 L_ankle_pitch","X4"),("18 L_ankle_roll","X4"),
 ("19 R_hip_pitch","X8"),("20 R_hip_roll","X8"),("21 R_hip_yaw","X8"),("22 R_knee","X8"),
 ("23 R_ankle_roll","X4"),("24 R_ankle_pitch","X4")]
jx0=LX+16; jy=Gy+92; jcw=243; jch=28; jgx=8; jgy=8; jcols=13
joint_ids=[]
for k,(jn,mt) in enumerate(joints):
    r,c=divmod(k,jcols); x=jx0+c*(jcw+jgx); y=jy+r*(jch+jgy)
    joint_ids.append(chip(x,y,jcw,jch,f"{jn} ·{mt}",c=MX[mt],fs=9))
py=jy+2*(jch+jgy)+12
box(LX+16,py,1040,160,"MyActuator RMD X V4 motor families",
 "X4 (blue) 36:1 · Kt 1.9 · I_rated 8.625 A — wrists/ankles/yaw (10 drives)\n"
 "X6 (orange) 19.612:1 · Kt 2.1 · I_rated 13.40 A — shoulder/elbow (6 drives)\n"
 "X8 (red) 19.612:1 · Kt 2.4 · I_rated 24.88 A — waist/hip/knee (9 drives)\n"
 "encoder ±65535 = ±180° → π/65535 rad/count · factor_cmd = 1000/(Kt·I_rated)\n"
 "modes CSP/CSV/CST/PVT · max_torque SDO default 50%",
 c=GRAY,tsize=12,ssize=9)
box(LX+16+1056,py,900,160,"Differential ankle linkage  +  calibration",
 "eccentric-pushrod serial chain · 2×X4 per ankle (pitch+roll coupled)\n"
 "v1 analytic IK/FK · v2 calibrated cubic-primary (mirror-crank), analytic compare on /ankle_method_compare\n"
 "per-joint zero-offset (raw cnt) + direction(±1, negate factors)\n"
 "e.g. L_sh_pitch offset 2704 dir−1 · L_sh_yaw 28044 · recalib: demo_joint_offset",
 c=GRAY,tsize=12,ssize=9)
box(LX+16+1972,py,W-32-1972,160,"On-board sensors",
 "BNO055 IMU ×2 → /pelvis/imu · /torso/imu (+ gravity vectors)\n"
 "Orbbec Gemini 335L/336L RGB-D camera\n"
 "drive telemetry: motor temp · drive temp · bus voltage · error code (TxPDO 0x1A02)\n"
 "absolute encoders, zero-offset calibrated per joint\n"
 "warn/err: motor 70/90°C · drive 70/85°C · 48 V rail 40–54 V",
 c=GRAY,tsize=12,ssize=9)

# ============================================================ FOOTNOTE
fy=Gy+Gh+14
box(LX,fy,W,72,"BRINGUP entry-points + remaining packages  (all 48 src packages represented) · startup ordering",
 "arm_system_bringup → headless.launch (sim)    arm_real_bringup → arm_real / position_viewer / recording / safety (CSP·CST)    robot_bringup → robot_pvt / robot_pvt_viewer / remote_view (PVT mode 5)\n"
 "startup chain: ros2_control_node(RT) → joint_state_broadcaster → [filtered + drive_status] → mode_controller → trajectory/effort controllers → homing → safety monitor → joint_state_publisher → RViz   (PVT controller spawned INACTIVE, operator-armed)\n"
 "legacy / stub: gripper_control · hand_control · arm_hardware · gripper_hardware · hand_hardware        meta: ethercat_driver_ros2",
 c=GRAY,tsize=12,ssize=9)

# ============================================================ EDGES
G="#7a7a7a"
# UI <-> DDS
edge(dds,viz_ids[0],"/joint_states · /tf · /planning_scene",color=BLUE[1],exitX=0.1,exitY=0,entryX=0.5,entryY=1)
edge(tl_ids[0],dds,"teleop JointState / cmd_vel",color=BLUE[1],exitX=0.5,exitY=0,entryX=0.06,entryY=1)
edge(gui_ids[1],dds,"FollowJointTrajectory",color=BLUE[1],dashed=1,exitX=0.5,exitY=0,entryX=0.16,entryY=1)
# Planning/RL <-> DDS
edge(mv_ids[0],dds,"/move_group · FollowJointTrajectory",color=ORANGE[1],width=2,exitX=0.4,exitY=1,entryX=0.42,entryY=0)
edge(dds,mv_ids[10],"/planning_scene · /joint_states",color=ORANGE[1],exitX=0.5,exitY=0,entryX=0.5,entryY=1)
edge(rl_ids[4],dds,"deployed as controller",color=PURPLE[1],dashed=1,exitX=0.5,exitY=1,entryX=0.72,entryY=0)
# DDS <-> Control
edge(dds,cm,"command topics · actions · services",color=ORANGE[1],width=2.4,exitX=0.5,exitY=1,entryX=0.5,entryY=0)
edge(bc_ids[1],dds,"state broadcasters ⟶ DDS",color=GREEN[1],width=2,exitX=0.4,exitY=0,entryX=0.34,entryY=1)
# Control -> model ; model -> sim/real
edge(cm,rm_ids[10],"robot_description (param)",color=PURPLE[1],dashed=1,exitX=0.6,exitY=1,entryX=0.5,entryY=0)
edge(rm_ids[10],sim_ids[0],"use_sim:=true",color=TEAL[1],width=2,exitX=0.25,exitY=1,entryX=0.5,entryY=0)
edge(rm_ids[10],real_ids[0],"use_sim:=false",color=RED[1],width=2,exitX=0.78,exitY=1,entryX=0.5,entryY=0)
# Control => hardware (in-process)
edge(cm,sim_ids[0],"SystemInterface ⟹ in-process",color=TEAL[1],width=2.4,exitX=0.18,exitY=1,entryX=0.5,entryY=0,extra="endArrow=blockThin;startArrow=blockThin;startFill=1;")
edge(cm,real_ids[0],"SystemInterface ⟹ in-process",color=RED[1],width=2.4,exitX=0.86,exitY=1,entryX=0.5,entryY=0,extra="endArrow=blockThin;startArrow=blockThin;startFill=1;")
# inside sim
edge(sim_ids[0],simp_ids[0],"gz plugin",color=TEAL[1],exitX=0.5,exitY=1,entryX=0.5,entryY=0)
# inside real: driver -> master -> bus -> joints
edge(real_ids[0],real_ids[4],"EtherCAT API",color=RED[1],exitX=0.4,exitY=1,entryX=0.5,entryY=0)
edge(real_ids[7],igh,"ecrt.h ⟶ master",color=RED[1],width=2,exitX=0.5,exitY=1,entryX=0.45,entryY=0)
edge(igh,joint_ids[0],"PDO @ 1 kHz · DC SYNC",color=RED[1],width=2,exitX=0.2,exitY=1,entryX=0.5,entryY=0)
# feedback
edge(joint_ids[24],cm,"IMU · encoders · drive telemetry ⟶ state",color=G,dashed=1,width=1.4,exitX=1,exitY=0.5,entryX=1,entryY=0.5)

xml_head=f'''<mxfile host="app.diagrams.net" version="24.0.0">
  <diagram id="harambe-arch" name="System Architecture">
    <mxGraphModel dx="1422" dy="800" grid="0" gridSize="10" guides="1" tooltips="1" connect="1" arrows="1"
        fold="1" page="1" pageScale="1" pageWidth="3370" pageHeight="2384" math="0" shadow="0" background="{BG}">
      <root>
        <mxCell id="0"/>
        <mxCell id="1" parent="0"/>
'''
out="/home/andrei-dragomir/Documents/GitHub/ldr-harambe-arms-sim2real/docs/system_architecture.drawio"
body="\n".join(cells)
open(out,"w").write(xml_head+body+"\n      </root>\n    </mxGraphModel>\n  </diagram>\n</mxfile>")
print("wrote",out,"cells:",len(cells))
