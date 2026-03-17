# Distributed Setup: Jetson Nano + PC/NUC

Reference guide for running the real-time EtherCAT control loop on a Jetson Nano with high-level planning on a separate PC (later Intel NUC).

## Architecture Overview

```
┌───────────────────────────────────┐      USB gadget ethernet       ┌──────────────────────────────┐
│          JETSON NANO              │     (usb0 ↔ usb0/enx*)        │       PC / Intel NUC         │
│     (RT kernel, PREEMPT_RT)       │◄──────────────────────────────►│     (non-RT, Ubuntu 24.04)   │
│                                   │     ROS 2 DDS (Cyclone)        │                              │
│  Real-time nodes:                 │                                │  Non-RT nodes:               │
│  ┌─────────────────────────────┐  │  Topics crossing boundary:     │  ┌──────────────────────┐   │
│  │ ros2_control_node           │  │   /joint_states                │  │ MoveIt 2 move_group  │   │
│  │   (SCHED_FIFO 49, 100 Hz)  │  │   /tf, /tf_static             │  │ RViz2                │   │
│  │   EthercatDriver plugin     │  │   /robot_description          │  │ GUI launcher         │   │
│  │   25 slaves, 10 ms cycle    │  │   /*_controller/commands      │  │ Teleop nodes         │   │
│  ├─────────────────────────────┤  │   /*_controller/state         │  │ Application nodes    │   │
│  │ Joint State Broadcaster     │  │   /follow_joint_trajectory    │  └──────────────────────┘   │
│  │ JTC controllers (position)  │  │   /safety/status              │                              │
│  │ Effort controllers (torque) │  │   /diagnostics                │                              │
│  │ Mode controller             │  │                                │                              │
│  ├─────────────────────────────┤  │                                │                              │
│  │ Safety monitor (C++)        │  │                                │                              │
│  │ Gravity comp node (Python)  │  │                                │                              │
│  │ Robot State Publisher       │  │                                │                              │
│  └─────────────────────────────┘  │                                │                              │
│           │                       │                                │                              │
│      eth0 (EtherCAT)             │                                │                              │
│           │                       │                                │                              │
│   ┌───────┴───────┐              │                                │                              │
│   │ IgH Master    │              │                                │                              │
│   │ /dev/EtherCAT0│              │                                │                              │
│   └───────┬───────┘              │                                │                              │
└───────────┼───────────────────────┘                                └──────────────────────────────┘
            │
     EtherCAT bus (100 Mbps)
     ┌──┬──┬──┬──┬──┐
     S0 S1 S2 ... S24
     (25 MyActuator RMD X V4 slaves)
```

## Hardware Requirements

### Jetson Nano
- **RT kernel**: PREEMPT_RT patched (required for deterministic EtherCAT timing)
- **Ethernet port (eth0)**: Dedicated to EtherCAT bus — do NOT use for networking
- **USB port**: USB gadget mode for ROS 2 DDS communication with PC
- **RAM**: 4 GB sufficient for RT nodes (ros2_control + controllers + safety ≈ 500 MB)
- **Storage**: 32 GB+ microSD or NVMe (for ROS 2 Jazzy + workspace)

### PC / Intel NUC
- **OS**: Ubuntu 24.04 (ROS 2 Jazzy)
- **RAM**: 8 GB+ (MoveIt 2 + RViz can use 2-4 GB)
- **USB port**: For connection to Jetson
- **No RT kernel needed** — only runs non-RT planning/visualization nodes

## Step 1: USB Gadget Ethernet (Jetson ↔ PC)

The Jetson Nano supports USB Device Mode, appearing as a virtual Ethernet adapter to the connected PC.

### On the Jetson Nano

```bash
# Check if USB gadget ethernet is already configured (L4T default)
ip link show usb0

# If not, enable USB gadget ethernet:
# Edit /opt/nvidia/l4t-usb-device-mode/nv-l4t-usb-device-mode.sh
# or create a systemd service

# Assign static IP to USB gadget interface
sudo nmcli connection add type ethernet con-name jetson-usb ifname usb0 \
    ipv4.addresses 192.168.55.1/24 \
    ipv4.method manual \
    connection.autoconnect yes

# Verify
ip addr show usb0
# Should show: 192.168.55.1/24
```

### On the PC / NUC

```bash
# When Jetson is connected via USB, a new interface appears (usually enx* or usb0)
# Find it:
ip link show | grep -E "usb|enx"

# Assign static IP on the same subnet
IFACE="enx<mac>"  # Replace with actual interface name
sudo nmcli connection add type ethernet con-name jetson-link ifname $IFACE \
    ipv4.addresses 192.168.55.100/24 \
    ipv4.method manual \
    connection.autoconnect yes

# Test connectivity
ping 192.168.55.1   # Should reach Jetson
```

### Verify Bidirectional Connectivity

```bash
# From PC:
ping 192.168.55.1    # Jetson

# From Jetson:
ping 192.168.55.100  # PC
```

## Step 2: ROS 2 DDS Configuration

Both machines must use the same DDS middleware and domain ID, and discover each other over the USB gadget link.

### DDS Middleware: Cyclone DDS (recommended)

Cyclone DDS has better performance for distributed setups than Fast DDS.

```bash
# Install on BOTH machines
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
```

### Cyclone DDS Config (both machines)

Create `~/cyclonedds.xml`:

#### On the Jetson (192.168.55.1)

```xml
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="usb0" priority="default" multicast="false"/>
      </Interfaces>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <Peers>
        <Peer address="192.168.55.100"/>
      </Peers>
      <ParticipantIndex>auto</ParticipantIndex>
    </Discovery>
  </Domain>
</CycloneDDS>
```

#### On the PC/NUC (192.168.55.100)

```xml
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="enx*" priority="default" multicast="false"/>
        <!-- Replace enx* with exact interface name, e.g., enxaabbccddee -->
      </Interfaces>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <Peers>
        <Peer address="192.168.55.1"/>
      </Peers>
      <ParticipantIndex>auto</ParticipantIndex>
    </Discovery>
  </Domain>
</CycloneDDS>
```

### Environment Variables (both machines)

Add to `~/.bashrc` on both:

```bash
# ROS 2 DDS config
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
export ROS_DOMAIN_ID=0  # Same on both machines

# Source ROS 2
source /opt/ros/jazzy/setup.bash
source ~/ldr-harambe-arms-sim2real/install/setup.bash
```

### Verify DDS Discovery

```bash
# Terminal 1 (Jetson): publish test topic
ros2 topic pub /test std_msgs/msg/String "{data: 'hello from jetson'}"

# Terminal 2 (PC): verify reception
ros2 topic echo /test

# Also check node discovery
ros2 node list  # Should show nodes from BOTH machines
```

## Step 3: Clock Synchronization

Accurate clock sync is critical for TF transforms and trajectory execution across machines.

### Option A: chrony (recommended, easiest)

```bash
# Install on both machines
sudo apt install chrony

# On Jetson — act as NTP server
# Edit /etc/chrony/chrony.conf, add:
local stratum 10
allow 192.168.55.0/24

# On PC — sync to Jetson
# Edit /etc/chrony/chrony.conf, add:
server 192.168.55.1 iburst prefer minpoll 0 maxpoll 2

# Restart chrony on both
sudo systemctl restart chrony

# Verify sync (on PC)
chronyc sources -v
# Should show 192.168.55.1 as source with low offset (<1 ms)
```

### Option B: PTP (sub-microsecond, hardware support needed)

Only if both machines have PTP-capable NICs (unlikely over USB gadget).

## Step 4: Split Launch Files

### Jetson Launch: `jetson_real.launch.py`

Runs all real-time and hardware-interface nodes. Based on existing `arm_real.launch.py` but WITHOUT RViz.

**Nodes to run on Jetson:**
- `robot_state_publisher` — publishes `/robot_description` and TF
- `ros2_control_node` — EtherCAT driver, 100 Hz RT loop
- `joint_state_broadcaster` — publishes `/joint_states`
- All controllers (JTC, effort, mode)
- `safety_monitor` — independent safety watchdog
- `gravity_comp_node` — if using CST/recording mode
- `homing_sequence` — initial homing after controller startup
- `joint_state_publisher` — fills in non-EtherCAT joints (hands)

**Key changes from `arm_real.launch.py`:**
- Remove `rviz2` node
- No MoveIt nodes
- Ensure `robot_state_publisher` uses `TRANSIENT_LOCAL` QoS for `/robot_description` so remote nodes can discover it late

### PC/NUC Launch: `pc_planning.launch.py`

Runs planning, visualization, and high-level control.

**Nodes to run on PC/NUC:**
- `rviz2` — visualization
- MoveIt 2 `move_group` node (when ready)
- GUI tools (`full_system_launcher`, `ethercat_monitor`, etc.)
- Teleop nodes
- Application nodes

**Key considerations:**
- MoveIt needs `/robot_description` — gets it from Jetson's `robot_state_publisher` via DDS
- MoveIt needs `/joint_states` — streamed from Jetson via DDS
- MoveIt sends trajectories — action client on PC, action server on Jetson (via `left_arm_group_controller`)
- All transparent over DDS — no code changes needed

### Minimal Example: PC-side Launch

```python
# pc_planning.launch.py (runs on PC/NUC)
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # RViz only — everything else comes from Jetson via DDS
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_path],
        ),
        # MoveIt move_group (when ready)
        # ...
    ])
```

## Step 5: Building on Jetson Nano

### Cross-compilation vs Native Build

**Native build (simpler, slower):**
```bash
# On the Jetson:
cd ~/ldr-harambe-arms-sim2real
colcon build --packages-select \
    arm_description dual_arm_description hand_description \
    arm_control dual_arm_control \
    arm_real_bringup \
    arm_ethercat_safety \
    ethercat_driver ethercat_interface ethercat_generic_plugins

source install/setup.bash
```

**Only build what runs on Jetson** — skip MoveIt, GUI tools, teleop, MTC.

### Packages for Jetson (RT node)

| Package | Why |
|---------|-----|
| `arm_description`, `dual_arm_description`, `hand_description` | URDF for robot_state_publisher |
| `arm_control`, `dual_arm_control` | Controller configs |
| `arm_real_bringup` | Launch files, EtherCAT configs, scripts |
| `arm_ethercat_safety` | Safety monitor node |
| `ethercat_driver`, `ethercat_interface`, `ethercat_generic_plugins` | EtherCAT framework |

### Packages for PC/NUC (non-RT node)

Everything else — MoveIt configs, GUI tools, teleop, MTC, Gazebo (sim).

## Step 6: IgH EtherCAT Master on Jetson

### Prerequisites

```bash
# RT kernel must be installed and booted
uname -a  # Should show PREEMPT_RT

# Install EtherLab IgH EtherCAT master
# Build from source (IgH 1.5.2 or 1.6-dev) — see docs/ETHERCAT.md

# Configure master
sudo nano /etc/sysconfig/ethercat
# Set:
MASTER0_DEVICE="<MAC_ADDRESS_OF_JETSON_ETH0>"
DEVICE_MODULES="generic"

# Load kernel module
sudo systemctl start ethercat
sudo systemctl enable ethercat

# Verify
ethercat master   # Should show master info
ethercat slaves   # Should list 25 slaves (when connected)

# Set permissions
sudo bash -c 'echo "KERNEL==\"EtherCAT[0-9]*\", MODE=\"0666\"" > /etc/udev/rules.d/99-ethercat.rules'
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### Verify EtherCAT on Jetson

```bash
# Pre-flight check
bash $(ros2 pkg prefix arm_real_bringup)/share/arm_real_bringup/scripts/check_ethercat.sh 25
```

## Step 7: RT Kernel Tuning on Jetson

```bash
# Set CPU governor to performance
for cpu in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
    echo performance | sudo tee $cpu
done

# Disable CPU frequency scaling
sudo jetson_clocks  # NVIDIA tool to max out clocks

# Isolate CPU core for ros2_control (optional, advanced)
# Add to kernel cmdline: isolcpus=3 nohz_full=3 rcu_nocbs=3
# Then run ros2_control_node pinned to core 3:
# taskset -c 3 chrt -f 49 ros2_control_node ...

# Check RT scheduling limits
ulimit -r  # Should be >= 49, or unlimited
# If not, add to /etc/security/limits.conf:
# <user>  -  rtprio  99
# <user>  -  memlock unlimited
```

## Step 8: Intel NUC Migration (Future)

When replacing the PC with an Intel NUC:

1. Install Ubuntu 24.04 + ROS 2 Jazzy
2. Clone workspace, build non-RT packages only
3. Copy `~/cyclonedds.xml` (same config, same IP)
4. Copy `~/.bashrc` environment variables
5. Connect USB to Jetson, assign same IP (192.168.55.100)
6. Test with `ros2 topic list` — should see Jetson topics
7. Launch PC-side nodes

**No changes needed on the Jetson side.**

## Troubleshooting

### DDS Discovery Issues
```bash
# Check if topics are visible
ros2 topic list  # Should show topics from both machines

# If not, verify:
# 1. Same ROS_DOMAIN_ID on both
# 2. Same RMW_IMPLEMENTATION on both
# 3. Firewall not blocking (sudo ufw disable, or allow ports 7400-7500)
# 4. Cyclone DDS config points to correct interfaces
# 5. Ping works between machines

# Debug DDS
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
ros2 daemon stop && ros2 daemon start
```

### EtherCAT Timing Issues on Jetson
```bash
# Check for RT scheduling
chrt -p $(pgrep ros2_control)  # Should show SCHED_FIFO, priority 49

# Check for CPU overload
top -H  # ros2_control should use < 10% CPU at 100 Hz

# Check EtherCAT timing jitter
ethercat master  # Look at "Timing" section
# Jitter should be < 100 µs for stable 10 ms cycle
```

### USB Gadget Link Drops
```bash
# If USB disconnects, DDS will lose discovery
# Re-establish:
sudo ifconfig usb0 192.168.55.1 up  # On Jetson
ros2 daemon stop && ros2 daemon start  # On both

# For reliability, add a systemd service to auto-configure usb0 on boot
```

## Performance Expectations

| Metric | Expected Value |
|--------|---------------|
| EtherCAT cycle time | 10 ms (100 Hz) |
| EtherCAT jitter | < 100 µs (with PREEMPT_RT) |
| DDS latency (USB gadget) | 1-5 ms typical |
| /joint_states frequency | 100 Hz |
| MoveIt planning time | 0.1-2s (on PC/NUC) |
| Trajectory execution | Real-time on Jetson, monitoring on PC |
| Total bandwidth (USB) | ~80 KB/s EtherCAT PDO + ~500 KB/s ROS topics |

## Network Topology Summary

```
Internet ─── PC/NUC WiFi/Ethernet (for development)
                │
           PC/NUC USB ──── Jetson USB (gadget ethernet, 192.168.55.0/24)
                                │
                           Jetson eth0 ──── EtherCAT bus (25 slaves)
```

Three isolated networks, no routing conflicts.
