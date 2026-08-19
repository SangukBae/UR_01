# UR_01 — UR10 Robotics Summer School: Case 1 + ROS2 Driver Integration

UR Robotics Summer School 2026 (UR10, PolyScope X simulator): Case 1 MCP
server + ROS2 ↔ UR10 integration, both verified end-to-end against the sim.

- [`case1/`](case1/) — MCP server, Bronze → Diamond, socket + ROS2 backends.
- [`ros2_ur_driver/`](ros2_ur_driver/) — `ros-humble-ur-robot-driver` against
  the simulator.
- [`llm_client/`](llm_client/) — standalone chat wrapper (text + `--voice`),
  Cerebras by default, MCP tool-calling into `case1/server.py`.

## Environment

Windows + WSL2 (Ubuntu 22.04) + Docker Desktop (WSL2 integration enabled).
Commands below are run inside the WSL2 Ubuntu shell.

## 0. Get the simulator

The PolyScope X simulator compose file lives in the course repo (not
duplicated here):

```bash
git clone https://github.com/ureskr/international-summer-school-robotics-TER-UR.git
cd international-summer-school-robotics-TER-UR/"simulation environment"
```

## 1. Start the simulator

Docker Desktop must be running on Windows first (WSL2 integration on).

```bash
docker compose up -d          # ~40s first boot
docker ps                     # confirm essre-ursim-psx is Up
```

Open `http://localhost` in a browser:

1. Power the robot on and release the brakes until it reads **RUNNING**.
2. Remote Control / Operational Mode (Settings → Password, `easybot` /
   `operator`): neither needs touching. Verified both backends move the
   robot fine with Operational Mode on `Manual` (which forces Remote
   Control to `Local`) -- this-simulator-verified, not a general PolyScope
   X claim. Details in [`ros2_ur_driver/README.md`](ros2_ur_driver/README.md).

Verify state is readable (from `case1/`):

```bash
cd case1
pip install -r requirements.txt
python3 -c "from ur_client import URClient; r=URClient(); r.connect(); print(r.get_state())"
# robot_mode should be 7 (RUNNING)
```

## 2. Install ROS2 Humble

```bash
sudo apt update && sudo apt install -y software-properties-common curl gnupg lsb-release
sudo add-apt-repository universe -y

ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo apt install -y /tmp/ros2-apt-source.deb

sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools python3-colcon-common-extensions
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source /opt/ros/humble/setup.bash

sudo rosdep init
rosdep update
```

## 3. Install the UR ROS2 driver

```bash
sudo apt install -y ros-humble-ur-robot-driver ros-humble-ur-calibration ros-humble-ros2controlcli
```

## 4. Calibrate against this simulator

Kinematics must match the sim exactly or the driver refuses to start.

```bash
ros2 launch ur_calibration calibration_correction.launch.py \
  robot_ip:=127.0.0.1 \
  target_filename:="$(pwd)/ros2_ur_driver/config/ur10_calibration.yaml"
```

(Already generated once — `ros2_ur_driver/config/ur10_calibration.yaml` in
this repo is the output for this simulator image.)

## 5. Launch the driver

`reverse_ip` **must** be `host.docker.internal`, not auto-detected --
otherwise it resolves to the sim container itself instead of the WSL2
host, and the robot never connects.

```bash
ros2 launch ur_robot_driver ur10.launch.py \
  robot_ip:=127.0.0.1 \
  reverse_ip:=host.docker.internal \
  kinematics_params_file:="$(pwd)/ros2_ur_driver/config/ur10_calibration.yaml" \
  launch_rviz:=false \
  headless_mode:=true
```

Wait for `Robot connected to reverse interface. Ready to receive control
commands.` in the log, then in a second terminal (after `source
/opt/ros/humble/setup.bash`):

```bash
ros2 control list_controllers   # scaled_joint_trajectory_controller should be 'active'
ros2 topic echo /joint_states --once
```

## 6. Send a motion command

```bash
ros2 action send_goal /scaled_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [elbow_joint, shoulder_lift_joint, shoulder_pan_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint], points: [{positions: [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0], time_from_start: {sec: 4, nanosec: 0}}]}}"
```

Should return `Goal successfully reached!` and the robot visibly moves to
home in the PolyScope X web UI.

### If a goal gets stuck (no movement, no error)

Something else sent the robot a program (e.g. a raw script via
`ur_client.py`) and knocked out the driver's control script. Resend it:

```bash
ros2 service call /io_and_status_controller/resend_robot_program std_srvs/srv/Trigger "{}"
```

## Case 1 (MCP server)

```bash
cd case1
claude mcp add ur-tools -- python3 "$(pwd)/server.py"
claude mcp list
```

All four tiers (Bronze → Diamond) implemented and verified -- see
[`case1/README.md`](case1/README.md).
