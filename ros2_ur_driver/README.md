# ROS2 ↔ UR10 (PolyScope X) Driver Integration

`ros-humble-ur-robot-driver` against the PolyScope X (URSim) simulator,
Docker Desktop + WSL2. One real blocker below (`reverse_ip`); Remote
Control mode is not one, despite what this file used to say.

## What's here

- `config/ur10_calibration.yaml` — calibration extracted from this simulator
  via `ur_calibration`'s `calibration_correction.launch.py`. Pass it to
  `ur_robot_driver` launches as `kinematics_params_file` so the driver's
  kinematics match the sim exactly.
- `ros2_client.py` — the bridge that lets `../case1/server.py`'s MCP tools go
  through this driver instead of talking to the controller over raw sockets
  (`UR_BACKEND=ros2` in Case 1's server; see its README for the full picture).
  Same `RobotState` shape and method names as `case1/ur_client.py`, so every
  tool works unchanged either way.

## RMW note (sandboxed / restricted-network environments)

If plain `ros2` commands hang (`ros2 topic list`, `ros2 daemon status`, even
with the driver not involved), the default RMW (FastRTPS) may be stuck on
multicast discovery rather than actually broken. Confirmed fix in this
environment:
```bash
sudo apt install -y ros-humble-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```
Set that env var for **every** `ros2`/`rclpy` process you launch (the driver
included) — mixing RMWs between processes on the same topics is not reliable.
Also useful once it's working: `ros2 <cmd> --no-daemon` sidesteps a stuck
background daemon that was already started under the wrong RMW.

## The one real blocker (WSL2 + Docker Desktop specific)

Remote Control / Operational Mode (Settings, `easybot` / `operator`)
doesn't need touching -- verified with Operational Mode on `Manual`
(Remote Control forced to `Local`): driver still logged `Robot connected
to reverse interface`, `move_joint` landed exactly on target, same for the
socket backend. This-simulator-verified, not a general PolyScope X claim.

1. **`reverse_ip` must not be `127.0.0.1`.** Sim's in a container, driver's
   on the WSL2 host -- auto-detect resolves to the container itself, not
   the host, so the reverse/trajectory/script-command sockets never
   connect (PolyScope X: `Trajectory/Script command/Reverse socket
   connected: False`). Fix: `reverse_ip:=host.docker.internal` (compose
   file already maps it via `extra_hosts`).

Full command sequence is in the top-level [`README.md`](../README.md). With
the RMW note above folded in:
```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch ur_robot_driver ur10.launch.py \
  robot_ip:=127.0.0.1 \
  reverse_ip:=host.docker.internal \
  kinematics_params_file:="$(pwd)/config/ur10_calibration.yaml" \
  launch_rviz:=false \
  headless_mode:=true
```
Wait for `Robot connected to reverse interface. Ready to receive control
commands.` before pointing `ros2_client.py` (or `case1/server.py` with
`UR_BACKEND=ros2`) at it.

## If a trajectory goal gets rejected ("Controller is not running")

Something (e.g. `case1/ur_client.py` socket backend) knocked out External
Control, or it dropped on its own after idle. Either way:
```bash
ros2 service call /io_and_status_controller/resend_robot_program std_srvs/srv/Trigger "{}"
```
Wait for `Robot connected to reverse interface.` before retrying.
