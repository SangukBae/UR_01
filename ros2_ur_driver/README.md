# ROS2 ↔ UR10 (PolyScope X) Driver Integration

Notes from getting `ros-humble-ur-robot-driver` talking to the PolyScope X
(URSim) simulator from `simulation environment/` in a Docker Desktop + WSL2
setup, including the two non-obvious blockers that stop the robot from
actually moving.

## What's here

- `config/ur10_calibration.yaml` — calibration extracted from this simulator
  via `ur_calibration`'s `calibration_correction.launch.py`. Pass it to
  `ur_robot_driver` launches as `kinematics_params_file` so the driver's
  kinematics match the sim exactly.

## The two blockers (WSL2 + Docker Desktop specific)

1. **Remote Control mode.** PolyScope X refuses external (ROS/script)
   control until the robot is switched from `Local` to `Remote` control in
   the web UI (Settings → password-protected panel), and Operational Mode is
   set to `Automatic`. Default passwords: Admin = `easybot`, Operational
   Mode = `operator` (you'll be forced to change these on first use).
   Symptom if skipped: motion commands are accepted but the robot never
   moves, with no error anywhere.

2. **`reverse_ip` must not be `127.0.0.1`.** The simulator runs inside a
   Docker container; the driver runs on the WSL2 host. If you launch with
   only `robot_ip:=127.0.0.1`, the driver auto-detects its own address as
   `127.0.0.1` too — which, from *inside the container*, means the
   container itself, not the host. The robot then can't connect back for
   the reverse/trajectory/script-command sockets, and PolyScope X shows:
   `Error connecting to remote: Trajectory socket(50003) connected: False /
   Script command socket(50005) connected: False / Reverse socket (50001)
   connected: False`.
   Fix: pass `reverse_ip:=host.docker.internal` explicitly (the compose
   file already maps that hostname via `extra_hosts: host.docker.internal:
   host-gateway`).

Full command sequence is in the top-level [`README.md`](../README.md).
