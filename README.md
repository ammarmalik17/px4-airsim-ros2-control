# px4-airsim-ros2-control

Keyboard teleoperation of a PX4 SITL drone in Microsoft AirSim, driven from ROS 2
Humble over uXRCE-DDS. The node streams offboard velocity setpoints to PX4 at 20 Hz,
verifies that offboard mode was actually accepted, and runs an isolated camera
pipeline on its own AirSim RPC session.

This is a working simulation harness, not a flight-ready autopilot: there is no
path planning, mission logic or autonomy layer.

## Tested stack

| Component | Version / notes |
|---|---|
| Host | Windows 10/11 running AirSim + Unreal Engine (Blocks environment) |
| Guest | Ubuntu 22.04 on WSL 2 |
| ROS 2 | Humble |
| PX4 | v1.16 SITL, `make px4_sitl none_iris` |
| Bridge | Micro XRCE-DDS Agent v2.4.3, UDP port 8888 |
| Python | 3.10 |

The AirSim client reaches the Windows host over the WSL 2 NAT gateway
(`airsim.ip` in [config/params.yaml](src/airsim_offboard/config/params.yaml),
default `192.168.224.1`, API port 41451).

## Install

```bash
# 1. Upstream PX4 message packages (pinned by SHA)
vcs import src < dependencies.repos

# 2. ROS dependencies
rosdep install --from-paths src --ignore-src -y

# 3. Python dependencies
#    NOTE: the "airsim" client has no rosdep key and must come from pip.
python3 -m pip install -r requirements.txt
```

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run

Three terminals, in order.

**1. PX4 SITL** (from your PX4-Autopilot checkout):

```bash
make px4_sitl none_iris
```

Then apply the required PX4 parameters — **offboard will be rejected without them**.
See [px4_params.md](px4_params.md); in short, in the `pxh>` shell:

```
param set COM_OF_LOSS_T 300.0
param set CBRK_SUPPLY_CHK 894281
param set COM_LOW_BAT_ACT 0
param save
```

**2. XRCE-DDS agent:**

```bash
MicroXRCEAgent udp4 -p 8888
```

**3. The node:**

```bash
ros2 launch airsim_offboard offboard.launch.py
```

The launch file sets `emulate_tty=True` so the keyboard handler receives raw
stdin. If you run the node in a context without a TTY it silently degrades to a
line-based `input()` prompt.

Equivalent manual invocations:

```bash
ros2 run airsim_offboard offboard_control
ros2 launch airsim_offboard offboard.launch.py params_file:=/path/to/params.yaml
```

## Controls

| Key | Action |
|---|---|
| `W` / `S` | Forward / backward (horizontal, clipped at `velocity.max_horizontal`) |
| `A` / `D` | Left / right |
| `R` / `F` | Up / down (clipped at `velocity.max_vertical`) |
| `Q` / `E` | Yaw left / right by `yaw.step` radians |
| `O` | Toggle offboard mode (engage / return to AUTO LOITER) |
| `V` | Toggle the AirSim camera window |
| `T` | Takeoff to `control.takeoff_altitude` |
| `L` | Land |
| `X` | Zero all velocities |
| `ESC` | Exit |

Movement keys only act while offboard mode is engaged. Release a key and the
target velocity decays toward zero at `velocity.decay`.

**Wait for `control.offboard_min_setpoints` (default 10) setpoints before pressing
`O`** — PX4 rejects offboard entry until it has seen a stream of valid setpoints.

## ROS 2 interface

Publishers (QoS: `best_effort`, `transient_local`, depth 1 by default):

| Topic | Type |
|---|---|
| `/fmu/in/offboard_control_mode` | `px4_msgs/msg/OffboardControlMode` |
| `/fmu/in/trajectory_setpoint` | `px4_msgs/msg/TrajectorySetpoint` |
| `/fmu/in/vehicle_command` | `px4_msgs/msg/VehicleCommand` |

Subscriber:

| Topic | Type |
|---|---|
| `/fmu/out/vehicle_status_v1` | `px4_msgs/msg/VehicleStatus` |

The `_v1` suffix is mandatory on PX4 v1.16+: `VehicleStatus` carries
`MESSAGE_VERSION=1`, so the uORB topic is `vehicle_status_v1`. Subscribing to
`/fmu/out/vehicle_status` silently never fires.

All four topic names, the QoS policy strings, the AirSim endpoint and the
`VehicleCommand` target system/component are parameters, so a second vehicle can
be driven from an override YAML instead of a source edit.

## Parameters

Defaults live in [config/params.yaml](src/airsim_offboard/config/params.yaml).
Override them at launch (`params_file:=...`) or at runtime with `ros2 param set`.

| Group | Parameters |
|---|---|
| `airsim.*` | `ip`, `port`, `vehicle_name`, `camera` |
| `topic.*` | `offboard_control_mode`, `trajectory_setpoint`, `vehicle_command`, `vehicle_status` |
| `vehicle.*` | `target_system`, `target_component` |
| `control.*` | `publish_period_s`, `offboard_min_setpoints`, `takeoff_altitude` |
| `velocity.*` | `max_horizontal`, `max_vertical`, `step`, `decay` |
| `yaw.*` | `step` |
| `video.*` | `target_fps`, `queue_size`, `window_name` |
| `qos.*` | `reliability`, `durability`, `depth` |

`control.publish_period_s` drives both the timer and the velocity-decay step, so
changing the control rate cannot desynchronise deceleration from the publish rate.

**PX4-side parameters are not ROS parameters.** The failsafe-relevant ones
(`COM_OF_LOSS_T`, `CBRK_SUPPLY_CHK`, `COM_LOW_BAT_ACT`) live in PX4 and are
documented with the reasoning behind each value in [px4_params.md](px4_params.md).

## Tests

```bash
colcon test --packages-select airsim_offboard --python-testing pytest
colcon test-result --verbose          # -> 3 tests, 0 errors, 0 failures
```

Three unit tests cover the pure helpers in `airsim_offboard/velocity.py`
(decay-to-zero without sign flip, clamping at the horizontal/vertical limits,
yaw wrapping). They import only the standard library, so they also run standalone:

```bash
cd src/airsim_offboard && python3 -m pytest test -q
```

Use `--python-testing pytest` to get JUnit results. colcon defaults to the
`setuppy_test` runner (`python -m unittest`), which executes the same tests but
emits no XML, so `colcon test-result` reports zero for it; add
`--event-handlers console_direct+` if you want to read the raw unittest output.

## Troubleshooting

**Offboard mode is rejected (`nav_state` stays out of 14).**
PX4 samples setpoint freshness against its *simulated* clock while this node
stamps with the WSL wall clock; AirSim's Unreal loop runs well below real time,
so PX4 sees every setpoint as stale. Fix by measuring the drift and sizing
`COM_OF_LOSS_T` above it (the node's failure path prints the current values):

```
ros2 topic echo /fmu/out/timesync_status     # read observed_offset (microseconds)
param show COM_OF_LOSS_T                     # must exceed |observed_offset|
```

Full reasoning and the measurement procedure: [px4_params.md](px4_params.md).
Do not simply raise the value blindly — `COM_OF_LOSS_T` is also the delay before
PX4's real offboard-loss failsafe fires.

**The drone arms, takes off, then immediately lands.**
AirSim's battery model drains in simulated time and trips PX4's battery failsafe.
Apply `CBRK_SUPPLY_CHK 894281` and `COM_LOW_BAT_ACT 0` (plus the `BAT_*_THR`
zeros) as described in [px4_params.md](px4_params.md).

**Unreal renders slowly / the video feed stutters.**
AirSim on integrated GPUs needs help; see the rendering notes in
[px4_params.md](px4_params.md) and lower `video.target_fps`. Low render FPS also
accelerates the clock drift in the first item above.

**The camera window never appears.**
Press `V` after the node reports a successful AirSim connection; `start_video_feed`
refuses to open a second RPC session if the initial connection failed. See
[docs/camera_pipeline.md](docs/camera_pipeline.md) for the thread model.

## Repository layout

```
src/airsim_offboard/            # this package (ament_python)
  airsim_offboard/offboard_control.py   # node
  airsim_offboard/velocity.py           # pure, unit-tested flight math
  config/params.yaml
  launch/offboard.launch.py
  test/test_offboard_control.py
src/px4_msgs, src/px4_ros_com   # upstream, restored via dependencies.repos (gitignored)
docs/camera_pipeline.md         # camera thread model
px4_params.md                   # required PX4 parameters and why
dependencies.repos              # pinned upstream dependencies
```

## License

Apache-2.0. See [LICENSE](LICENSE).
