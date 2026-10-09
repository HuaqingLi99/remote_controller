# remote_controller

## Project Attribution

This project was developed by **Huaqing Li (go98niv)** as part of his
**Ingenieurpraxis**, under the supervision of **Yangsong Wu**.

## License

The original code and documentation authored by Huaqing Li are licensed under
the [MIT License](LICENSE).

Third-party materials retain their own licenses. The bundled Panda robot
model assets are excluded from this project's MIT license grant; see
[Third-party notices](THIRD_PARTY_NOTICES.md) for the verified upstream
attributions and remaining provenance details.

## Overview

`remote_controller` is a small remote-control framework for a Franka Panda setup.
It provides:

- a C++ XML-RPC server for arm and gripper commands
- a UDP state stream for q, dq, force, arm state, and gripper state feedback
- an installable Python package named `remote_controller`
- Pinocchio-based kinematics helpers using the packaged Panda URDF/assets

The current Python source layout is:

```text
remote_controller/
├── pyproject.toml
├── requirements.txt
├── run_server.sh
├── config/
│   └── remote_controller.env
├── src/
│   └── remote_controller/
│       ├── RemoteControllerClient.py
│       ├── robot_kinematics.py
│       ├── udp_receiver.py
│       └── assets/
├── examples/
└── cpp_server/
```

## Safety

This project sends real robot commands. Start with small motions, low speed, and
clear workspace. Keep the Franka desk / user stop reachable. The examples are
for controlled development, not unattended operation.

All Cartesian distances are in **meters**. For example:

```text
0.03 m = 3 cm
0.10 m = 10 cm
```

## Quick Start

### 1. Clone And Enter The Repo

```bash
git clone https://github.com/LiHuaqing-tum/remote_controller.git
cd remote_controller
```

### 2. Create A Python Environment

On Ubuntu, install `venv` first if needed:

```bash
sudo apt update
sudo apt install python3.10-venv
```

Create and activate the environment:

```bash
python3 -m venv .venv
source .venv/bin/activate
```

Install the Python package in editable mode:

```bash
python -m pip install -U pip setuptools wheel
python -m pip install -e .
```

After this, examples can import:

```python
from remote_controller import RemoteControllerClient, RobotModel
```

If you open a new terminal later, activate the same environment again:

```bash
source /path/to/remote_controller/.venv/bin/activate
```

### 3. Build The C++ Server

The server depends on libfranka, Eigen3, and xmlrpc-c.

Build from the repo root:

```bash
cd cpp_server
cmake -S . -B build
cmake --build build -j
```

The binary should be:

```text
cpp_server/build/remote_controller_server
```

### 4. Configure Robot IP And Port

Create your local config from the example:

```bash
cp config/remote_controller.example.env config/remote_controller.env
```

Then edit:

```text
config/remote_controller.env
```

Example:

```bash
ROBOT_IP=192.168.3.100
GRIPPER_IP=192.168.3.100
XMLRPC_PORT=8008

SERVER_BINARY=cpp_server/build/remote_controller_server
```

`config/remote_controller.env` is local machine configuration and is ignored by
git. Commit changes to `config/remote_controller.example.env` only when you want
to update the shared template.

### 5. Start The Server

From the repo root:

```bash
./run_server.sh
```

This reads `config/remote_controller.env` and runs:

```bash
sudo cpp_server/build/remote_controller_server ROBOT_IP GRIPPER_IP XMLRPC_PORT
```

You can also start it manually:

```bash
cd cpp_server/build
sudo ./remote_controller_server 192.168.3.100 192.168.3.100 8008
```

### 6. Run Examples

In another terminal:

```bash
cd /path/to/remote_controller/examples
source /path/to/remote_controller/.venv/bin/activate
python test_kinematics.py
```

Common examples:

```bash
python test_moveJ_path.py
python test_moveC_path.py
python test_moveC_relative.py
python test_stop_motion.py
python test_moveJ_no_queue_denied.py
python test_moveC_no_queue_denied.py
python test_udp_idle.py
python test_udp_moving.py
```

## Documentation

Build the Sphinx documentation:

```bash
python -m pip install -e ".[docs]"
sphinx-build -M html docs/source docs/build
```

Open:

```text
docs/build/html/index.html
```

## Python Usage

Minimal client setup:

```python
from remote_controller import RemoteControllerClient, RobotModel

SERVER_URL = "http://localhost:8008/RPC2"

client = RemoteControllerClient(
    SERVER_URL,
    capacity=16,
    horizon_prev=7,
    sensor_size=22,
    default_frame_name="panda_hand_tcp",
)

result = client.init(
    udp_ip="127.0.0.1",
    udp_port=9000,
    udp_frequency_hz=500,
    recover_before_init=True,
)
client.print_rpc_result(result, "init")
```

`udp_frequency_hz` is a target streaming frequency. During robot motion, the
measured UDP rate can be slightly lower than the requested value because the
server is also running the robot control loop and UDP is best-effort. This is
normal as long as the rate is close enough for your application and packets are
not repeatedly expired.

For VLA or policy inference, keep the UDP `capacity` small, for example `16`,
and use `get_padded_data()` to read a fixed-size history window from the
`DataBuffer`. The UDP receiver keeps state in this rolling `DataBuffer`; latest
state reads take the newest frame from the same buffer. Use a large capacity
such as `30000` only for examples that record many UDP frames for plotting or
offline analysis.

Get the latest UDP state:

```python
state = client.wait_for_first_udp(timeout=2.0)
q = state["q"]
dq = state["dq"]
arm_state = state["arm_state"]
gripper_state = state["gripper_state"]
```

Get the current TCP pose from UDP q and the packaged URDF:

```python
robot_model = RobotModel()
T_tcp = client.get_current_tcp_pose(robot_model, flush=True)
```

The default TCP frame is:

```text
panda_hand_tcp
```

You can override it when constructing the client:

```python
client = RemoteControllerClient(SERVER_URL, default_frame_name="panda_link8")
```

## Command Semantics

The Python client exposes unified helpers that choose single-target motion vs
path motion from the shape of the input.

### moveJ

Single joint target:

```python
result = client.movej(
    q_goal,
    stiffness=joint_stiffness,
    dq_max=[0.2] * 7,
    ddq_max=[0.5] * 7,
    queue=False,
)
```

Joint path:

```python
client.set_slowdown_factor(1.0)

result = client.movej(
    waypoints_q,
    stiffness=joint_stiffness,
    dq_max=[0.3] * 7,
    ddq_max=[1.0] * 7,
    queue=True,
    samples_per_segment=100,
    path_mode="catmull_rom",
)
```

Supported path modes depend on the server build. Current examples use:

```text
linear
catmull_rom
```

### moveC / Cartesian

Single absolute pose target:

```python
T_goal_flat = T_goal.reshape(-1).tolist()

result = client.movecart(
    T_goal_flat,
    stiffness=cartesian_stiffness,
    vmax_linear=0.03,
    acc_max_linear=0.05,
    vmax_angular=0.10,
    acc_max_angular=0.20,
    nullspace_stiffness=0.0,
    queue=False,
)
```

Cartesian path:

```python
client.set_slowdown_factor(1.0)

result = client.movecart(
    waypoints_T_flat,
    stiffness=cartesian_stiffness,
    vmax_linear=0.04,
    vmax_angular=0.15,
    nullspace_stiffness=0.0,
    queue=True,
    samples_per_segment=500,
    path_mode="linear",
)
```

`client.set_slowdown_factor(1.0)` preserves the original timing for `moveJ`,
`moveCartesian`, `moveJPath`, and `moveCartesianPath`. A value like `10.0`
advances motion references about 10 times slower and can be called while an arm
motion is running.

For Cartesian path replay, `acc_max_linear` and `acc_max_angular` are accepted
for API compatibility but are not used by the current server implementation.
`moveCartesianPath` checks the implied segment-to-segment velocity against
`vmax_linear` and `vmax_angular`; timing is mainly determined by
`samples_per_segment`, the runtime motion slowdown factor, waypoint spacing, and
the 1 kHz base replay period.

For online changes from the same Python process, enqueue the motion so the RPC
call returns, then update the shared slowdown value:

```python
client.movecart(waypoints_T_flat, ..., queue=True)
time.sleep(1.0)
client.set_slowdown_factor(10.0)
```

`moveCartesian` targets are absolute poses, not relative transforms. Use
`movecart_relative(...)` if you want to move relative to the current TCP pose:

```python
result = client.movecart_relative(
    robot_model,
    dz=0.03,
    vmax=0.03,
    acc_max=0.05,
    queue=False,
    reference_frame="tool",
)
```

## Queue vs No Queue

Most arm and gripper commands have both direct and queued versions.

`queue=False`:

- executes directly in the RPC call path
- should return `DENIED_MOVING` if the arm is already moving
- useful when you want synchronous behavior

`queue=True`:

- submits a task to a worker queue
- returns after the command is accepted
- useful for asynchronous command sequences

The stop test uses:

```python
client.stop_arm_motion()
```

It is intended to stop the current arm motion and clear queued arm commands.
After stopping, a new command can be sent.

## Return Codes

The client decodes common server return codes:

```text
 0  OK
-2  DENIED_MOVING
-3  DENIED_ERROR
-4  EXECUTION_FAILED
```

Example:

```python
result = client.movej(...)
client.print_rpc_result(result, "moveJ")
```

## UDP Frequency And Idle Polling

`client.init(..., udp_frequency_hz=500)` tells the server the desired UDP state
stream frequency. In practice, the measured rate can be a little lower than the
target, especially while the robot is moving.

When the robot is idle, the server may publish state more slowly depending on
the idle polling setting. If your application needs fresher idle-state data, use:

```python
client.set_idle_state_poll_frequency(100)
```

Higher idle polling gives faster state updates while idle, but it also increases
server CPU/network load. For motion tests, always check the reported packet count
and measured Hz instead of assuming the exact requested frequency was achieved.

## State

UDP state packets contain 20 doubles plus 2 integer state ids:

```text
q[0:7]
dq[0:7]
K_F_ext_hat[0:6]
arm_state
gripper_state
```

Client helpers:

```python
state, info = client.get_latest_state(allow_stale=True)
data, meta = client.get_padded_data(allow_stale=True)
obs = client.get_observation()
obs_kin = client.get_observation_with_kinematics(robot_model)
```

`get_observation()` and `get_observation_with_kinematics()` use one latest UDP
frame for q, dq, force, arm state, and gripper state, so the observation fields
belong to the same received state sample. Use the RPC helpers below when you need
reliable command-state confirmation.

Both `get_latest_state()` and `get_padded_data()` read from the same UDP
`DataBuffer`:

```text
get_latest_state() -> newest frame in DataBuffer
get_padded_data()  -> last horizon_prev frames in DataBuffer, padded if needed
```

For VLA/policy inference, prefer:

```python
data, meta = client.get_padded_data(allow_stale=False)
```

`data` is returned as one batch item:

```text
[[frame_1, frame_2, ..., frame_horizon]]
```

with length `horizon_prev * sensor_size`. With the default
`horizon_prev=7` and `sensor_size=22`, that is `154` values. If there are not
enough frames yet, the buffer pads the oldest available frame so the shape stays
stable. `get_latest_state()` is still useful for simple controllers or quick
checks that only need the newest UDP frame, including `arm_state` and
`gripper_state`. VLA inference should usually use the padded history window.

RPC state helpers:

```python
arm_state = client.get_arm_state()
gripper_state = client.get_gripper_state()
```

Possible arm states:

```text
IDLE
MOVING
ERROR
```

Possible gripper states:

```text
IDLE
MOVING
ERROR
WIDTH_TOO_LARGE
HOLDING
OPEN_FAILED
```

Wait helpers:

```python
client.wait_until_arm_moving_finished(timeout=30.0)
client.wait_until_arm_idle_ok(timeout=30.0)
client.wait_until_gripper_moving_finished(timeout=30.0)
client.wait_until_motion_done(timeout=30.0)
```

## Kinematics And Assets

`RobotModel()` uses the packaged URDF by default:

```text
src/remote_controller/assets/panda/panda_arm.urdf
```

The URDF includes `panda_hand_tcp`, and mesh paths are packaged under:

```text
src/remote_controller/assets/meshes/
```

Basic usage:

```python
from remote_controller import RobotModel

robot_model = RobotModel()
print("panda_hand_tcp" in robot_model.list_frames())

pose = robot_model.get_frame_pose(q, frame_name="panda_hand_tcp")
T = pose["T"]
```

Higher-level client helper:

```python
T = client.get_tcp_pose_from_q(robot_model, q)
terms = client.get_all_basic_terms(robot_model, q, dq)
```

## Example Guide

### `test_kinematics.py`

Checks that the packaged URDF and `panda_hand_tcp` frame are usable. It can also
send the current FK pose back through `moveCartesian` and compare q before/after.

### `test_moveC_path.py`

Moves to a fixed Cartesian start pose, then runs a Cartesian path using
`moveC_path`.

### `test_stop_motion.py`

Queues multiple Cartesian path commands, sends `stopArmMotion` while the first
path is active, checks that the path stopped before its final target, then sends
a new recovery path.

### `test_moveJ_no_queue_denied.py`

Starts one no-queue `moveJ` in the background, then sends a second no-queue
`moveJ` while the arm is moving. The expected result is:

```text
-2 DENIED_MOVING
```

### `test_moveC_no_queue_denied.py`

Same idea as above, but for single-target no-queue Cartesian `moveC`.

## C++ Server Internals

Main files:

```text
cpp_server/src/main.cpp
cpp_server/src/robot_manager.cpp
cpp_server/src/rpc_methods.cpp
cpp_server/src/moveJ.cpp
cpp_server/src/moveJ_path.cpp
cpp_server/src/moveC.cpp
cpp_server/src/moveC_path.cpp
cpp_server/src/udp_state_publisher.cpp
```

Startup arguments:

```text
remote_controller_server <robot_ip> <gripper_ip> <xmlrpc_port>
```

Important RPC methods include:

```text
initSession
moveJ
moveJ_queue
moveJPath
moveJPath_queue
moveCartesian
moveCartesian_queue
moveCartesianPath
moveCartesianPath_queue
stopArmMotion
setSlowdownFactor
getSlowdownFactor
getArmState
getGripperState
recoverSystem
setIdleStatePollFrequency
graspO
graspO_queue
gripperHome
gripperHome_queue
gripperRelease
gripperRelease_queue
```

## Development Notes

After changing Python files under `src/remote_controller`, editable install
uses the new code immediately:

```bash
python -m pip install -e .
```

Lightweight checks:

```bash
python -m py_compile src/remote_controller/*.py examples/*.py
python -c "from remote_controller import RemoteControllerClient, RobotModel; print('ok')"
```

If `python` is not available on your system, use `python3`, or activate the
project virtual environment:

```bash
source .venv/bin/activate
```
