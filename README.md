# Unitree G1 MuJoCo + ROS 2 simulation

Two ROS 2 packages:

- `unitree_g1_interfaces` (ament_cmake) — defines `JointCommand.msg`
  (`string[] joint_names`, `float64[] positions`), the message used to
  command joint positions into the simulation.
- `unitree_g1_mujoco` (ament_python) — the G1 MJCF/mesh files
  (`mjcf/unitree_g1/`), the `simulation` node, three exercise nodes
  (`exercise` = wave hello, `walk`, `crunches`), `gait.py` (the walking
  planner/IK library `walk.py` is built on), and launch files for all four.

## Setup

1. Copy the `src/` folder into your `~/ros2_ws/src/` (or unpack this
   archive directly there).
2. Make sure `mujoco` and `numpy` are installed in the environment your
   workspace uses:
   ```
   pip install mujoco numpy
   ```
3. Build and source:
   ```
   cd ~/ros2_ws
   colcon build --packages-select unitree_g1_interfaces unitree_g1_mujoco
   source install/setup.bash
   ```

## Run

Terminal 1 — start the simulation (opens the MuJoCo viewer). Which
arguments you pass depends on which exercise you're about to run:

```
# wave hello
ros2 launch unitree_g1_mujoco simulation.launch.py

# walk
ros2 launch unitree_g1_mujoco simulation.launch.py leg_kp_scale:=4.0 leg_kd_scale:=3.0

# crunches
ros2 launch unitree_g1_mujoco simulation.launch.py initial_pose:=supine
```

Terminal 2 — run the exercise:

```
ros2 launch unitree_g1_mujoco exercise.launch.py   # wave hello
ros2 launch unitree_g1_mujoco walk.launch.py       # walk
ros2 launch unitree_g1_mujoco crunches.launch.py   # crunches
```

## Topics

| Topic           | Type                                   | Direction |
|-----------------|-----------------------------------------|-----------|
| `joint_states`  | `sensor_msgs/JointState`                | published by `simulation` |
| `imu/pelvis`    | `sensor_msgs/Imu`                       | published by `simulation` |
| `imu/torso`     | `sensor_msgs/Imu`                       | published by `simulation` |
| `joint_command` | `unitree_g1_interfaces/JointCommand`    | subscribed by `simulation`, published by `exercise`/`walk`/`crunches` |

## The exercises, and their limits

**Wave hello** (`exercise.py`) — raises the right arm, waves the wrist,
lowers it. No balance concerns; the robot stands still throughout.

**Walk** (`walk.py`, `gait.py`) — an *open-loop* walk: footsteps and a
centre-of-mass trajectory are planned offline with a linear-inverted-
pendulum/DCM model, converted to leg angles via inverse kinematics, and
played back as a fixed trajectory. There is no balance feedback — nothing
in this pipeline watches the IMU or corrects for tracking error — so it
only works because the default leg gains are raised 4x/3x
(`leg_kp_scale`/`leg_kd_scale`) to keep tracking tight enough that the
plan's assumptions hold. **Only the default parameters (6 steps of 6 cm,
0.5 s/step) have been validated not to fall**, in headless MuJoCo. Larger
`step_length`/`step_time`/`n_steps` are likely to fall over; there's
nothing to catch it if they do. A robust walk would need a state estimator
(fusing `imu/pelvis` + leg kinematics into an actual CoM/velocity estimate)
and closed-loop foot placement — out of scope here.

**Crunches** (`crunches.py`) — the G1_23dof model has no torso/spine joint
(`waist_yaw_joint` only yaws, it doesn't pitch), so a sit-up-style torso
curl isn't physically possible with this model. What's implemented instead
is a **reverse crunch**: lying on the back (`initial_pose:=supine`),
cyclically pulling both knees toward the chest with the hip and knee
joints and extending back out — a real bodyweight ab exercise that works
the same muscles. There's no stabilization here either: in headless
testing the pelvis rocks roughly +/-0.15–0.35 m per cycle but stays on the
floor and doesn't flip, for the default amplitude/rep count in
`crunches.py`. Untested beyond those defaults.

## Swapping the exercise

`exercise.py` and `crunches.py` only command specific joints (right arm;
hip+knee); everything else holds its last commanded position. `walk.py`
commands all 12 leg joints throughout. To build a different move, publish
`JointCommand` for the relevant joint names/targets from a new node — all
23 joint names and their ranges are listed in
`mjcf/unitree_g1/mjcf/g1_23dof_rev_1_0_tcri.mjcf`.
