# Gazebo simulation of the robotic arm

Simulation of our six-axis colour-sorting arm (RAS Build Challenge 2026, Project 3). It supports Section 4 of the design book: motion and kinematics, clearance, torque and load, and power and control logic.

## What it simulates

- **The arm.** It uses the URDF exported from our SolidWorks assembly, cleaned up as follows:
  - joint limits added;
  - the arm stands upright on the table;
  - J4 and J5 renamed (J4 wrist pitch, J5 wrist roll);
  - light collision shapes added;
  - two simple gripper jaws added.

  The joints are:

  | Joint | Motion | Actuator |
  |---|---|---|
  | J1 | Base rotation | NEMA17 + 1:14 cycloidal gearbox |
  | J2 | Shoulder lift | NEMA17 + 1:14 cycloidal gearbox |
  | J3 | Elbow | NEMA17 + 1:14 cycloidal gearbox |
  | J4 | Wrist pitch | NEMA17 + 1:14 cycloidal gearbox |
  | J5 | Wrist roll | DS3218MG servo |
  | J6 | Tool rotation | DS3218MG servo |
  | Gripper | Rack and pinion | MG996R servo |

- **The sorting cell.** A table holds:
  - a pick stand, 185 mm high at 200 mm from the base axis;
  - two bins (red and blue), with the rim at 180 mm, 210 mm from the axis and 80° on either side;
  - 30 mm cubes of 50 g.
- **The sequence of the real arm.**
  1. Homing of J1 to J4 on virtual limit switches.
  2. For each cube: approach, descend, grasp, lift, classify, move to the bin, release and return.
  3. Inverse kinematics: numerical (damped least squares) on the URDF chain.
  4. Moves: synchronised trapezoidal profiles at 60 °/s.
- **Emergency stop.** The torque of J1 to J4 is removed, as the hardware does by disabling the drivers.
- **Logging and figures.** Joint positions, speeds and torques are logged, and one command turns them into the figures of the design book.

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Fortress

```bash
sudo apt install ros-humble-ros-gz ros-humble-ign-ros2-control ros-humble-ros2-control \
  ros-humble-ros2-controllers ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui ros-humble-rviz2 python3-colcon-common-extensions \
  python3-numpy python3-scipy python3-matplotlib python3-yaml python3-opencv
pip install pin    # optional: only for the offline analysis (static torque, offline dynamics)
```

## Build

```bash
cd simulation
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run

Every new terminal needs both `source` lines first:

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
```

| Step | Command |
|---|---|
| 1. Start the simulation (terminal 1) | `ros2 launch robotic_arm_gazebo sim.launch.py` |
| 2. Homing + 4 pick-and-place cycles (terminal 2) | `ros2 launch robotic_arm_control demo.launch.py` |
| Demo with screenshots and a 50 s clip | `ros2 launch robotic_arm_control demo.launch.py cycles:=2 record:=true video_seconds:=50.0` |
| E-stop test (restart the simulation first) | `ros2 launch robotic_arm_control demo.launch.py estop_test:=true` |
| Figures and LaTeX section from the logs | `ros2 run robotic_arm_analysis make_figures --out ~/sim_evidence` |
| View the model with joint sliders (RViz) | `ros2 launch robotic_arm_description display.launch.py` |

- **Where things are saved:** logs go to `~/ras_arm_logs/`, and screenshots and clips to `~/ras_arm_logs/media/`.
- **Changing the cell:** station positions, heights, speeds and payload are in `src/robotic_arm_control/config/task.yaml`.

## Repository contents

| Folder | Content |
|---|---|
| `src/robotic_arm_description` | Cleaned URDF/xacro, visual and collision meshes, `ros2_control` tags, RViz launch |
| `src/robotic_arm_gazebo` | Sorting-cell world (`worlds/sorting_cell.sdf`), simulation launch, world generator |
| `src/robotic_arm_control` | Controllers, homing, inverse kinematics, path planner, pick-and-place state machine, E-stop |
| `src/robotic_arm_analysis` | Static torque, workspace, clearance, offline simulation, figure generation |
| `sim_evidence/` | Results used in the design book: figures, torque table, `results.json`, logged run |
| `docs/WORKSPACE_DETAILS.md` | Full technical notes: URDF fixes, assumptions, test status |

## Main results (design book, Section 4)

| Quantity | Result |
|---|---|
| Reach of the tool | 746 mm from the J1 axis |
| Pick-and-place cycle | 13.6 s |
| Homing of J1 to J4 | 11.8 s |
| Largest tracking error | 0.36° |
| Smallest clearance between non-adjacent links | 35 mm (no collision) |
| Torque margin in the task poses | J2 1.40, J3 4.15, J4 2.00 |
| Torque margin with the arm stretched out | J2 0.31, J3 0.62 |
| Peak torque along the cycles | at most 56 % of the available torque |
| Peak mechanical power of a stepper joint | 0.73 W (9.4 W budgeted) |
| E-stop with the arm loaded | J2 turns by 57°; the gearbox needs at least 1.9 N·m of unpowered friction to hold it |

The arm cannot work stretched out: J2 and J3 margins are below 1 there. This is why the pick stand and the bins are raised and close to the base.

## Troubleshooting

| Problem | Fix |
|---|---|
| Black window or Gazebo crash | Run `export LIBGL_ALWAYS_SOFTWARE=1` before launching |
| Arm meshes not visible | `export IGN_GAZEBO_RESOURCE_PATH=$(ros2 pkg prefix robotic_arm_description)/share` |
| Controllers do not start | Run `ros2 control list_controllers`, and check that `ros-humble-ign-ros2-control` is installed |
| Arm shakes | Lower the `d` gains in `src/robotic_arm_control/config/controllers.yaml`, then rebuild |
