# GP7 Pick-and-Place Simulation

A ROS 2 Humble and Gazebo Fortress simulation of a Yaskawa GP7 robot that picks up a small box, places it on a larger box, and returns to its initial position.

The project includes CAD meshes, a URDF robot description, provisional inertial properties, ROS 2 controllers, a simplified suction-style gripper, trajectory planning, and execution feedback checks.

## Requirements

- Ubuntu 22.04
- ROS 2 Humble, preferably the Desktop installation
- Gazebo Fortress (`ign gazebo`); the compatibility fixes target version 6.18.0
- Python 3, NumPy and SciPy

Install ROS 2 first using the [official Humble Ubuntu instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html). Gazebo installation instructions are available in the [Fortress documentation](https://gazebosim.org/docs/fortress/install_ubuntu/).

This package uses **Gazebo Fortress**, not Gazebo Classic.

## Installation

### 1. Install package dependencies

After installing ROS 2 Humble and Gazebo Fortress:

```bash
sudo apt update
sudo apt install git python3-colcon-common-extensions python3-numpy python3-scipy \
  ros-humble-gz-ros2-control ros-humble-ros2-controllers \
  ros-humble-ros-gz-sim ros-humble-ros-gz-bridge \
  ros-humble-control-msgs ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui ros-humble-rviz2
```

### 2. Clone the repository

Replace `YOUR_GITHUB_USERNAME` with the repository owner's GitHub username.

```bash
mkdir -p ~/gp7_ws/src
cd ~/gp7_ws/src
git clone https://github.com/YOUR_GITHUB_USERNAME/gp7-pick-and-place.git gp7_stl_description
```

If `~/gp7_ws/src/gp7_stl_description` already contains this package, use that existing copy instead of cloning another one. Keep only one copy of the package inside the workspace's `src` directory.

### 3. Build the package

```bash
source /opt/ros/humble/setup.bash
cd ~/gp7_ws
colcon build --symlink-install --packages-select gp7_stl_description
source install/setup.bash
```

## Start the complete simulation

Stop any previous GP7 simulation launches with **Ctrl+C**, then run:

```bash
source /opt/ros/humble/setup.bash
source ~/gp7_ws/install/setup.bash
export ROS_DOMAIN_ID=73

ros2 launch gp7_stl_description pick_place.launch.py run:=true
```

This starts Gazebo, spawns the robot and boxes, activates the controllers, and runs one pick-and-place cycle automatically.

The sequence is:

1. Detach the box from the startup attachment and confirm the gripper is released.
2. Approach Box B and descend to the grasp position.
3. Attach Box B to the gripper.
4. Lift and transfer it while maintaining its orientation.
5. Lower it onto the centre of Box A and release it.
6. Retreat vertically and return all six joints to their initial zero-angle position.
7. Check the final box pose and robot joint positions.

Allow approximately **60 simulated seconds**, including settling. Wall-clock time depends on the simulation speed.

On successful verification, the terminal prints:

```text
PICK AND PLACE COMPLETE: Box B is upright on Box A; arm returned to its initial position.
```

## Start the scene and motion separately

**Terminal 1 — start the scene:**

```bash
source /opt/ros/humble/setup.bash
source ~/gp7_ws/install/setup.bash
export ROS_DOMAIN_ID=73

ros2 launch gp7_stl_description pick_place.launch.py
```

Wait for `GP7 controllers active. Scene ready.` and keep the terminal open.

**Terminal 2 — run one cycle:**

```bash
source /opt/ros/humble/setup.bash
source ~/gp7_ws/install/setup.bash
export ROS_DOMAIN_ID=73

ros2 run gp7_stl_description pick_place.py --ros-args -p use_sim_time:=true
```

Use the same `ROS_DOMAIN_ID` in every terminal interacting with this simulation. Do not start a second runner while the automatic cycle is running.

To repeat the task, stop and restart the scene. The cube must return to its original pickup location before another cycle; returning the arm home alone does not reset the cube.

## Scene configuration

All positions are expressed in metres in the world frame.

| Object | Dimensions | Initial position |
|---|---|---|
| Robot base | GP7 model | Base origin: `(0, 0, 0)` |
| Box A | `0.30 × 0.30 × 0.20` | Bottom centre: `(0.5, 0.5, 0)` |
| Box B | `0.05 × 0.05 × 0.05` | Bottom centre: approximately `(0.4300, -0.2303, 0)` |

Box B's pickup position is a reproducible random sample using seed 7. Its target centre after placement is **`(0.5, 0.5, 0.225)`**.

The gripper is a simplified 100 mm tool. Grasping uses a detachable fixed joint, not simulated suction pressure or finger friction.

## RViz and camera controls

To show RViz alongside the simulation:

```bash
ros2 launch gp7_stl_description pick_place.launch.py run:=true rviz:=true
```

Run this instead of another simulation launch, using the environment setup above. RViz displays robot feedback; the boxes are Gazebo world objects.

For an independent robot-only view with joint sliders:

```bash
ros2 launch gp7_stl_description display.launch.py
```

Do not run the joint-slider display alongside Gazebo because both would publish joint states.

In Gazebo, middle-button drag rotates the view, left-button drag pans, and the scroll wheel zooms. With a touchpad, use **Shift + left-click drag** to rotate and two-finger scrolling to zoom.

## Project files

| Path | Purpose |
|---|---|
| `urdf/gp7.urdf` | Robot links, joints, meshes and inertial properties |
| `meshes/` | Robot STL meshes |
| `launch/pick_place.launch.py` | Complete simulation startup |
| `worlds/pick_place.sdf` | Ground and boxes |
| `config/controllers.yaml` | Joint-state and arm controllers |
| `config/pick_place_bridge.yaml` | ROS/Gazebo communication |
| `scripts/pick_place_model.py` | Forward kinematics, numerical IK and trajectory planning |
| `scripts/pick_place_scene.py` | Gripper and scene generation |
| `scripts/pick_place.py` | Motion execution and feedback checks |
| `scripts/check_pick_place.py` | Offline trajectory validation |
| `PICK_PLACE.md` | Detailed configuration and troubleshooting |
| `reference/SOURCES.md` | Model sources and attribution |

## Validation and logs

Run the offline checks with:

```bash
python3 ~/gp7_ws/src/gp7_stl_description/scripts/check_pick_place.py
```

The current trajectory passed 5,000 offline configuration checks covering joint limits, speed limits, planned tool orientation, and coarse clearance from the floor and boxes. These checks do not replace Gazebo runtime testing.

Measured box poses, execution stages and orientation errors are saved to:

```text
~/.ros/gp7_pick_place/<timestamp>.csv
```

The model uses provisional CAD inertias and coarse collision envelopes. Robot self-collision is disabled. Numerical IK and an ideal grasp support simulation development; they are not a calibrated model of real GP7 dynamics or a substitute for an analytical kinematics derivation.

## Troubleshooting

### Controllers are already loaded or fail to configure

Stop old simulation launches. Set `export ROS_DOMAIN_ID=73` in both terminals, then launch one fresh scene.

Check controller status:

```bash
ros2 control list_controllers
```

Both `joint_state_broadcaster` and `arm_controller` should be `active`.

### Missing joint states, box pose or clock

Keep Gazebo playing and check:

```bash
timeout 5s ros2 topic echo /clock --once
timeout 5s ros2 topic echo /joint_states --once
timeout 5s ros2 topic echo /gp7/box_b_pose --once
```

Use `pick_place.launch.py`, and verify that both terminals have the same ROS domain. Inspect the launch terminal for plugin or bridge errors.

### Box B moves before the grasp stage

Use the current runner, which explicitly detaches the box before moving the arm. Fortress 6.18.0 does not honour `initial_attach=false`. Restart the whole scene after updating the package.

### `handle cannot be modified after node creation`

An older runner is installed. Update the source and rebuild. The current script uses `self.trajectory_goal_handle`, not `self.handle`.

## Attribution

Retain the source and licensing information in `reference/SOURCES.md`. CAD meshes retain their original ownership and licensing terms.
