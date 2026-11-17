# GP7 — Gazebo pick and place (v0.3.3)

**Start with [PICK_PLACE.md](PICK_PLACE.md)** for the new gripper, two boxes and
complete monitored sequence. Box A uses the corrected bottom-centre position
**(0.5, 0.5, 0)**. The original empty-world controller test remains available.

The sections below document the CAD inertias and original controller setup.

The portable robot description is **urdf/gp7.urdf**. Keep the entire package,
including **meshes/**. This update preserves the geometry inspected in RViz
and adds the supplied mass, centre-of-mass and inertia data to all seven links.

**Physics values are provisional.** Every reported mass/volume ratio is about
1 g/cm³ (1000 kg/m³), suggesting uniform/default CAD density rather than
verified component materials. Total supplied mass is **20.27507 kg**. These
values support an initial software test, not validated real GP7 dynamics,
motor torques or payload performance. Check component materials or mass
overrides in SolidWorks. Do not assign one metal to every component or simply
rescale everything to a total robot mass.

## Plugin path fix in v0.2.1

The control plugin is now loaded using its absolute installed library path,
resolved from the gz_ros2_control ROS package prefix. This addresses the
reported Fortress error: could not find the shared library for
gz_ros2_control-system. If the library is absent, the launch now reports
that directly instead of leaving the controller spawner waiting.

## Changes

- Converted grams to kilograms, millimetres to metres and g·mm² to kg·m².
- Used the tensors taken at the centre of mass. Negated the off-diagonal
  products from SolidWorks positive tensor notation, reproducing the reported
  principal moments. Document-origin tensors were used only for verification.
- Transformed each COM and rotated each COM tensor into the corresponding
  URDF link axes using the existing mesh transforms. Inertial origins are at
  COM with zero RPY. No parallel-axis offset belongs in these COM tensors.
- Added world mounting, six position-controlled joints, a joint-state
  broadcaster and an arm_controller (JointTrajectoryController).
- In Gazebo only, detailed collision meshes become inexpensive bounding boxes
  and self-collision is disabled. The boxes fill gaps in hollow/U-shaped parts;
  they are not precise grasp/contact surfaces.

Meshes are unchanged, at scale 0.001. Their alignment remains estimated against
upstream GP7 geometry. Joint axes and limits remain from that reference.
The supplied inertias do not independently verify mesh/joint alignment.
Position control tests motion; it is not a calibrated servo/torque model.
The separate pick_place.launch.py now adds the gripper and task scene; see PICK_PLACE.md.

## 1. Replace the previous package

Stop the earlier RViz launch with Ctrl+C. Do not run its joint sliders alongside
Gazebo, because they would also publish /joint_states.

Back up local edits **outside** the workspace src folder. Extract this ZIP so
its package replaces `~/gp7_ws/src/gp7_stl_description`. Check that package.xml
says version 0.3.3. Keep only one copy of this package under src.

## 2. Install dependencies

For Ubuntu 22.04, ROS 2 Humble and your existing Gazebo Fortress:

```bash
sudo apt update
sudo apt install ros-humble-gz-ros2-control ros-humble-ros2-controllers ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-control-msgs
```

Current Humble gz_ros2_control binaries target Fortress; this launch still uses
`ign gazebo`. This is not Gazebo Classic. If apt cannot find a package, retain
the exact error instead of installing a different Gazebo generation.

## 3. Build and launch

```bash
source /opt/ros/humble/setup.bash
cd ~/gp7_ws
colcon build --symlink-install --packages-select gp7_stl_description
source install/setup.bash
ros2 launch gp7_stl_description gazebo.launch.py
```

Wait for `GP7 controllers are active. Ready for a test trajectory.` The launch
starts gravity and a ground plane, spawns the robot, then loads the controllers.
For an additional RViz view of simulated feedback, use this launch instead:

```bash
ros2 launch gp7_stl_description gazebo.launch.py rviz:=true
```

Run only one Gazebo launch. This RViz view uses simulated feedback, not sliders.

## 4. Check and move

Open a second terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/gp7_ws/install/setup.bash
ros2 control list_controllers
```

Both joint_state_broadcaster and arm_controller should be active. The base
should remain fixed. If links fall or jump, save the first launch error before
attempting motion.

From the initial zero pose in the empty scene, this test rotates only joint 1
to 0.25 radians in five seconds. Other joints target zero. It is a controller
test, not the assignment trajectory or collision-aware planning.

```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6], points: [{positions: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0], velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 5, nanosec: 0}}]}}" --feedback
```

Return to zero by sending the same command with all six positions set to 0.0.
After successful test motion, follow PICK_PLACE.md for the gripper, boxes and task trajectory.

## RViz-only mode

```bash
ros2 launch gp7_stl_description display.launch.py
```

Use separately from Gazebo. A KDL root-inertia warning can occur in display-only
mode; base_link remains the fixed visual frame. The Gazebo variant adds a
massless world root and a fixed mounting joint.

## Files and validation

- reference/mass_properties_input.json: transcribed source reports and units.
- checks/inertia_conversion.json: transformed values and numerical checks.
- scripts/convert_inertias.py: conversion and verification code; requires NumPy.
- scripts/gazebo_description.py: prepares the simulation model on your computer,
  resolving mesh paths and adding collision boxes and control configuration.
- config/controllers.yaml: controller settings.
- checks/physics_validation.json: structural and inertia check results.
- checks/preview.png: offline geometry rendering, not a Gazebo screenshot.

ROS 2/Gazebo are not installed in the preparation environment. Numerical and
file checks passed; plugin loading, controller activation, contacts and runtime
stability remain to be tested on your computer. Correcting source properties
requires rerunning conversion, reviewing checks and rebuilding the package.

Optional diagnostic model export:

```bash
python3 ~/gp7_ws/src/gp7_stl_description/scripts/gazebo_description.py --output /tmp/gp7_gazebo.urdf
ign sdf -p /tmp/gp7_gazebo.urdf > /tmp/gp7_gazebo.sdf
```

The expanded file contains local paths. Share the original ZIP instead.
