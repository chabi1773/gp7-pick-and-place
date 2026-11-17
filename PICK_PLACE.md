# GP7 pick and place — v0.3.3

This update uses the user's corrected **Box A bottom-centre position
(0.5, 0.5, 0) metres**. There is no alternate shifted layout.

## Return to the initial position in v0.3.3

After release, the tool retreats vertically, then returns smoothly over 10
seconds to the initial pose: all six joint angles zero. The return only runs
after confirmed detachment. Its path is checked against the floor, Box A and
the placed Box B using the existing coarse collision envelopes. Completion
checks both the placed cube and the actual arm position (0.005 rad tolerance
per joint). The gripper orientation may change during this empty return.

## Fixes in v0.3.2

Gazebo Fortress 6.18.0 ignores `initial_attach=false` and initially attaches
Box B at its original offset. The runner now explicitly detaches it, waits for
confirmation, and checks its ground position before moving the arm. It also
stops if Box B moves during approach or descent before grasping.

Use `export ROS_DOMAIN_ID=73` in both terminals to isolate this run from old
controller managers. Restart the whole scene after installing this fix.

## Previous fix in v0.3.1

Renamed the trajectory action goal variable to `trajectory_goal_handle`.
The previous `self.handle` name conflicted with the read-only ROS Node handle
and prevented the runner from starting. No geometry or trajectory changed.

## What is included

- A 100 mm simplified suction-style gripper attached to `tool0`.
- Green Box A: 0.30 × 0.30 × 0.20 m, static, centre (0.5, 0.5, 0.1).
- Orange Box B: 0.05 m cube, assumed mass 0.10 kg, dynamic.
- Box B's reachable ground position is a reproducible random sample, seed 7:
  approximately (0.4300, -0.2303, 0) at its bottom centre.
- Complete path preflight and one cycle: approach, descend, grasp, lift,
  transfer, lower, release, retreat, return home, verify placement.
- Box pose and joint feedback, grasp/release acknowledgement and CSV logging.

The robot remains fixed at (0,0,0). The final **Box B centre** is (0.5,0.5,0.225).
The gripper points down throughout the Cartesian stages. The planned box
orientation is unchanged during transport, including yaw.

## 1. Stop the existing launch and update the package

Use Ctrl+C in the current Gazebo/ROS launch terminal. Close any earlier RViz
joint-slider launch. Save the new ZIP as `~/Downloads/gp7_stl_description.zip`.
Back up your current source package outside `src` before overwriting it:

```bash
mkdir -p ~/gp7_backups
cp -a ~/gp7_ws/src/gp7_stl_description ~/gp7_backups/gp7_stl_description_$(date +%Y%m%d_%H%M%S)
unzip -o ~/Downloads/gp7_stl_description.zip -d ~/gp7_ws/src
```

The ZIP contains a top-level `gp7_stl_description` directory. Do not nest it
inside another copy of the package. Its package.xml should say `0.3.3`.

## 2. Install the additional Python dependencies and build

Your working Humble/Fortress controllers and bridges are retained.

```bash
sudo apt update
sudo apt install python3-numpy python3-scipy
source /opt/ros/humble/setup.bash
cd ~/gp7_ws
colcon build --symlink-install --packages-select gp7_stl_description
source install/setup.bash
```

## 3. Open the scene first

```bash
export ROS_DOMAIN_ID=73
ros2 launch gp7_stl_description pick_place.launch.py
```

You should see the robot, a short gripper, the green support box and the small
orange cube. Wait for `GP7 controllers active. Scene ready.`

In a **second terminal**, execute one cycle:

```bash
source /opt/ros/humble/setup.bash
source ~/gp7_ws/install/setup.bash
export ROS_DOMAIN_ID=73
ros2 run gp7_stl_description pick_place.py --ros-args -p use_sim_time:=true
```

The program checks the full trajectory before moving. Expect about 60 simulated
seconds including settling. Wall time may be longer if Gazebo runs slowly.
Stages are printed in the terminal. Success is reported only after checking
that Box B is on top of Box A and upright:

```text
PICK AND PLACE COMPLETE: Box B is upright on Box A; arm returned to its initial position.
```

The final check allows 10 mm in each horizontal coordinate, 5 mm vertically,
and 3 degrees of full orientation error. During carrying, exceeding 3 degrees
also stops the cycle. Those are monitoring tolerances, not claimed accuracy.
The actual maximum carried orientation error is printed and the measured
pose history is written to `~/.ros/gp7_pick_place/<timestamp>.csv`.

Alternatively, on a **fresh launch**, start everything automatically:

```bash
export ROS_DOMAIN_ID=73
ros2 launch gp7_stl_description pick_place.launch.py run:=true
```

Optional feedback RViz: append `rviz:=true` to either launch command. Do not
start `display.launch.py` at the same time. RViz shows the robot; the boxes are
Gazebo world objects and are not published as RViz markers.

## Repeat or stop

Press Ctrl+C to stop the runner. It requests action cancellation and retains
an attached box. Stop and restart the scene before another cycle; the script
requires the initial zero-joint configuration and the cube's initial position.
Do not use Gazebo's reset button during a running cycle.

The launch uses a unique Ignition transport partition for its simulator and
bridges. Other ROS launches must still be stopped to avoid duplicate ROS topics.

## How the grasp works

The visible gripper uses two cylindrical shapes. A Gazebo `DetachableJoint`
attaches Box B to `link_6` after the tool is aligned above the cube, with an
approximately 1 mm gap. The attachment preserves the current relative pose;
the cube is not teleported. This is an **ideal rigid grasp**, not a simulation
of suction pressure, finger contact or friction forces. The gap avoids the
Fortress/DART restriction on attaching bodies already in contact.

The plugin starts attached for compatibility with Fortress 6.18.0; before any
arm motion, the runner sends a detach request and waits for acknowledgement.
Do not manually move the arm before this initialization. ROS topics `/gp7/gripper/attach` and
`/gp7/gripper/detach` bridge `std_msgs/Empty` to Gazebo. Its state is returned on
`/gp7/gripper/state`. Box B's actual world pose is available on
`/gp7/box_b_pose`. The script waits for state acknowledgement before continuing.
The gripper adds an assumed 0.15 kg; the original CAD inertias are unchanged.

## What was checked, and what still needs testing

Offline checks passed for 5,000 trajectory configurations, including the cubic
interpolation used between position/velocity trajectory points. Checks cover
joint position/speed limits, fixed tool orientation, and the robot/cube against
the floor and Box A, plus the placed Box B during retreat and return. See `checks/pick_place_validation.json` for measured values.

**ROS 2 and Gazebo execution were not available in the authoring environment.**
You must run the above commands to validate plugins, feedback and dynamic
tracking on your machine. The code targets your Fortress 6.18.0 installation.

The simulation still uses conservative per-link bounding boxes and disables
robot self-collision. The check is not a full collision planner. CAD masses
remain provisional; the model is not calibrated real GP7 dynamics.

Run the offline checks again after geometry or path edits:

```bash
python3 ~/gp7_ws/src/gp7_stl_description/scripts/check_pick_place.py
```

## Files to study or edit

| File | Purpose |
|---|---|
| `launch/pick_place.launch.py` | Simulator, robot, bridges, controller startup |
| `worlds/pick_place.sdf` | Ground, Box A and Box B |
| `scripts/pick_place_scene.py` | Gripper and reproducible world generation |
| `scripts/pick_place_model.py` | FK, bounded numerical IK, waypoints, smooth timing |
| `scripts/pick_place.py` | Action execution, grasp/release, feedback checks |
| `config/pick_place_bridge.yaml` | Clock, grasp commands/state, box pose bridges |
| `scripts/check_pick_place.py` | Offline trajectory and scene consistency checks |

`scene()` in `pick_place_model.py` is the authoritative coordinate definition.
After editing it, regenerate the world and rebuild the package:

```bash
cd ~/gp7_ws/src/gp7_stl_description
PYTHONPATH="$PWD/scripts" python3 - <<'PY'
from pathlib import Path
from pick_place_scene import world
package = Path.cwd()
(package / 'worlds/pick_place.sdf').write_text(world(package))
PY
python3 scripts/check_pick_place.py
cd ~/gp7_ws
colcon build --symlink-install --packages-select gp7_stl_description
```

The Cartesian segments use quintic time scaling, with zero endpoint velocity
and acceleration in the underlying scalar profile. Joint positions and
estimated velocities are sent at 20 Hz as a single action per stage. The
controller interpolates them. This numerical IK implementation is a reference
for the simulation; it does **not** replace your assignment's own DH table,
analytical FK/IK derivations, explanations or results discussion.

## Troubleshooting

- **No `/gp7/box_b_pose`:** inspect launch output for PosePublisher and bridge
  errors; `ros2 topic echo /gp7/box_b_pose --once` should show the cube pose.
- **No attached acknowledgement:** inspect DetachableJoint loading; this code
  uses explicit startup detachment, `attach_topic`, `detach_topic`, `child_link`.
- **Missing controller manager:** the previous absolute library path fix is
  retained in `gazebo_description.py`; inspect the first plugin-loading error.
- **Grasp alignment failed:** keep the printed gap value and joint feedback;
  do not remove the guard. It prevents attaching the cube from a distance or
  while intersecting the tool.
- **Orientation or placement failure:** keep the CSV and terminal error.
  The program intentionally does not report success based on commands alone.

## Primary implementation references

- [Gazebo Fortress DetachableJoint documentation](https://gazebosim.org/api/gazebo/6/detachablejoints.html)
- [Fortress DetachableJoint source](https://github.com/gazebosim/gz-sim/blob/ignition-gazebo6_6.18.0/src/systems/detachable_joint/DetachableJoint.cc)
- [Fortress PosePublisher source](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/src/systems/pose_publisher/PosePublisher.cc)
- [ROS/Gazebo Humble bridge documentation](https://github.com/gazebosim/ros_gz/blob/humble/ros_gz_bridge/README.md)

Original model and mesh attribution remain in `reference/SOURCES.md`.
