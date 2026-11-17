#!/usr/bin/env python3
"""One monitored GP7 pick/place cycle. Restart the scene before each run."""
from pathlib import Path
import csv
import sys
import time
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty, String

PACKAGE = Path(get_package_share_directory('gp7_stl_description'))
sys.path.insert(0, str(PACKAGE / 'scripts'))
from pick_place_model import Robot, JOINT_NAMES, DOWN, make_plan, scene


class PickPlace(Node):
    def __init__(self):
        super().__init__('gp7_pick_place')
        self.robot = Robot(PACKAGE)
        self.q = self.box = None
        self.joint_received = self.box_received = 0.
        self.grip_state = ''
        self.holding = False
        self.trajectory_goal_handle = None
        self.stage = 'startup'
        self.reference_rotation = np.eye(3)
        self.max_orientation_error = 0.
        self.create_subscription(JointState, '/joint_states', self.on_joints, qos_profile_sensor_data)
        self.create_subscription(Pose, '/gp7/box_b_pose', self.on_box, qos_profile_sensor_data)
        self.create_subscription(String, '/gp7/gripper/state', self.on_grip, 10)
        self.attach_pub = self.create_publisher(Empty, '/gp7/gripper/attach', 10)
        self.detach_pub = self.create_publisher(Empty, '/gp7/gripper/detach', 10)
        self.arm = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        logdir = Path.home() / '.ros/gp7_pick_place'
        logdir.mkdir(parents=True, exist_ok=True)
        self.logpath = logdir / (time.strftime('%Y%m%d_%H%M%S') + '.csv')
        self.logfile = self.logpath.open('w', newline='')
        self.writer = csv.writer(self.logfile)
        self.writer.writerow(['sim_seconds', 'stage', 'holding', 'box_x', 'box_y', 'box_z',
                              'box_qx', 'box_qy', 'box_qz', 'box_qw', 'orientation_error_deg'])

    def on_joints(self, msg):
        values = dict(zip(msg.name, msg.position))
        if all(name in values for name in JOINT_NAMES):
            self.q = np.array([values[name] for name in JOINT_NAMES])
            self.joint_received = time.monotonic()

    def on_box(self, msg):
        self.box = msg
        self.box_received = time.monotonic()
        p, q = msg.position, msg.orientation
        angle = self.box_rotation_error()
        if self.holding:
            self.max_orientation_error = max(self.max_orientation_error, angle)
        self.writer.writerow([self.get_clock().now().nanoseconds / 1e9, self.stage, self.holding,
                              p.x, p.y, p.z, q.x, q.y, q.z, q.w, np.rad2deg(angle)])

    def on_grip(self, msg):
        self.grip_state = msg.data

    def box_rotation(self):
        q = self.box.orientation
        return Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

    def box_rotation_error(self):
        return Rotation.from_matrix(self.box_rotation() @ self.reference_rotation.T).magnitude()

    def box_xyz(self):
        p = self.box.position
        return np.array([p.x, p.y, p.z])

    def spin(self, monitor=True):
        rclpy.spin_once(self, timeout_sec=.05)
        if monitor:
            now = time.monotonic()
            if now-self.joint_received > 3 or now-self.box_received > 3:
                raise RuntimeError('Feedback stopped. Check simulation playback and the pose/clock bridges.')
            if self.stage in ('approach', 'descend_to_grasp'):
                if np.linalg.norm(self.box_xyz()-np.array(scene()['b'])) > .005:
                    raise RuntimeError('Box B moved before grasping; check startup detachment. Motion cancelled.')
            if self.holding:
                if self.grip_state != 'attached':
                    raise RuntimeError('Gripper unexpectedly detached while carrying Box B.')
                if self.box_rotation_error() > np.deg2rad(3):
                    raise RuntimeError('Box orientation changed by more than 3 degrees; motion cancelled.')
                tcp = self.robot.tcp(self.q)[:3, 3]
                if np.linalg.norm(self.box_xyz() - (tcp + [0, 0, -.026])) > .02:
                    raise RuntimeError('Box B did not follow the gripper; motion cancelled.')

    def wait(self, condition, timeout, message, monitor=True):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not condition():
            if time.monotonic() > deadline:
                raise RuntimeError(message)
            self.spin(monitor)
        if not rclpy.ok():
            raise RuntimeError('ROS shutdown')

    def settle(self, seconds=1.):
        start = self.get_clock().now().nanoseconds
        self.wait(lambda: (self.get_clock().now().nanoseconds-start)/1e9 >= seconds,
                  60, 'Simulation clock stopped during settling.')

    def move(self, segment):
        self.stage = segment['name']
        self.get_logger().info('Stage: ' + self.stage)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES
        count = len(segment['q'])
        for i, (q, v) in enumerate(zip(segment['q'], segment['velocity'])):
            point = JointTrajectoryPoint()
            point.positions, point.velocities = q, v
            ns = round(i*segment['duration']/(count-1)*1e9)
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(ns, 10**9)
            goal.trajectory.points.append(point)
        future = self.arm.send_goal_async(goal)
        self.wait(future.done, 10, 'Controller did not acknowledge the goal.')
        self.trajectory_goal_handle = future.result()
        if not self.trajectory_goal_handle.accepted:
            raise RuntimeError('Controller rejected ' + self.stage)
        result_future = self.trajectory_goal_handle.get_result_async()
        self.wait(result_future.done, 180, 'Trajectory timed out.')
        result = result_future.result()
        self.trajectory_goal_handle = None
        if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code != 0:
            raise RuntimeError('Trajectory failed: ' + result.result.error_string)
        self.wait(lambda: np.max(np.abs(self.q-np.array(segment['q'][-1]))) < .005,
                  15, 'Arm has not settled at the commanded joint positions.')
        self.settle(.5)

    def grip(self, attach, initializing=False):
        self.stage = 'initialize_gripper' if initializing else ('grasp' if attach else 'release')
        self.get_logger().info('Stage: ' + self.stage)
        expected = 'attached' if attach else 'detached'
        publisher = self.attach_pub if attach else self.detach_pub
        deadline = time.monotonic() + 10
        # Turn off carrying checks while waiting for the commanded detachment.
        if not attach:
            self.holding = False
        while self.grip_state != expected:
            if time.monotonic() > deadline:
                raise RuntimeError('No '+expected+' acknowledgement. Check DetachableJoint plugin and bridge.')
            publisher.publish(Empty())
            for _ in range(4):
                self.spin()
        self.holding = attach
        self.settle(.5)

    def run(self):
        self.get_logger().info('Checking the complete path before moving...')
        plan = make_plan(PACKAGE)
        self.wait(lambda: self.q is not None and self.box is not None and self.get_clock().now().nanoseconds > 0,
                  30, 'Missing /joint_states, /gp7/box_b_pose or /clock.', monitor=False)
        if not self.arm.wait_for_server(timeout_sec=20):
            raise RuntimeError('arm_controller action server is unavailable.')
        self.settle(1.)
        if np.max(np.abs(self.q)) > .03:
            raise RuntimeError('Start from a freshly launched scene with zero joint angles. Restart Gazebo.')
        # Fortress 6.18.0 always starts attached; it ignores initial_attach=false.
        # Start attached consistently across versions, then await a detach ACK
        # before sending any arm trajectory. Never reset/teleport the cube.
        self.grip(False, initializing=True)
        self.settle(1.)
        if np.linalg.norm(self.box_xyz()-scene()['b']) > .005:
            raise RuntimeError('Box B is not at its initial position. Restart the scene before another cycle.')
        self.reference_rotation = self.box_rotation()
        if Rotation.from_matrix(self.reference_rotation).magnitude() > np.deg2rad(1):
            raise RuntimeError('Box B is not initially upright.')
        for segment in plan['segments']:
            if segment['name'] == 'return_home' and (self.holding or self.grip_state != 'detached'):
                raise RuntimeError('Cannot return home until Box B is confirmed detached.')
            self.move(segment)
            if segment['name'] == 'descend_to_grasp':
                tcp = self.robot.tcp(self.q)
                gap = tcp[2, 3] - (self.box.position.z + .025)
                if np.linalg.norm(tcp[:2, 3]-self.box_xyz()[:2]) > .002 or not .0002 < gap < .003:
                    raise RuntimeError(f'Grasp alignment failed: gap={gap:.5f} m. No attachment requested.')
                if Rotation.from_matrix(tcp[:3, :3] @ DOWN.T).magnitude() > np.deg2rad(.5):
                    raise RuntimeError('Gripper is not pointing vertically down.')
                self.grip(True)
            elif segment['name'] == 'lower':
                if np.linalg.norm(self.box_xyz()[:2]-np.array(scene()['a'][:2])) > .01:
                    raise RuntimeError('Box is not over the centre of Box A; retaining grasp.')
                self.grip(False)
        self.stage = 'verify_placement'
        self.settle(2.)
        target = np.array([.5, .5, .225])
        error = np.abs(self.box_xyz()-target)
        if max(error[:2]) > .01 or error[2] > .005 or self.box_rotation_error() > np.deg2rad(3):
            raise RuntimeError('Placement verification failed: measured Box B centre '+str(self.box_xyz()))
        if np.max(np.abs(self.q)) > .005:
            raise RuntimeError('Arm has not returned to its initial zero-joint position.')
        self.stage = 'complete'
        self.get_logger().info('PICK AND PLACE COMPLETE: Box B is upright on Box A; arm returned to its initial position. '
                               f'Max carried orientation error: {np.rad2deg(self.max_orientation_error):.3f} deg. '
                               f'Feedback log: {self.logpath}')

    def abort(self):
        if self.trajectory_goal_handle is not None:
            future = self.trajectory_goal_handle.cancel_goal_async()
            deadline = time.monotonic()+3
            while not future.done() and rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=.1)
        attachment = 'Box B is attached; retaining it.' if self.grip_state == 'attached' else 'No confirmed attachment is being retained.'
        self.get_logger().error('Cycle stopped. '+attachment+' Restart the scene before retrying. '
                                f'Feedback log: {self.logpath}')


def main():
    rclpy.init()
    node = PickPlace()
    code = 0
    try:
        node.run()
    except KeyboardInterrupt:
        code = 130
        node.abort()
    except Exception as exc:
        node.get_logger().error(str(exc))
        node.abort()
        code = 1
    finally:
        node.logfile.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
