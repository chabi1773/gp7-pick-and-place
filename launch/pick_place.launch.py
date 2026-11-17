"""Fortress pick-and-place scene: Box A bottom centre at (0.5, 0.5, 0)."""
from pathlib import Path
import sys
import uuid
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, EmitEvent, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share = Path(get_package_share_directory('gp7_stl_description'))
    sys.path.insert(0, str(share / 'scripts'))
    from pick_place_scene import description
    gazebo = ExecuteProcess(cmd=['ign', 'gazebo', '-r', '-v', '3', str(share / 'worlds' / 'pick_place.sdf')], output='screen')
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher',
               parameters=[{'robot_description': description(share), 'use_sim_time': True}], output='screen')
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                  parameters=[{'config_file': str(share / 'config/pick_place_bridge.yaml')}], output='screen')
    spawn = Node(package='ros_gz_sim', executable='create',
                 arguments=['-world', 'gp7_world', '-topic', '/robot_description', '-name', 'gp7', '-allow_renaming', 'false'], output='screen')
    jsb = Node(package='controller_manager', executable='spawner',
               arguments=['joint_state_broadcaster', '--controller-manager-timeout', '120'], output='screen')
    arm = Node(package='controller_manager', executable='spawner',
               arguments=['arm_controller', '--controller-manager-timeout', '120'], output='screen')
    runner = Node(package='gp7_stl_description', executable='pick_place.py',
                  parameters=[{'use_sim_time': True}],
                  condition=IfCondition(LaunchConfiguration('run')), output='screen')
    rviz = Node(package='rviz2', executable='rviz2', parameters=[{'use_sim_time': True}],
                arguments=['-d', str(share / 'rviz/gp7.rviz')], condition=IfCondition(LaunchConfiguration('rviz')), output='screen')
    def next_if_ok(actions, label):
        def callback(event, context):
            if event.returncode != 0:
                return [LogInfo(msg=label+' failed; see preceding errors.'), EmitEvent(event=Shutdown(reason=label+' failed'))]
            return actions
        return callback
    return [
        SetEnvironmentVariable('IGN_PARTITION', 'gp7_'+uuid.uuid4().hex[:12]),
        LogInfo(msg='Box A bottom centre: (0.5, 0.5, 0); top surface: z=0.2 m.'),
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=next_if_ok([jsb], 'Spawn'))),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=next_if_ok([arm], 'Joint-state broadcaster'))),
        RegisterEventHandler(OnProcessExit(target_action=arm, on_exit=next_if_ok([LogInfo(msg='GP7 controllers active. Scene ready.'), runner], 'Arm controller'))),
        RegisterEventHandler(OnProcessExit(target_action=gazebo, on_exit=[EmitEvent(event=Shutdown(reason='Gazebo closed'))])),
        gazebo, rsp, bridge, spawn, rviz,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('run', default_value='false', description='Start one pick/place cycle after the controllers activate'),
        DeclareLaunchArgument('rviz', default_value='false'), OpaqueFunction(function=setup),
    ])
