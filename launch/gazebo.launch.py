from pathlib import Path
import importlib.util
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, EmitEvent, LogInfo
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('gp7_stl_description'))
    spec = importlib.util.spec_from_file_location('gp7_gazebo_description', share / 'scripts/gazebo_description.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.build_description(share)

    gazebo = ExecuteProcess(cmd=['ign', 'gazebo', '-r', '-v', '3', str(share / 'worlds/gp7_world.sdf')], output='screen')
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher',
               parameters=[{'robot_description': description, 'use_sim_time': True}], output='screen')
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                  arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'], output='screen')
    spawn = Node(package='ros_gz_sim', executable='create',
                 arguments=['-world', 'gp7_world', '-topic', '/robot_description', '-name', 'gp7', '-allow_renaming', 'false'], output='screen')
    jsb = Node(package='controller_manager', executable='spawner',
               arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager', '--controller-manager-timeout', '120'], output='screen')
    arm = Node(package='controller_manager', executable='spawner',
               arguments=['arm_controller', '--controller-manager', '/controller_manager', '--controller-manager-timeout', '120'], output='screen')
    rviz = Node(package='rviz2', executable='rviz2', parameters=[{'use_sim_time': True}],
                arguments=['-d', str(share / 'rviz/gp7.rviz')], condition=IfCondition(LaunchConfiguration('rviz')), output='screen')

    def after_success(next_actions, label):
        def callback(event, context):
            if event.returncode != 0:
                return [LogInfo(msg=label + ' failed; see preceding errors.'), EmitEvent(event=Shutdown(reason=label + ' failed'))]
            return next_actions
        return callback

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=after_success([jsb], 'Robot spawn'))),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=after_success([arm], 'Joint-state broadcaster startup'))),
        RegisterEventHandler(OnProcessExit(target_action=arm, on_exit=after_success([LogInfo(msg='GP7 controllers are active. Ready for a test trajectory.')], 'Arm controller startup'))),
        RegisterEventHandler(OnProcessExit(target_action=gazebo, on_exit=[EmitEvent(event=Shutdown(reason='Gazebo closed'))])),
        gazebo, rsp, bridge, spawn, rviz,
    ])
