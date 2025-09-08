from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_my_bot = FindPackageShare('my_bot_description')
    urdf_path = PathJoinSubstitution([pkg_my_bot, 'urdf', 'my_bot.urdf.xacro'])
    config_path = PathJoinSubstitution([pkg_my_bot, 'config', 'my_bot_controllers.yaml'])

    xacro_cmd = Command([
        FindExecutable(name='xacro'),
        ' ',
        urdf_path
    ])

    robot_description = {'robot_description': ParameterValue(xacro_cmd, value_type=str)}

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description, config_path],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
            shell=True,
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint1_position_controller'],
            shell=True,
            output='screen'
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'my_bot', '-topic', 'robot_description', '-x', '0', '-y', '0', '-z', '0.1'],
            output='screen'
        )
    ])

