from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ur10_moveit_control',
            executable='goto_pose_ik',
            name='ur10_mover',
            output='screen'
        )
    ])
