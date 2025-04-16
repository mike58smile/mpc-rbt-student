from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mpc_rbt_student',
            executable='PlanningNode',
            name='planning_node',
            output='screen'
        )
    ])