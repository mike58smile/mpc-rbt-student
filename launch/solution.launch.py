import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_dir = get_package_share_directory('mpc_rbt_student')
    rviz_config_path = os.path.join(package_dir, 'rviz', 'config.rviz')

    # Path to the simulator's launch file
    simulator_pkg_dir = get_package_share_directory('mpc_rbt_simulator')
    simulator_launch_path = os.path.join(simulator_pkg_dir, 'launch', 'simulation.launch.py')

    return LaunchDescription([
        # Include the simulator launch file
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(simulator_launch_path)
        # ),
        Node(
            package='mpc_rbt_student',
            executable='LocalizationNode',
            name='localization_node',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        ),
        # Add other nodes as needed
    ])