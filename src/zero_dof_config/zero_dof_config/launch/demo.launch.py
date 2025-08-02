# File: panda_visualization.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Set this to the actual package that contains your 'urdf' folder and panda.urdf
    package_name = 'panda_0dof_moveit_config'
    urdf_name = 'panda.urdf'

    # Find the path to the URDF using ROS 2 package tools
    package_share_dir = get_package_share_directory(package_name)
    urdf_file = os.path.join(package_share_dir, 'urdf', urdf_name)
    rviz_config = "panda_view.rviz"

    # Read the URDF
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    print(os.path.join(package_share_dir, 'rviz', rviz_config))

    return LaunchDescription([
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            arguments=[urdf_file],  # optional; some setups only need robot_description param
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(package_share_dir, 'rviz', rviz_config)],
            output='screen'
        )

        

    ])
