from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_path

import os

def generate_launch_description():
    # Ruta al paquete my_robot_description
    my_robot_pkg = get_package_share_path('my_robot_description')

    # Incluye el launch que visualiza el robot en RViz
    view_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_robot_pkg, 'launch', 'view_robot.py')
        )
    )

    return LaunchDescription([
        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),

        # Nodo de wall_follower autónomo
        Node(
            package='wall_follower_slam',
            executable='wall_follower_node',
            name='wall_follower_node',
            output='screen',
        ),

        # Incluye RViz y visualización del robot
        view_robot_launch
    ])
