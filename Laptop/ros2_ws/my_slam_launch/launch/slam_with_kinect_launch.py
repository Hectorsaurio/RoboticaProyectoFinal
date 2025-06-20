from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Ruta al URDF
    urdf_path = PathJoinSubstitution([
        FindPackageShare("my_robot_description"),
        "urdf",
        "mi_robot.urdf"
    ])

    return LaunchDescription([
        # Publicador de estado del robot
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": open(
                get_package_share_directory("my_robot_description") + "/urdf/mi_robot.urdf").read()}]
        ),

        # Static transform del frame base_link a camera_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link'],
            output='screen'
        ),

        # Nodo SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
    ])
