from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # Nodo 1: micro-ROS agent
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent_serial',
            arguments=['serial', '--dev', '/dev/ttyACM0'],
            output='screen'
        ),

        # Nodo 2: Kinect driver
        Node(
            package='kinect_ros2',
            executable='kinect_ros2_node',
            name='kinect_node',
            output='screen'
        ),

        # Nodo 3: depthimage_to_laserscan con remapeos
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depth_to_scan',
            output='screen',
            remappings=[
                ('depth', '/depth/image_raw'),
                ('depth_camera_info', '/depth/camera_info')
            ]
        ),
    ])