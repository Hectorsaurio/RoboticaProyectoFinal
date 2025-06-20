from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    # Nodo 1: Kinect v1 con resolución y FPS reducidos
    kinect_node = Node(
        package='kinect_ros2',
        executable='kinect_ros2_node',
        name='kinect_node',
        output='screen',
        parameters=[{
            'image_width': 320,
            'image_height': 240,
            'fps': 15,
        }]
    )


    # Nodo 3: micro-ROS Agent por serial
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent_serial',
        arguments=['serial', '--dev', '/dev/ttyACM0'],
        output='screen'
    )

    return LaunchDescription([
        micro_ros_agent,
        kinect_node
    ])