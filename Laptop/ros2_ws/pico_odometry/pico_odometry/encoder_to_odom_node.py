#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_from_euler
import math
import time

class EncoderToOdomNode(Node):
    def __init__(self):
        super().__init__('encoder_to_odom_node')

        self.sub = self.create_subscription(
            Float32MultiArray,
            'encoder_angles',
            self.encoder_callback,
            10)

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_left = None
        self.last_right = None
        self.last_time = self.get_clock().now().nanoseconds / 1e9

        self.L = 0.20  # distancia entre ruedas (m)
        self.R = 0.025 # radio de rueda (m)

    def encoder_callback(self, msg):
        angles = msg.data
        if len(angles) != 2:
            self.get_logger().warn('Se esperaban 2 valores: izquierda y derecha')
            return

        angle_left, angle_right = angles

        if self.last_left is None:
            self.last_left = angle_left
            self.last_right = angle_right
            return

        dt = self.get_clock().now().nanoseconds / 1e9 - self.last_time
        self.last_time = self.get_clock().now().nanoseconds / 1e9

        dtheta_l = self.compute_delta(angle_left, self.last_left)
        dtheta_r = self.compute_delta(angle_right, self.last_right)

        self.last_left = angle_left
        self.last_right = angle_right

        d_left = dtheta_l * self.R
        d_right = dtheta_r * self.R
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.L

        self.x += d_center * math.cos(self.theta + d_theta / 2.0)
        self.y += d_center * math.sin(self.theta + d_theta / 2.0)
        self.theta += d_theta

        # Publicar odometría
        odom = Odometry()
        now = self.get_clock().now().to_msg()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        q = quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        self.odom_pub.publish(odom)

    def compute_delta(self, current, previous):
        delta = current - previous
        if delta > math.pi:
            delta -= 2 * math.pi
        elif delta < -math.pi:
            delta += 2 * math.pi
        return delta

def main(args=None):
    rclpy.init(args=args)
    node = EncoderToOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
