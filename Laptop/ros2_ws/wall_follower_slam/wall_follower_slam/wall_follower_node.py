#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class SLAMAvoidanceNode(Node):
    def __init__(self):
        super().__init__('slam_avoidance_node')
        # Publisher for velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Initial distances
        self.dist_right = float('inf')
        self.dist_front = float('inf')
        self.actual_speed = 0.0 

        # Wall-following target distance
        self.wall_dist = 0.4
        # Speeds
        self.forward_speed = 0.2
        self.turn_speed = 0.5

        # PID coefficients
        self.kp = 0.9
        self.ki = 0.2
        self.kd = 1
        self.integral = 0.0
        self.prev_error = 0.0
        self.dt = 0.1  # control loop interval

        # Subscribers and timer
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # Control loop timer
        self.timer = self.create_timer(1, self.control_loop)

    def scan_callback(self, msg: LaserScan):
        n = len(msg.ranges)
        # Define right and front sectors
        right_sector = msg.ranges[int(0.75*n):] + msg.ranges[:int(0.25*n)]
        front_sector = msg.ranges[int(0.375*n):int(0.625*n)]

        # Filter out invalid readings (<= 0)
        right_valid = [d for d in right_sector if d > 0]
        front_valid = [d for d in front_sector if d > 0]

        # Update distances
        self.dist_right = min(right_valid) if right_valid else float('inf')
        self.dist_front = min(front_valid) if front_valid else float('inf')

    def odom_callback(self, msg: Odometry):
        # Store the current speed from Odometry (used for detecting stuck state)
        self.actual_speed = msg.twist.twist.linear.x

    def control_loop(self):
        twist = Twist()

        # 1) Check if the robot is stuck: if front and sides are free but speed is near zero
        if self.dist_front > 1.0 and self.dist_right > 0.7 and self.actual_speed < 0.01:
            twist.linear.x = 0.0
            twist.angular.z = 0.8
            self.get_logger().warn('Stuck! Turning to free up...')
        else:
            # 2) Wall-following logic with PID on right-hand side
            if self.dist_right > 2 * self.wall_dist:
                # Searching for wall: Rotate right
                twist.linear.x = 0.0
                twist.angular.z = -self.turn_speed
                self.get_logger().info(f'Searching wall: dist_right={self.dist_right:.2f}')
            else:
                # PID to maintain wall distance
                error = self.wall_dist - self.dist_right
                self.integral += error * self.dt
                derivative = (error - self.prev_error) / self.dt
                control = self.kp * error + self.ki * self.integral + self.kd * derivative
                self.prev_error = error

                # Forward if front is clear, else rotate left
                if self.dist_front > self.wall_dist:
                    twist.linear.x = self.forward_speed
                    twist.angular.z = control
                    self.get_logger().info(f'Moving forward: dist_front={self.dist_front:.2f}')
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = self.turn_speed
                    self.get_logger().info(f'Obstacle in front: Turning left')

        # Publish the velocity command
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = SLAMAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()