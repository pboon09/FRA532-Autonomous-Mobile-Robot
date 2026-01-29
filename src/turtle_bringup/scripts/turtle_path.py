#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class TurtlePath(Node):
    def __init__(self):
        super().__init__('turtle_path')

        self.wheel_odom_path = Path()
        self.wheel_odom_path.header.frame_id = 'odom'

        self.ekf_path = Path()
        self.ekf_path.header.frame_id = 'odom'

        self.wheel_odom_sub = self.create_subscription(
            Odometry, '/wheel_odom', self.wheel_odom_callback, 10)
        self.ekf_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.ekf_callback, 10)

        self.wheel_odom_path_pub = self.create_publisher(Path, '/path/wheel_odom', 10)
        self.ekf_path_pub = self.create_publisher(Path, '/path/ekf', 10)

    def wheel_odom_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.wheel_odom_path.poses.append(pose)
        self.wheel_odom_path.header.stamp = msg.header.stamp
        self.wheel_odom_path_pub.publish(self.wheel_odom_path)

    def ekf_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.ekf_path.poses.append(pose)
        self.ekf_path.header.stamp = msg.header.stamp
        self.ekf_path_pub.publish(self.ekf_path)


def main(args=None):
    rclpy.init(args=args)
    node = TurtlePath()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
