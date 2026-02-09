#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


class TurtlePath(Node):
    def __init__(self):
        super().__init__('turtle_path')

        self.wheel_odom_path = Path()
        self.wheel_odom_path.header.frame_id = 'odom'

        self.ekf_path = Path()
        self.ekf_path.header.frame_id = 'odom'

        self.icp_path = Path()
        self.icp_path.header.frame_id = 'odom'

        self.slam_path = Path()
        self.slam_path.header.frame_id = 'map'

        self.wheel_odom_sub = self.create_subscription(
            Odometry, '/odometry/wheel_odom', self.wheel_odom_callback, 10)
        self.ekf_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.ekf_callback, 10)
        self.icp_sub = self.create_subscription(
            Odometry, '/odometry/icp_keyframes', self.icp_callback, 10)
        self.slam_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/pose', self.slam_pose_callback, 10)

        self.wheel_odom_path_pub = self.create_publisher(Path, '/path/wheel_odom', 10)
        self.ekf_path_pub = self.create_publisher(Path, '/path/ekf', 10)
        self.icp_path_pub = self.create_publisher(Path, '/path/icp', 10)
        self.slam_path_pub = self.create_publisher(Path, '/path/slam', 10)

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

    def icp_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.icp_path.poses.append(pose)
        self.icp_path.header.stamp = msg.header.stamp
        self.icp_path_pub.publish(self.icp_path)

    def slam_pose_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.slam_path.poses.append(pose)
        self.slam_path.header.stamp = msg.header.stamp
        self.slam_path_pub.publish(self.slam_path)


def main(args=None):
    rclpy.init(args=args)
    node = TurtlePath()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
