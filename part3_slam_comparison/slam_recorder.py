#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from pathlib import Path
import subprocess
import signal
import argparse
import time


class SlamRecorder(Node):
    def __init__(self, sequence_name):
        super().__init__('slam_recorder')

        self.sequence_name = sequence_name
        self.bag_dir = Path(__file__).parent / 'bags'
        self.bag_dir.mkdir(parents=True, exist_ok=True)

        self.bag_process = None
        self.last_msg_time = time.time()
        self.timeout = 5.0

        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/wheel_odom', self.msg_callback, 10)

        self.timer = self.create_timer(1.0, self.check_timeout)

        self.start_bag_recording()
        self.get_logger().info(f'Recording started for {sequence_name}')
        self.get_logger().info(f'Will auto-stop after {self.timeout}s of no messages')

    def msg_callback(self, msg):
        self.last_msg_time = time.time()

    def start_bag_recording(self):
        topics = [
            '/odometry/wheel_odom',
            '/odometry/filtered',
            '/odometry/icp',
            '/odometry/icp_keyframes',
            '/map',
            '/map_icp',
            '/scan',
            '/tf',
            '/tf_static'
        ]

        cmd = ['ros2', 'bag', 'record', '-o', str(self.bag_dir / self.sequence_name), '--storage', 'sqlite3'] + topics

        self.bag_process = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def check_timeout(self):
        if time.time() - self.last_msg_time > self.timeout:
            self.get_logger().info(f'No messages for {self.timeout}s. Stopping...')
            self.stop_and_exit()

    def stop_and_exit(self):
        if self.bag_process is not None:
            self.get_logger().info('Stopping bag recording...')
            self.bag_process.terminate()
            try:
                self.bag_process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.bag_process.kill()
                self.bag_process.wait()
            self.get_logger().info(f'Saved to {self.bag_dir}/{self.sequence_name}/')
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='Record SLAM data as bag file')
    parser.add_argument('--sequence', type=str, required=True, help='Sequence name (e.g., seq00)')
    args = parser.parse_args()

    rclpy.init()
    node = SlamRecorder(args.sequence)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop_and_exit()


if __name__ == '__main__':
    main()
