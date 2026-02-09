#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
import math

class TurtleWheelOdometry(Node):
    def __init__(self):
        super().__init__('turtle_wheel_odometry')
        self.declare_parameter('wheel_radius', 0.033)
        self.declare_parameter('track_width', 0.160)
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.track_width = self.get_parameter('track_width').get_parameter_value().double_value

        self.last_rad_l = None
        self.last_rad_r = None
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.rate = 20.0
        self.last_time = None

        self.odom_pub = self.create_publisher(Odometry, '/odometry/wheel_odom', 10)
        self.sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.timer = self.create_timer(1.0/self.rate, self.timer_callback)

        self.latest_odom = None
        self.latest_joint_data = None
        
        self.get_logger().info('turtle_wheel_odometry node is started!')

    def joint_state_callback(self, msg):
        try:
            left_idx = msg.name.index('wheel_left_joint')
            right_idx = msg.name.index('wheel_right_joint')
            rad_l = msg.position[left_idx]
            rad_r = msg.position[right_idx]
            omega_l = msg.velocity[left_idx] if len(msg.velocity) > left_idx else 0.0
            omega_r = msg.velocity[right_idx] if len(msg.velocity) > right_idx else 0.0
            stamp = msg.header.stamp
        except (ValueError, IndexError):
            return
        self.latest_joint_data = {
            'rad_l': rad_l,
            'rad_r': rad_r,
            'omega_l': omega_l,
            'omega_r': omega_r,
            'stamp': stamp
        }

    def timer_callback(self):
        data = self.latest_joint_data
        if data is None:
            return

        rad_l = data['rad_l']
        rad_r = data['rad_r']
        omega_l = data['omega_l']
        omega_r = data['omega_r']
        stamp = data['stamp']
        now = stamp.sec + stamp.nanosec * 1e-9

        if self.last_rad_l is None or self.last_rad_r is None or self.last_time is None:
            self.last_rad_l = rad_l
            self.last_rad_r = rad_r
            self.last_time = now
            return

        dt = now - self.last_time
        if dt <= 0:
            return

        r = self.wheel_radius
        b = self.track_width

        delta_rad_l = rad_l - self.last_rad_l
        delta_rad_r = rad_r - self.last_rad_r

        d_l = delta_rad_l * r
        d_r = delta_rad_r * r

        d_theta = (d_r - d_l) / b

        if abs(d_r - d_l) < 1e-6:
            d_center = (d_l + d_r) / 2.0
            self.x += d_center * math.cos(self.theta)
            self.y += d_center * math.sin(self.theta)
        else:
            R = (b / 2.0) * (d_l + d_r) / (d_r - d_l)

            icc_x = self.x - R * math.sin(self.theta)
            icc_y = self.y + R * math.cos(self.theta)

            cos_dtheta = math.cos(d_theta)
            sin_dtheta = math.sin(d_theta)

            new_x = cos_dtheta * (self.x - icc_x) - sin_dtheta * (self.y - icc_y) + icc_x
            new_y = sin_dtheta * (self.x - icc_x) + cos_dtheta * (self.y - icc_y) + icc_y

            self.x = new_x
            self.y = new_y

        self.theta += d_theta

        v = (d_l + d_r) / (2.0 * dt)
        w = (d_r - d_l) / (b * dt)

        self.publish_odom(stamp, self.x, self.y, self.theta, v, w)
        self.last_rad_l = rad_l
        self.last_rad_r = rad_r
        self.last_time = now

    def publish_odom(self, stamp, x, y, theta, v, w):
        q = quaternion_from_euler(0, 0, theta)
        odom = Odometry()

        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = w

        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleWheelOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
