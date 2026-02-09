#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
import numpy as np
import tf_transformations


class ICPMapper(Node):
    def __init__(self):
        super().__init__('turtle_icp_mapper')

        self.declare_parameter('map_resolution', 0.05)
        self.declare_parameter('map_update_interval', 10.0)
        self.declare_parameter('max_laser_range', 3.5)
        self.declare_parameter('min_laser_range', 0.12)
        self.declare_parameter('map_size_buffer', 100)

        self.map_resolution = self.get_parameter('map_resolution').value
        self.map_update_interval = self.get_parameter('map_update_interval').value
        self.max_laser_range = self.get_parameter('max_laser_range').value
        self.min_laser_range = self.get_parameter('min_laser_range').value
        self.map_size_buffer = self.get_parameter('map_size_buffer').value

        self.hit_count = {}
        self.miss_count = {}

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        self.min_x = float('inf')
        self.max_x = float('-inf')
        self.min_y = float('inf')
        self.max_y = float('-inf')

        self.current_map_min_x = 0.0
        self.current_map_max_x = 0.0
        self.current_map_min_y = 0.0
        self.current_map_max_y = 0.0
        self.map_initialized = False

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/icp', self.odom_callback, 10)

        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.map_timer = self.create_timer(self.map_update_interval, self.publish_map)

        self.get_logger().info(f'ICP Mapper initialized | Resolution: {self.map_resolution}m | Update: {self.map_update_interval}s | Range: {self.min_laser_range}-{self.max_laser_range}m')

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.robot_theta = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

    def scan_callback(self, msg):
        self.update_map(msg)

    def world_to_map(self, x, y):
        map_x = int(np.floor(x / self.map_resolution))
        map_y = int(np.floor(y / self.map_resolution))
        return map_x, map_y

    def update_map(self, scan):
        robot_mx, robot_my = self.world_to_map(self.robot_x, self.robot_y)

        ranges = np.array(scan.ranges)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment

        valid = np.isfinite(ranges) & (ranges > self.min_laser_range) & (ranges < self.max_laser_range)
        ranges = ranges[valid]
        angles = angles[valid]

        for r, angle in zip(ranges, angles):
            global_angle = self.robot_theta + angle
            end_x = self.robot_x + r * np.cos(global_angle)
            end_y = self.robot_y + r * np.sin(global_angle)

            end_mx, end_my = self.world_to_map(end_x, end_y)

            self.hit_count[(end_mx, end_my)] = self.hit_count.get((end_mx, end_my), 0) + 1
            self.min_x = min(self.min_x, end_x)
            self.max_x = max(self.max_x, end_x)
            self.min_y = min(self.min_y, end_y)
            self.max_y = max(self.max_y, end_y)

            ray_cells = self.bresenham(robot_mx, robot_my, end_mx, end_my)
            for cx, cy in ray_cells[:-1]:
                self.miss_count[(cx, cy)] = self.miss_count.get((cx, cy), 0) + 1

    def bresenham(self, x0, y0, x1, y1):
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        x, y = x0, y0
        while True:
            cells.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

        return cells

    def publish_map(self):
        if len(self.hit_count) == 0:
            return

        buffer = self.map_size_buffer
        needs_resize = False

        if not self.map_initialized:
            self.current_map_min_x = self.min_x - buffer * self.map_resolution
            self.current_map_max_x = self.max_x + buffer * self.map_resolution
            self.current_map_min_y = self.min_y - buffer * self.map_resolution
            self.current_map_max_y = self.max_y + buffer * self.map_resolution
            self.map_initialized = True
            needs_resize = True
        else:
            if self.min_x < self.current_map_min_x + buffer * self.map_resolution:
                self.current_map_min_x = self.min_x - buffer * self.map_resolution
                needs_resize = True
            if self.max_x > self.current_map_max_x - buffer * self.map_resolution:
                self.current_map_max_x = self.max_x + buffer * self.map_resolution
                needs_resize = True
            if self.min_y < self.current_map_min_y + buffer * self.map_resolution:
                self.current_map_min_y = self.min_y - buffer * self.map_resolution
                needs_resize = True
            if self.max_y > self.current_map_max_y - buffer * self.map_resolution:
                self.current_map_max_y = self.max_y + buffer * self.map_resolution
                needs_resize = True

        if needs_resize:
            self.get_logger().info(f'Map resized to cover area: '
                                   f'[{self.current_map_min_x:.1f}, {self.current_map_max_x:.1f}] x '
                                   f'[{self.current_map_min_y:.1f}, {self.current_map_max_y:.1f}]')

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'

        min_mx = int(self.current_map_min_x / self.map_resolution)
        max_mx = int(self.current_map_max_x / self.map_resolution)
        min_my = int(self.current_map_min_y / self.map_resolution)
        max_my = int(self.current_map_max_y / self.map_resolution)

        width = max_mx - min_mx + 1
        height = max_my - min_my + 1

        msg.info.resolution = self.map_resolution
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = min_mx * self.map_resolution
        msg.info.origin.position.y = min_my * self.map_resolution
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        map_data = np.full((height, width), -1, dtype=np.int8)

        all_cells = set(self.hit_count.keys()) | set(self.miss_count.keys())
        for (mx, my) in all_cells:
            if min_mx <= mx <= max_mx and min_my <= my <= max_my:
                hits = self.hit_count.get((mx, my), 0)
                misses = self.miss_count.get((mx, my), 0)
                total = hits + misses

                if total > 0:
                    hit_ratio = hits / total
                    grid_x = mx - min_mx
                    grid_y = my - min_my

                    if hit_ratio > 0.6:
                        map_data[grid_y, grid_x] = 100
                    elif hit_ratio < 0.4:
                        map_data[grid_y, grid_x] = 0

        msg.data = map_data.flatten().tolist()
        self.map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ICPMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
