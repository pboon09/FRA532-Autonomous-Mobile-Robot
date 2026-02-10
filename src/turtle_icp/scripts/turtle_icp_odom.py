#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import numpy as np
from tf2_ros import TransformBroadcaster
import tf_transformations
from scipy.spatial import KDTree
from collections import deque
import time
import struct


class PointToPointICP:
    def __init__(self, max_correspondence_distance=0.5, max_iteration=50,
                 tolerance=1e-6, outlier_rejection_percentile=80):
        self.max_correspondence_distance = max_correspondence_distance
        self.max_iteration = max_iteration
        self.tolerance = tolerance
        self.outlier_rejection_percentile = outlier_rejection_percentile
        self.min_correspondences = 30

    def register_scan_to_map(self, scan_points_body, map_points_odom, init_x, init_y, init_theta):
        start_time = time.time()
        x, y, theta = init_x, init_y, init_theta

        scan_2d = scan_points_body[:, :2] if scan_points_body.shape[1] > 2 else scan_points_body
        map_2d = map_points_odom[:, :2] if map_points_odom.shape[1] > 2 else map_points_odom

        map_tree = KDTree(map_2d)
        prev_error = float('inf')
        iteration = 0

        for iteration in range(self.max_iteration):
            c = np.cos(theta)
            s = np.sin(theta)
            transformed = np.column_stack([
                scan_2d[:, 0] * c - scan_2d[:, 1] * s + x,
                scan_2d[:, 0] * s + scan_2d[:, 1] * c + y
            ])

            dists, indices = map_tree.query(transformed)
            valid = (dists < self.max_correspondence_distance) & np.isfinite(dists)
            n_valid = np.sum(valid)

            if n_valid < self.min_correspondences:
                return self._fail_result(init_x, init_y, init_theta, iteration, start_time)

            if n_valid > 30:
                valid_dists = dists[valid]
                trim_thresh = np.percentile(valid_dists, self.outlier_rejection_percentile)
                trim_mask = np.zeros(len(dists), dtype=bool)
                trim_mask[valid] = dists[valid] <= trim_thresh
                valid = trim_mask
                n_valid = np.sum(valid)
                if n_valid < self.min_correspondences:
                    return self._fail_result(init_x, init_y, init_theta, iteration, start_time)

            mean_error = np.mean(dists[valid])
            if np.isnan(mean_error) or abs(prev_error - mean_error) < self.tolerance:
                break
            prev_error = mean_error

            src_m = scan_2d[valid]
            tgt_m = map_2d[indices[valid]]
            src_c = np.mean(src_m, axis=0)
            tgt_c = np.mean(tgt_m, axis=0)
            W = (tgt_m - tgt_c).T @ (src_m - src_c)

            try:
                U, _, Vt = np.linalg.svd(W)
                R_opt = U @ np.diag([1.0, np.sign(np.linalg.det(U @ Vt))]) @ Vt
                t_opt = tgt_c - R_opt @ src_c
                theta = np.arctan2(R_opt[1, 0], R_opt[0, 0])
                x, y = t_opt[0], t_opt[1]
            except np.linalg.LinAlgError:
                break

        c, s = np.cos(theta), np.sin(theta)
        final_transformed = np.column_stack([
            scan_2d[:, 0] * c - scan_2d[:, 1] * s + x,
            scan_2d[:, 0] * s + scan_2d[:, 1] * c + y
        ])

        final_dists, _ = map_tree.query(final_transformed)
        final_valid = (final_dists < self.max_correspondence_distance) & np.isfinite(final_dists)
        inlier_rmse = np.sqrt(np.mean(final_dists[final_valid] ** 2)) if np.sum(final_valid) > 0 else 1.0
        fitness = np.sum(final_valid) / len(scan_2d) if len(scan_2d) > 0 else 0.0

        return {
            'x': x, 'y': y, 'theta': theta,
            'fitness': fitness, 'inlier_rmse': inlier_rmse,
            'iterations': iteration + 1, 'runtime_ms': (time.time() - start_time) * 1000.0,
            'success': True
        }

    def _fail_result(self, x, y, theta, iteration, start_time):
        return {
            'x': x, 'y': y, 'theta': theta,
            'fitness': 0.0, 'inlier_rmse': 1.0,
            'iterations': iteration + 1, 'runtime_ms': (time.time() - start_time) * 1000.0,
            'success': False
        }


class LidarProcessor:
    def __init__(self, voxel_size=0.05, translation_threshold=0.15, rotation_threshold=0.087):
        self.voxel_size = voxel_size
        self.translation_threshold = translation_threshold
        self.rotation_threshold = rotation_threshold
        self.last_keyframe_pose = None

    def scan_to_pointcloud(self, ranges, angles):
        ranges = np.array(ranges, dtype=np.float64)
        angles = np.array(angles, dtype=np.float64)
        valid = (np.isfinite(ranges) & (ranges > 0.12) & (ranges < 3.5))
        ranges, angles = ranges[valid], angles[valid]
        if len(ranges) == 0:
            return np.empty((0, 2))
        return np.column_stack((ranges * np.cos(angles), ranges * np.sin(angles)))

    def is_keyframe(self, current_pose):
        if self.last_keyframe_pose is None:
            self.last_keyframe_pose = current_pose
            return True
        dx = current_pose[0] - self.last_keyframe_pose[0]
        dy = current_pose[1] - self.last_keyframe_pose[1]
        dtheta = abs(np.arctan2(np.sin(current_pose[2] - self.last_keyframe_pose[2]),
                                np.cos(current_pose[2] - self.last_keyframe_pose[2])))
        if np.sqrt(dx**2 + dy**2) > self.translation_threshold or dtheta > self.rotation_threshold:
            self.last_keyframe_pose = current_pose
            return True
        return False

    def voxel_downsample(self, points, voxel_size):
        if len(points) == 0 or voxel_size <= 0:
            return points
        voxel_indices = np.floor(points / voxel_size).astype(int)
        voxel_dict = {}
        for i in range(len(points)):
            key = (voxel_indices[i, 0], voxel_indices[i, 1])
            if key not in voxel_dict:
                voxel_dict[key] = []
            voxel_dict[key].append(points[i])
        return np.array([np.mean(pts, axis=0) for pts in voxel_dict.values()])


class ICPOdometry(Node):
    def __init__(self):
        super().__init__('turtle_icp_odom')

        self.declare_parameter('local_map_size', 15)
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('keyframe_dist_thresh', 0.3)
        self.declare_parameter('keyframe_angle_thresh', 0.174533)
        self.declare_parameter('max_scan_points', 300)
        self.declare_parameter('max_translation_correction', 0.3)
        self.declare_parameter('max_rotation_correction', 0.087266)
        self.declare_parameter('publish_tf', True)

        self.local_map_size = self.get_parameter('local_map_size').value
        self.voxel_size = self.get_parameter('voxel_size').value
        self.keyframe_dist_thresh = self.get_parameter('keyframe_dist_thresh').value
        self.keyframe_angle_thresh = self.get_parameter('keyframe_angle_thresh').value
        self.max_scan_points = self.get_parameter('max_scan_points').value
        self.max_translation_correction = self.get_parameter('max_translation_correction').value
        self.max_rotation_correction = self.get_parameter('max_rotation_correction').value
        self.publish_tf = self.get_parameter('publish_tf').value

        self.icp = PointToPointICP(
            max_correspondence_distance=0.5, max_iteration=50,
            tolerance=1e-6, outlier_rejection_percentile=80
        )
        self.lidar_processor = LidarProcessor(
            voxel_size=self.voxel_size,
            translation_threshold=self.keyframe_dist_thresh,
            rotation_threshold=self.keyframe_angle_thresh
        )

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.local_map_scans = deque(maxlen=self.local_map_size)
        self.local_map_points = None
        self.local_map_dirty = True

        self.prev_ekf_x = None
        self.prev_ekf_y = None
        self.prev_ekf_theta = None

        self.update_count = 0
        self.icp_success_count = 0
        self.icp_reject_count = 0
        self.keyframe_count = 0
        self.last_kf_pose = [0.0, 0.0, 0.0]
        self.accumulated_dx = 0.0
        self.accumulated_dy = 0.0
        self.accumulated_dtheta = 0.0

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.ekf_odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.ekf_odom_callback, 10)

        self.odom_pub = self.create_publisher(Odometry, '/odometry/icp', 10)
        self.odom_keyframe_pub = self.create_publisher(Odometry, '/odometry/icp_keyframes', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.start_time = None
        self.get_logger().info(f'ICP Odometry initialized | Map: {self.local_map_size}kf | Voxel: {self.voxel_size}m | TF: {self.publish_tf}')

    def ekf_odom_callback(self, msg):
        self.latest_ekf_odom = msg

    def scan_callback(self, msg):
        current_time = self.get_clock().now()
        if self.start_time is None:
            self.start_time = current_time
            self.publish_odom_and_tf(current_time)

        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        current_points = self.lidar_processor.scan_to_pointcloud(ranges, angles)
        if len(current_points) < 10:
            return

        if not hasattr(self, 'latest_ekf_odom'):
            return

        ekf_x, ekf_y, ekf_theta = self.extract_pose(self.latest_ekf_odom)

        if self.prev_ekf_x is None:
            self.prev_ekf_x, self.prev_ekf_y, self.prev_ekf_theta = ekf_x, ekf_y, ekf_theta
            if len(current_points) >= 10:
                odom_points = self.transform_to_odom(current_points, self.x, self.y, self.theta)
                self.local_map_scans.append(odom_points)
                self.local_map_dirty = True
                self.keyframe_count = 1
                self.last_kf_pose = [self.x, self.y, self.theta]
                self.accumulated_dx = 0.0
                self.accumulated_dy = 0.0
                self.accumulated_dtheta = 0.0
                self.publish_odom_and_tf(current_time, is_keyframe=True)
            return

        ekf_dx = ekf_x - self.prev_ekf_x
        ekf_dy = ekf_y - self.prev_ekf_y
        ekf_dtheta = np.arctan2(np.sin(ekf_theta - self.prev_ekf_theta),
                                np.cos(ekf_theta - self.prev_ekf_theta))

        self.accumulated_dx += ekf_dx
        self.accumulated_dy += ekf_dy
        self.accumulated_dtheta += ekf_dtheta
        self.accumulated_dtheta = np.arctan2(np.sin(self.accumulated_dtheta), np.cos(self.accumulated_dtheta))

        self.x = self.last_kf_pose[0] + self.accumulated_dx
        self.y = self.last_kf_pose[1] + self.accumulated_dy
        self.theta = self.last_kf_pose[2] + self.accumulated_dtheta
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))

        dist_from_kf = np.sqrt(self.accumulated_dx**2 + self.accumulated_dy**2)
        angle_from_kf = abs(self.accumulated_dtheta)
        is_keyframe = dist_from_kf > self.keyframe_dist_thresh or angle_from_kf > self.keyframe_angle_thresh
        if is_keyframe:
            if len(self.local_map_scans) >= 3:
                if self.local_map_dirty:
                    self.rebuild_local_map()

                if self.local_map_points is not None and len(self.local_map_points) > 50:
                    try:
                        result = self.icp.register_scan_to_map(current_points, self.local_map_points,
                                                                self.x, self.y, self.theta)
                        if result['success']:
                            corr_t = np.sqrt((result['x'] - self.x)**2 + (result['y'] - self.y)**2)
                            corr_r = abs(np.arctan2(np.sin(result['theta'] - self.theta),
                                                     np.cos(result['theta'] - self.theta)))
                            if corr_t < self.max_translation_correction and corr_r < self.max_rotation_correction:
                                self.x, self.y, self.theta = result['x'], result['y'], result['theta']
                                self.icp_success_count += 1
                            else:
                                self.icp_reject_count += 1
                        else:
                            self.icp_reject_count += 1
                    except Exception as e:
                        self.get_logger().error(f'ICP error: {e}', throttle_duration_sec=5.0)
                        self.icp_reject_count += 1

            if len(current_points) >= 10:
                odom_points = self.transform_to_odom(current_points, self.x, self.y, self.theta)
                self.local_map_scans.append(odom_points)
                self.local_map_dirty = True
                self.keyframe_count += 1
                self.last_kf_pose = [self.x, self.y, self.theta]
                self.accumulated_dx = 0.0
                self.accumulated_dy = 0.0
                self.accumulated_dtheta = 0.0

        self.publish_odom_and_tf(current_time, is_keyframe=is_keyframe)
        # self.get_logger().info(f'pose x={self.x:.3f} y={self.y:.3f} theta={self.theta:.3f}', throttle_duration_sec=1.0)
        self.prev_ekf_x, self.prev_ekf_y, self.prev_ekf_theta = ekf_x, ekf_y, ekf_theta

        self.update_count += 1
        if self.update_count % 50 == 0:
            elapsed = (current_time - self.start_time).nanoseconds / 1e9
            diff = np.sqrt((self.x - ekf_x)**2 + (self.y - ekf_y)**2)
            # self.get_logger().info(
            #     f'[{elapsed:.1f}s] #{self.update_count} | '
            #     f'ICP: ({self.x:.3f}, {self.y:.3f}) θ={np.degrees(self.theta):.1f}° | '
            #     f'EKF diff: {diff:.3f}m | '
            #     f'ok: {self.icp_success_count} rej: {self.icp_reject_count} '
            #     f'kf: {self.keyframe_count} map: {len(self.local_map_scans)}')

    def rebuild_local_map(self):
        if len(self.local_map_scans) == 0:
            self.local_map_points = None
            return
        all_points = np.vstack(list(self.local_map_scans))
        if self.voxel_size > 0:
            all_points = self.lidar_processor.voxel_downsample(all_points, self.voxel_size)
        self.local_map_points = all_points
        self.local_map_dirty = False

    def transform_to_odom(self, points, x, y, theta):
        c, s = np.cos(theta), np.sin(theta)
        return np.column_stack([
            points[:, 0] * c - points[:, 1] * s + x,
            points[:, 0] * s + points[:, 1] * c + y
        ])

    def extract_pose(self, odom_msg):
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        q = odom_msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        return x, y, yaw

    def publish_odom_and_tf(self, timestamp, is_keyframe=False):
        q = tf_transformations.quaternion_from_euler(0, 0, self.theta)

        odom = Odometry()
        odom.header.stamp = timestamp.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        if is_keyframe:
            self.odom_keyframe_pub.publish(odom)
            
        self.odom_pub.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = timestamp.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_footprint'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ICPOdometry()
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
