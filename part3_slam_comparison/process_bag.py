#!/usr/bin/env python3

import numpy as np
import argparse
from pathlib import Path
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry, OccupancyGrid
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion
import math
from utils import save_json, compute_drift_metrics, compute_trajectory_errors, compute_map_metrics


class BagProcessor:
    def __init__(self, bag_path):
        self.bag_path = bag_path
        self.wheel_data = []
        self.ekf_data = []
        self.icp_data = []
        self.slam_data = []
        self.imu_data = []
        self.slam_map = None
        self.slam_map_metadata = None
        self.icp_map = None
        self.icp_map_metadata = None
        self.map_to_odom_buffer = []
        self.odom_to_base_buffer = []
        self.imu_offset = None

    def process(self):
        storage_options = StorageOptions(uri=self.bag_path, storage_id='sqlite3')
        converter_options = ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )

        reader = SequentialReader()
        reader.open(storage_options, converter_options)

        print('Reading bag file...')
        while reader.has_next():
            topic, data, timestamp = reader.read_next()

            if topic == '/odometry/wheel_odom':
                msg = deserialize_message(data, Odometry)
                self.wheel_data.append(self._parse_odometry(msg, timestamp))

            elif topic == '/odometry/filtered':
                msg = deserialize_message(data, Odometry)
                self.ekf_data.append(self._parse_odometry(msg, timestamp))

            elif topic == '/odometry/icp_keyframes':
                msg = deserialize_message(data, Odometry)
                self.icp_data.append(self._parse_odometry(msg, timestamp))

            elif topic == '/imu':
                msg = deserialize_message(data, Imu)
                self._parse_imu(msg, timestamp)

            elif topic == '/tf':
                msg = deserialize_message(data, TFMessage)
                self._buffer_transforms(msg, timestamp)

            elif topic == '/map':
                msg = deserialize_message(data, OccupancyGrid)
                self.slam_map, self.slam_map_metadata = self._parse_occupancy_grid(msg)

            elif topic == '/map_icp':
                msg = deserialize_message(data, OccupancyGrid)
                self.icp_map, self.icp_map_metadata = self._parse_occupancy_grid(msg)

        print('Synchronizing SLAM poses...')
        self._synchronize_slam_poses()

    def _parse_odometry(self, msg, timestamp):
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        return {
            'timestamp': timestamp * 1e-9,
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'theta': yaw
        }

    def _parse_imu(self, msg, timestamp):
        q = msg.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        if self.imu_offset is None:
            self.imu_offset = yaw

        corrected_yaw = yaw - self.imu_offset
        corrected_yaw = math.atan2(math.sin(corrected_yaw), math.cos(corrected_yaw))

        self.imu_data.append({
            'timestamp': timestamp * 1e-9,
            'yaw': corrected_yaw
        })

    def _buffer_transforms(self, msg, timestamp):
        for transform in msg.transforms:
            if transform.header.frame_id == 'map' and transform.child_frame_id == 'odom':
                t = transform.transform.translation
                q = transform.transform.rotation
                _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
                self.map_to_odom_buffer.append({
                    'timestamp': timestamp * 1e-9,
                    'x': t.x,
                    'y': t.y,
                    'theta': yaw
                })
            elif transform.header.frame_id == 'odom' and transform.child_frame_id == 'base_footprint':
                t = transform.transform.translation
                q = transform.transform.rotation
                _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
                self.odom_to_base_buffer.append({
                    'timestamp': timestamp * 1e-9,
                    'x': t.x,
                    'y': t.y,
                    'theta': yaw
                })

    def _synchronize_slam_poses(self):
        if not self.map_to_odom_buffer or not self.odom_to_base_buffer:
            return

        odom_idx = 0
        for map_tf in self.map_to_odom_buffer:
            while odom_idx < len(self.odom_to_base_buffer) - 1:
                if self.odom_to_base_buffer[odom_idx + 1]['timestamp'] > map_tf['timestamp']:
                    break
                odom_idx += 1

            if odom_idx < len(self.odom_to_base_buffer):
                odom_tf = self.odom_to_base_buffer[odom_idx]

                cos_yaw1 = math.cos(map_tf['theta'])
                sin_yaw1 = math.sin(map_tf['theta'])

                x = map_tf['x'] + cos_yaw1 * odom_tf['x'] - sin_yaw1 * odom_tf['y']
                y = map_tf['y'] + sin_yaw1 * odom_tf['x'] + cos_yaw1 * odom_tf['y']
                theta = map_tf['theta'] + odom_tf['theta']
                theta = math.atan2(math.sin(theta), math.cos(theta))

                self.slam_data.append({
                    'timestamp': map_tf['timestamp'],
                    'x': x,
                    'y': y,
                    'theta': theta
                })

    def _parse_occupancy_grid(self, msg):
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        grid = np.array(msg.data, dtype=np.int8).reshape((height, width))

        metadata = {
            'width': width,
            'height': height,
            'resolution': resolution,
            'origin_x': origin_x,
            'origin_y': origin_y
        }

        return grid, metadata

    def save_to_npz(self, output_dir, sequence_name):
        seq_dir = Path(output_dir) / sequence_name
        seq_dir.mkdir(parents=True, exist_ok=True)

        wheel_array = np.array([[d['x'], d['y'], d['theta']] for d in self.wheel_data])
        ekf_array = np.array([[d['x'], d['y'], d['theta']] for d in self.ekf_data])
        icp_array = np.array([[d['x'], d['y'], d['theta']] for d in self.icp_data])
        slam_array = np.array([[d['x'], d['y'], d['theta']] for d in self.slam_data])

        wheel_ts = np.array([[d['timestamp'], d['x'], d['y'], d['theta']] for d in self.wheel_data])
        ekf_ts = np.array([[d['timestamp'], d['x'], d['y'], d['theta']] for d in self.ekf_data])
        icp_ts = np.array([[d['timestamp'], d['x'], d['y'], d['theta']] for d in self.icp_data])
        slam_ts = np.array([[d['timestamp'], d['x'], d['y'], d['theta']] for d in self.slam_data])

        np.savez(
            seq_dir / 'trajectories.npz',
            wheel=wheel_array,
            ekf=ekf_array,
            icp=icp_array,
            slam=slam_array,
            wheel_ts=wheel_ts,
            ekf_ts=ekf_ts,
            icp_ts=icp_ts,
            slam_ts=slam_ts
        )

        if self.slam_map is not None:
            np.savez(
                seq_dir / 'slam_map.npz',
                map=self.slam_map,
                metadata=self.slam_map_metadata
            )

        if self.icp_map is not None:
            np.savez(
                seq_dir / 'icp_map.npz',
                map=self.icp_map,
                metadata=self.icp_map_metadata
            )

        summary = {
            'sequence': sequence_name,
            'methods': {},
            'comparisons': {}
        }

        trajectories = {
            'Wheel': wheel_array,
            'EKF': ekf_array,
            'ICP': icp_array,
            'SLAM': slam_array
        }

        final_imu_yaw = None
        if len(self.imu_data) > 0:
            final_imu_yaw = self.imu_data[-1]['yaw']

        for name, traj_array in trajectories.items():
            if len(traj_array) > 0:
                drift_metrics = compute_drift_metrics(traj_array)
                summary['methods'][name] = {
                    'num_poses': len(traj_array),
                    'final_pose': {
                        'x': float(traj_array[-1][0]),
                        'y': float(traj_array[-1][1]),
                        'theta_deg': float(math.degrees(traj_array[-1][2]))
                    },
                    **drift_metrics
                }

                if final_imu_yaw is not None:
                    final_heading = traj_array[-1][2]
                    heading_dev = abs(math.atan2(math.sin(final_heading - final_imu_yaw),
                                                 math.cos(final_heading - final_imu_yaw)))
                    summary['methods'][name]['heading_deviation_from_imu_deg'] = math.degrees(heading_dev)

        if len(slam_array) > 0:
            for name, traj_array in trajectories.items():
                if name != 'SLAM' and len(traj_array) > 0:
                    mean_err, max_err = compute_trajectory_errors(slam_array, traj_array)
                    if mean_err is not None:
                        summary['comparisons'][f'{name}_vs_SLAM'] = {
                            'mean_error_m': mean_err,
                            'max_error_m': max_err
                        }

        if self.slam_map is not None:
            slam_metrics = compute_map_metrics(self.slam_map)
            if slam_metrics:
                summary['slam_map_metrics'] = slam_metrics

        if self.icp_map is not None:
            icp_metrics = compute_map_metrics(self.icp_map)
            if icp_metrics:
                summary['icp_map_metrics'] = icp_metrics

        if self.slam_map is not None and self.icp_map is not None:
            from utils import compute_common_boundary_metrics
            common_metrics = compute_common_boundary_metrics(
                self.icp_map, self.icp_map_metadata,
                self.slam_map, self.slam_map_metadata
            )
            if common_metrics:
                summary['common_boundary_comparison'] = common_metrics

        save_json(summary, seq_dir / 'summary.json')

        return summary


def main():
    parser = argparse.ArgumentParser(description='Process bag file and extract trajectories')
    parser.add_argument('--sequence', type=str, required=True, help='Sequence name (e.g., seq00)')
    args = parser.parse_args()

    script_dir = Path(__file__).parent
    bag_dir = script_dir / 'bags'
    data_dir = script_dir / 'data'

    bag_path = bag_dir / args.sequence
    if not bag_path.exists():
        print(f'Bag not found: {bag_path}')
        return

    print(f'Processing bag: {bag_path}')
    processor = BagProcessor(str(bag_path))
    processor.process()

    print(f'Saving to NPZ...')
    summary = processor.save_to_npz(data_dir, args.sequence)

    print(f'\nSummary:')
    for method, info in summary['methods'].items():
        print(f'  {method}: {info["num_poses"]} poses, '
              f'drift: {info["drift_rate_percent"]:.2f}%, '
              f'length: {info["trajectory_length_m"]:.3f}m')

    print(f'\nDone! Files saved to {data_dir}/')


if __name__ == '__main__':
    main()
