#!/usr/bin/env python3

import numpy as np
import argparse
from pathlib import Path
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from collections import defaultdict
import math


def create_occupancy_grid(trajectory, scan_data, resolution=0.05, min_range=0.12, max_range=2.0,
                          occupied_threshold=0.6, free_threshold=0.4):
    if len(trajectory) == 0:
        return np.array([]), {}

    all_x = [pose[0] for pose in trajectory]
    all_y = [pose[1] for pose in trajectory]

    min_x = min(all_x) - 2.0
    max_x = max(all_x) + 2.0
    min_y = min(all_y) - 2.0
    max_y = max(all_y) + 2.0

    width = int(np.ceil((max_x - min_x) / resolution))
    height = int(np.ceil((max_y - min_y) / resolution))

    hit_count = defaultdict(int)
    miss_count = defaultdict(int)

    pose_skip = max(1, len(trajectory) // 500)

    total_poses = len(trajectory) // pose_skip
    print(f'Processing {total_poses} poses (skipping every {pose_skip})...')

    for idx, i in enumerate(range(0, len(trajectory), pose_skip)):
        if i >= len(scan_data):
            break

        if idx % 50 == 0:
            print(f'Progress: {idx}/{total_poses} poses ({100*idx//total_poses}%)')

        robot_x, robot_y, robot_theta = trajectory[i]
        ranges = scan_data[i]['ranges']
        angles = scan_data[i]['angles']

        angle_skip = max(1, len(ranges) // 100)

        for j in range(0, len(ranges), angle_skip):
            r = ranges[j]
            angle = angles[j]

            if not np.isfinite(r) or r < min_range or r > max_range:
                continue

            abs_angle = robot_theta + angle
            endpoint_x = robot_x + r * np.cos(abs_angle)
            endpoint_y = robot_y + r * np.sin(abs_angle)

            end_cell_x = int((endpoint_x - min_x) / resolution)
            end_cell_y = int((endpoint_y - min_y) / resolution)

            if 0 <= end_cell_x < width and 0 <= end_cell_y < height:
                hit_count[(end_cell_x, end_cell_y)] += 1

            robot_cell_x = int((robot_x - min_x) / resolution)
            robot_cell_y = int((robot_y - min_y) / resolution)

            dx = abs(end_cell_x - robot_cell_x)
            dy = abs(end_cell_y - robot_cell_y)
            x = robot_cell_x
            y = robot_cell_y

            x_inc = 1 if end_cell_x > robot_cell_x else -1
            y_inc = 1 if end_cell_y > robot_cell_y else -1

            if dx > dy:
                error = dx / 2
                for _ in range(dx):
                    if 0 <= x < width and 0 <= y < height:
                        miss_count[(x, y)] += 1
                    error -= dy
                    if error < 0:
                        y += y_inc
                        error += dx
                    x += x_inc
            else:
                error = dy / 2
                for _ in range(dy):
                    if 0 <= x < width and 0 <= y < height:
                        miss_count[(x, y)] += 1
                    error -= dx
                    if error < 0:
                        x += x_inc
                        error += dy
                    y += y_inc

    grid = np.full((height, width), -1, dtype=np.int8)

    for (x, y) in set(hit_count.keys()) | set(miss_count.keys()):
        hits = hit_count[(x, y)]
        misses = miss_count[(x, y)]
        total = hits + misses

        if total > 0:
            ratio = hits / total
            if ratio > occupied_threshold:
                grid[y, x] = 100
            elif ratio < free_threshold:
                grid[y, x] = 0

    metadata = {
        'width': width,
        'height': height,
        'resolution': resolution,
        'origin_x': min_x,
        'origin_y': min_y
    }

    return grid, metadata


def load_scans_from_bag(bag_path):
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    scans = []
    timestamps = []

    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        if topic == '/scan':
            msg = deserialize_message(data, LaserScan)
            ranges = np.array(msg.ranges)
            angles = np.arange(msg.angle_min, msg.angle_max + msg.angle_increment/2, msg.angle_increment)

            if len(angles) > len(ranges):
                angles = angles[:len(ranges)]
            elif len(ranges) > len(angles):
                ranges = ranges[:len(angles)]

            valid_indices = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
            ranges = ranges[valid_indices]
            angles = angles[valid_indices]

            scans.append({'ranges': ranges, 'angles': angles})
            timestamps.append(timestamp * 1e-9)

    return scans, timestamps


def synchronize_scans_with_trajectory(trajectory, trajectory_ts, scans, scan_ts):
    synced_scans = []
    scan_idx = 0

    for traj_ts in trajectory_ts:
        while scan_idx < len(scan_ts) - 1 and scan_ts[scan_idx + 1] <= traj_ts:
            scan_idx += 1

        if scan_idx < len(scans):
            synced_scans.append(scans[scan_idx])

    return synced_scans


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--sequence', type=str, required=True)
    args = parser.parse_args()

    script_dir = Path(__file__).parent
    data_dir = script_dir / 'data' / args.sequence
    bag_dir = script_dir / 'bags' / args.sequence

    print(f'Loading SLAM trajectory...')
    traj_file = data_dir / 'trajectories.npz'
    data = np.load(traj_file, allow_pickle=True)
    slam_traj = data['slam']
    slam_ts = data['slam_ts'][:, 0]

    print(f'Loading laser scans from bag...')
    scans, scan_ts = load_scans_from_bag(str(bag_dir))

    print(f'Synchronizing scans with trajectory...')
    synced_scans = synchronize_scans_with_trajectory(slam_traj, slam_ts, scans, scan_ts)

    print(f'Building occupancy grid...')
    grid, metadata = create_occupancy_grid(
        slam_traj,
        synced_scans,
        resolution=0.05,
        min_range=0.12,
        max_range=2.0,
        occupied_threshold=0.6,
        free_threshold=0.4
    )

    print(f'Saving custom SLAM map...')
    np.savez(
        data_dir / 'slam_map.npz',
        map=grid,
        metadata=metadata
    )

    print(f'Done! Grid size: {metadata["width"]}x{metadata["height"]}')


if __name__ == '__main__':
    main()
