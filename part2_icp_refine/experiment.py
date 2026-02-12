#!/usr/bin/env python3

import os
import sys
import math
import numpy as np
from pathlib import Path
from collections import deque
import open3d as o3d

part1_path = Path(__file__).parent.parent / 'part1_ekf_odom'
sys.path.insert(0, str(part1_path))
from ekf import EKF
from wheel_odometry import WheelOdometry
sys.path.pop(0)

from bag_reader import BagReader
from lidar_processor import LidarProcessor
from icp_point_to_point import PointToPointICP
from utils import (
    save_figure, save_json, save_csv,
    plot_all_trajectories, plot_map_with_scans,
    compute_trajectory_drift, compute_convergence_rate, compute_consistency_score,
    create_occupancy_grid, plot_occupancy_grid
)


class Experiment:
    def __init__(self, bag_path, output_dir, sequence_name):
        self.bag_path = bag_path
        self.output_dir = Path(output_dir)
        self.sequence_name = sequence_name

        self.seq_dir = self.output_dir / 'figures' / sequence_name
        self.json_dir = self.seq_dir / 'json'
        self.csv_dir = self.seq_dir / 'csv'

        self.default_Q = [0.0, 0.0, 0.01]
        self.default_R = 0.1
        self.default_P0 = [0.0, 0.0, 0.1]

        self.wheel_radius = 0.033
        self.track_width = 0.160

        self.local_map_size = 15
        self.local_map_voxel_size = 0.05

        os.makedirs(self.seq_dir, exist_ok=True)
        os.makedirs(self.json_dir, exist_ok=True)
        os.makedirs(self.csv_dir, exist_ok=True)

    def transform_pointcloud_to_global(self, pcd, pose):
        x, y, theta = pose
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)

        points = np.asarray(pcd.points)
        transformed = np.zeros_like(points)
        transformed[:, 0] = points[:, 0] * cos_t - points[:, 1] * sin_t + x
        transformed[:, 1] = points[:, 0] * sin_t + points[:, 1] * cos_t + y
        if points.shape[1] > 2:
            transformed[:, 2] = points[:, 2]

        transformed_pcd = o3d.geometry.PointCloud()
        transformed_pcd.points = o3d.utility.Vector3dVector(transformed)
        if pcd.has_normals():
            normals = np.asarray(pcd.normals)
            transformed_normals = np.zeros_like(normals)
            transformed_normals[:, 0] = normals[:, 0] * cos_t - normals[:, 1] * sin_t
            transformed_normals[:, 1] = normals[:, 0] * sin_t + normals[:, 1] * cos_t
            if normals.shape[1] > 2:
                transformed_normals[:, 2] = normals[:, 2]
            transformed_pcd.normals = o3d.utility.Vector3dVector(transformed_normals)

        return transformed_pcd

    def build_local_map(self, keyframe_clouds):
        if len(keyframe_clouds) == 0:
            return None

        merged = o3d.geometry.PointCloud()
        for pcd in keyframe_clouds:
            merged += pcd

        if self.local_map_voxel_size > 0 and len(merged.points) > 0:
            merged = merged.voxel_down_sample(self.local_map_voxel_size)

        return merged

    def run_ekf_odometry(self, data):
        wheel_odom = WheelOdometry(self.wheel_radius, self.track_width)
        ekf = EKF(Q=self.default_Q, P0=self.default_P0)
        R_matrix = np.array([[self.default_R]])

        imu_offset = None
        results = []
        prev_time = None

        for d in data:
            t = d['timestamp']

            if prev_time is None:
                prev_time = t
                wheel_odom.update(d['rad_l'], d['rad_r'], 0.05)
                continue

            dt = t - prev_time
            if dt <= 0:
                continue

            wheel_odom.update(d['rad_l'], d['rad_r'], dt)
            v, omega = wheel_odom.get_velocity()

            ekf.predict(v, omega, dt)

            if imu_offset is None:
                imu_offset = d['imu_yaw']

            imu_corrected = d['imu_yaw'] - imu_offset
            imu_corrected = math.atan2(math.sin(imu_corrected), math.cos(imu_corrected))

            z = np.array([imu_corrected])
            k_theta, innovation, S = ekf.correct(z, R_matrix)

            wheel_pose = wheel_odom.get_pose()
            ekf_state = ekf.get_state()

            results.append({
                'timestamp': t,
                'wheel_pose': wheel_pose,
                'ekf_pose': ekf_state,
                'imu_yaw': imu_corrected,
                'scan_ranges': d['scan_ranges'],
                'scan_angles': d['scan_angles']
            })

            prev_time = t

        return results

    def run_icp_method(self, ekf_results, icp_method, processor):
        trajectory = []
        detailed_results = []
        scan_data = []
        local_map_keyframes = deque(maxlen=self.local_map_size)
        prev_ekf_pose = None
        current_icp_pose = [0.0, 0.0, 0.0]
        last_kf_pose = [0.0, 0.0, 0.0]
        accumulated_dx = 0.0
        accumulated_dy = 0.0
        accumulated_dtheta = 0.0

        for i, res in enumerate(ekf_results):
            current_ekf_pose = res['ekf_pose']

            if prev_ekf_pose is None:
                prev_ekf_pose = current_ekf_pose
                trajectory.append([res['timestamp'], 0.0, 0.0, 0.0])
                scan_data.append({'ranges': res['scan_ranges'], 'angles': res['scan_angles']})
                current_pcd = processor.preprocess(res['scan_ranges'], res['scan_angles'], compute_normals=False)
                if len(current_pcd.points) >= 10:
                    global_pcd = self.transform_pointcloud_to_global(current_pcd, current_icp_pose)
                    local_map_keyframes.append(global_pcd)
                    last_kf_pose = current_icp_pose.copy()
                continue

            dx_ekf = current_ekf_pose[0] - prev_ekf_pose[0]
            dy_ekf = current_ekf_pose[1] - prev_ekf_pose[1]
            dtheta_ekf = current_ekf_pose[2] - prev_ekf_pose[2]
            dtheta_ekf = math.atan2(math.sin(dtheta_ekf), math.cos(dtheta_ekf))

            accumulated_dx += dx_ekf
            accumulated_dy += dy_ekf
            accumulated_dtheta += dtheta_ekf
            accumulated_dtheta = math.atan2(math.sin(accumulated_dtheta), math.cos(accumulated_dtheta))

            predicted_x = last_kf_pose[0] + accumulated_dx
            predicted_y = last_kf_pose[1] + accumulated_dy
            predicted_theta = last_kf_pose[2] + accumulated_dtheta
            predicted_theta = math.atan2(math.sin(predicted_theta), math.cos(predicted_theta))

            dist_from_kf = math.sqrt(accumulated_dx**2 + accumulated_dy**2)
            angle_from_kf = abs(accumulated_dtheta)

            if dist_from_kf > 0.3 or angle_from_kf > math.radians(10.0):
                current_pcd = processor.preprocess(res['scan_ranges'], res['scan_angles'], compute_normals=False)

                if len(current_pcd.points) >= 10 and len(local_map_keyframes) >= 3:
                    local_map = self.build_local_map(list(local_map_keyframes))

                    if local_map is not None and len(local_map.points) >= 50:
                        scan_points_body = np.asarray(current_pcd.points)
                        map_points_odom = np.asarray(local_map.points)

                        try:
                            icp_result = icp_method.register_scan_to_map(
                                scan_points_body, map_points_odom,
                                predicted_x, predicted_y, predicted_theta
                            )

                            if icp_result.get('success', False):
                                corr_t = math.sqrt((icp_result['x'] - predicted_x)**2 + (icp_result['y'] - predicted_y)**2)
                                corr_r = abs(math.atan2(math.sin(icp_result['theta'] - predicted_theta), math.cos(icp_result['theta'] - predicted_theta)))

                                if corr_t < 0.3 and corr_r < math.radians(5.0):
                                    current_icp_pose = [icp_result['x'], icp_result['y'], icp_result['theta']]
                                else:
                                    current_icp_pose = [predicted_x, predicted_y, predicted_theta]
                            else:
                                current_icp_pose = [predicted_x, predicted_y, predicted_theta]

                            detailed_results.append({
                                'timestamp': res['timestamp'],
                                'x': current_icp_pose[0],
                                'y': current_icp_pose[1],
                                'theta': current_icp_pose[2],
                                'fitness': icp_result.get('fitness', 0.0),
                                'inlier_rmse': icp_result.get('inlier_rmse', 1.0),
                                'iterations': icp_result.get('iterations', 0),
                                'runtime_ms': icp_result.get('runtime_ms', 0.0)
                            })
                        except Exception as e:
                            print(f"ICP failed: {e}")
                            current_icp_pose = [predicted_x, predicted_y, predicted_theta]
                else:
                    current_icp_pose = [predicted_x, predicted_y, predicted_theta]

                if len(current_pcd.points) >= 10:
                    global_pcd = self.transform_pointcloud_to_global(current_pcd, current_icp_pose)
                    local_map_keyframes.append(global_pcd)
                    last_kf_pose = current_icp_pose.copy()

                trajectory.append([res['timestamp'], current_icp_pose[0], current_icp_pose[1], current_icp_pose[2]])
                scan_data.append({'ranges': res['scan_ranges'], 'angles': res['scan_angles']})

                accumulated_dx = 0.0
                accumulated_dy = 0.0
                accumulated_dtheta = 0.0

            prev_ekf_pose = current_ekf_pose

        processor.reset_keyframe()

        return trajectory, detailed_results, scan_data

    def run_all_methods(self):
        print(f"Running all ICP methods for {self.sequence_name}...")

        reader = BagReader(self.bag_path)
        data = reader.get_synchronized_data()

        if not data:
            print(f"No data found in {self.bag_path}")
            return None

        print(f"  Total data points: {len(data)}")

        ekf_results = self.run_ekf_odometry(data)
        print(f"  EKF odometry computed: {len(ekf_results)} keyframes")

        processor = LidarProcessor(voxel_size=0.0, translation_threshold=0.15, rotation_threshold=0.087)

        icp_methods = [
            PointToPointICP()
        ]

        all_trajectories = {}
        all_trajectories_with_time = {}
        all_detailed_results = {}
        all_scan_data = {}
        method_summaries = {}

        wheel_traj = [[r['wheel_pose'][0], r['wheel_pose'][1], r['wheel_pose'][2]] for r in ekf_results]
        ekf_traj = [[r['ekf_pose'][0], r['ekf_pose'][1], r['ekf_pose'][2]] for r in ekf_results]
        all_trajectories['Wheel'] = wheel_traj
        all_trajectories['EKF'] = ekf_traj

        final_imu_yaw = ekf_results[-1]['imu_yaw']

        wheel_heading_dev = abs(math.atan2(math.sin(wheel_traj[-1][2] - final_imu_yaw),
                                          math.cos(wheel_traj[-1][2] - final_imu_yaw)))
        ekf_heading_dev = abs(math.atan2(math.sin(ekf_traj[-1][2] - final_imu_yaw),
                                        math.cos(ekf_traj[-1][2] - final_imu_yaw)))

        method_summaries['Wheel'] = {
            'final_pose': {
                'x': wheel_traj[-1][0],
                'y': wheel_traj[-1][1],
                'theta_deg': math.degrees(wheel_traj[-1][2])
            },
            'heading_deviation_from_imu_deg': math.degrees(wheel_heading_dev),
            'trajectory_length_m': compute_trajectory_drift(wheel_traj)
        }

        method_summaries['EKF'] = {
            'final_pose': {
                'x': ekf_traj[-1][0],
                'y': ekf_traj[-1][1],
                'theta_deg': math.degrees(ekf_traj[-1][2])
            },
            'heading_deviation_from_imu_deg': math.degrees(ekf_heading_dev),
            'trajectory_length_m': compute_trajectory_drift(ekf_traj)
        }

        trajectory_poses = {}

        for icp_method in icp_methods:
            print(f"  Processing {icp_method.name}...")

            trajectory, detailed_results, scan_data = self.run_icp_method(ekf_results, icp_method, processor)

            if len(trajectory) == 0:
                print(f"    Warning: No trajectory generated for {icp_method.name}")
                continue

            traj_poses = [[t[1], t[2], t[3]] for t in trajectory]
            all_trajectories[icp_method.name] = traj_poses
            all_trajectories_with_time[icp_method.name] = trajectory
            trajectory_poses[icp_method.name] = traj_poses
            all_detailed_results[icp_method.name] = detailed_results
            all_scan_data[icp_method.name] = scan_data

            runtimes = [r['runtime_ms'] for r in detailed_results]
            fitness_scores = [r['fitness'] for r in detailed_results]
            rmse_scores = [r['inlier_rmse'] for r in detailed_results]
            iterations = [r['iterations'] for r in detailed_results]

            icp_heading_dev = abs(math.atan2(math.sin(traj_poses[-1][2] - final_imu_yaw),
                                            math.cos(traj_poses[-1][2] - final_imu_yaw)))

            method_summaries[icp_method.name] = {
                'num_keyframes': len(traj_poses),
                'final_pose': {
                    'x': traj_poses[-1][0],
                    'y': traj_poses[-1][1],
                    'theta_deg': math.degrees(traj_poses[-1][2])
                },
                'heading_deviation_from_imu_deg': math.degrees(icp_heading_dev),
                'avg_runtime_ms': np.mean(runtimes),
                'std_runtime_ms': np.std(runtimes),
                'total_runtime_s': np.sum(runtimes) / 1000.0,
                'avg_fitness': np.mean(fitness_scores),
                'std_fitness': np.std(fitness_scores),
                'avg_rmse': np.mean(rmse_scores),
                'std_rmse': np.std(rmse_scores),
                'avg_iterations': compute_convergence_rate(detailed_results),
                'consistency_score': compute_consistency_score(detailed_results),
                'trajectory_length_m': compute_trajectory_drift(traj_poses)
            }

            print(f"    {icp_method.name}: {len(trajectory)} total poses, "
                  f"avg_fitness={np.mean(fitness_scores):.3f}, "
                  f"avg_runtime={np.mean(runtimes):.1f}ms, "
                  f"final_theta={math.degrees(traj_poses[-1][2]):.2f}deg")

        best_method_name = icp_methods[0].name
        print(f"\n  Using method: {best_method_name}")

        print(f"  Generating trajectory comparison plot...")
        fig = plot_all_trajectories(all_trajectories, f'{self.sequence_name}: Trajectory Comparison')
        save_figure(fig, self.seq_dir / 'all_trajectories.png')

        print(f"  Saving detailed results to CSV...")
        for method_name, details in all_detailed_results.items():
            save_csv(details, self.csv_dir / f'{method_name.lower().replace(" ", "_")}_data.csv')

        print(f"  Generating map with scans...")
        for method_name in all_scan_data.keys():
            fig = plot_map_with_scans(
                all_trajectories[method_name],
                all_scan_data[method_name],
                f'{self.sequence_name}: {method_name} Map',
                downsample=5
            )
            save_figure(fig, self.seq_dir / f'map_{method_name.lower().replace(" ", "_").replace("-", "_")}.png')

        print(f"  Generating occupancy grid maps (matching turtle_icp_mapper)...")
        for method_name in all_scan_data.keys():
            occupancy_grid, grid_info = create_occupancy_grid(
                all_trajectories[method_name],
                all_scan_data[method_name],
                resolution=0.05,
                min_range=0.12,
                max_range=3.5,
                occupied_threshold=0.6,
                free_threshold=0.4
            )

            if occupancy_grid.size > 0:
                fig = plot_occupancy_grid(
                    occupancy_grid,
                    grid_info,
                    trajectory=all_trajectories[method_name],
                    title=f'{self.sequence_name}: {method_name} Occupancy Grid Map',
                    show_trajectory=True
                )
                save_figure(fig, self.seq_dir / f'occupancy_grid_{method_name.lower().replace(" ", "_").replace("-", "_")}.png')
                print(f"    {method_name}: Grid size {grid_info['width']}x{grid_info['height']} "
                      f"@ {grid_info['resolution']}m resolution")

        print(f"  Generating combined time series plot...")
        import matplotlib.pyplot as plt

        wheel_traj_ts = np.array([[r['timestamp'], r['wheel_pose'][0], r['wheel_pose'][1], r['wheel_pose'][2]] for r in ekf_results])
        ekf_traj_ts = np.array([[r['timestamp'], r['ekf_pose'][0], r['ekf_pose'][1], r['ekf_pose'][2]] for r in ekf_results])

        fig, axes = plt.subplots(3, 1, figsize=(14, 12))
        fig.suptitle(f'{self.sequence_name}: Time Series Comparison', fontsize=16, fontweight='bold')

        colors = {
            'Wheel': '#1f77b4',
            'EKF': '#d62728',
            'Point-to-Point': '#2ca02c'
        }

        t0 = ekf_traj_ts[0, 0]

        axes[0].plot(wheel_traj_ts[:, 0] - t0, wheel_traj_ts[:, 1], '-', color=colors['Wheel'], label='Wheel Odometry', linewidth=1.5, alpha=0.8)
        axes[0].plot(ekf_traj_ts[:, 0] - t0, ekf_traj_ts[:, 1], '-', color=colors['EKF'], label='EKF', linewidth=1.5, alpha=0.9)

        for method_name in all_trajectories_with_time.keys():
            icp_traj = np.array(all_trajectories_with_time[method_name])
            axes[0].plot(icp_traj[:, 0] - t0, icp_traj[:, 1], '-', color=colors[method_name], label=method_name, linewidth=1.5, alpha=0.8)

        axes[0].set_ylabel('X (m)', fontsize=13, fontweight='bold')
        axes[0].grid(True, alpha=0.3)
        axes[0].legend(loc='best', fontsize=10)

        axes[1].plot(wheel_traj_ts[:, 0] - t0, wheel_traj_ts[:, 2], '-', color=colors['Wheel'], label='Wheel Odometry', linewidth=1.5, alpha=0.8)
        axes[1].plot(ekf_traj_ts[:, 0] - t0, ekf_traj_ts[:, 2], '-', color=colors['EKF'], label='EKF', linewidth=1.5, alpha=0.9)

        for method_name in all_trajectories_with_time.keys():
            icp_traj = np.array(all_trajectories_with_time[method_name])
            axes[1].plot(icp_traj[:, 0] - t0, icp_traj[:, 2], '-', color=colors[method_name], label=method_name, linewidth=1.5, alpha=0.8)

        axes[1].set_ylabel('Y (m)', fontsize=13, fontweight='bold')
        axes[1].grid(True, alpha=0.3)
        axes[1].legend(loc='best', fontsize=10)

        axes[2].plot(wheel_traj_ts[:, 0] - t0, np.degrees(wheel_traj_ts[:, 3]), '-', color=colors['Wheel'], label='Wheel Odometry', linewidth=1.5, alpha=0.8)
        axes[2].plot(ekf_traj_ts[:, 0] - t0, np.degrees(ekf_traj_ts[:, 3]), '-', color=colors['EKF'], label='EKF', linewidth=1.5, alpha=0.9)

        for method_name in all_trajectories_with_time.keys():
            icp_traj = np.array(all_trajectories_with_time[method_name])
            axes[2].plot(icp_traj[:, 0] - t0, np.degrees(icp_traj[:, 3]), '-', color=colors[method_name], label=method_name, linewidth=1.5, alpha=0.8)

        axes[2].set_ylabel('Theta (deg)', fontsize=13, fontweight='bold')
        axes[2].set_xlabel('Time (s)', fontsize=13, fontweight='bold')
        axes[2].grid(True, alpha=0.3)
        axes[2].legend(loc='best', fontsize=10)

        plt.tight_layout()
        save_figure(fig, self.seq_dir / 'time_series.png')

        print(f"  Generating ICP performance analysis...")
        for method_name in all_detailed_results.keys():
            fig, axes = plt.subplots(3, 1, figsize=(14, 10))
            fig.suptitle(f'{self.sequence_name}: ICP Performance', fontsize=16, fontweight='bold')

            details = all_detailed_results[method_name]
            timestamps = np.array([d['timestamp'] for d in details])
            fitness = np.array([d['fitness'] for d in details])
            rmse = np.array([d['inlier_rmse'] for d in details])
            runtime = np.array([d['runtime_ms'] for d in details])

            t0 = timestamps[0]

            axes[0].plot(timestamps - t0, fitness, 'o-', color=colors[method_name], linewidth=2, markersize=4)
            axes[0].set_ylabel('Fitness Score', fontsize=13, fontweight='bold')
            axes[0].set_title('ICP Fitness at Keyframes', fontsize=12, fontweight='bold')
            axes[0].grid(True, alpha=0.3)

            fitness_min = max(0.0, np.min(fitness) - 0.02)
            fitness_max = min(1.0, np.max(fitness) + 0.02)
            axes[0].set_ylim([fitness_min, fitness_max])

            axes[1].plot(timestamps - t0, rmse, 'o-', color=colors[method_name], linewidth=2, markersize=4)
            axes[1].set_ylabel('RMSE [m]', fontsize=13, fontweight='bold')
            axes[1].set_title('ICP RMSE at Keyframes', fontsize=12, fontweight='bold')
            axes[1].grid(True, alpha=0.3)

            axes[2].plot(timestamps - t0, runtime, 'o-', color=colors[method_name], linewidth=2, markersize=4)
            axes[2].set_ylabel('Runtime [ms]', fontsize=13, fontweight='bold')
            axes[2].set_xlabel('Time [s]', fontsize=13, fontweight='bold')
            axes[2].set_title('ICP Runtime at Keyframes', fontsize=12, fontweight='bold')
            axes[2].grid(True, alpha=0.3)

            plt.tight_layout()
            save_figure(fig, self.seq_dir / 'icp_performance.png')

        summary = {
            'sequence': self.sequence_name,
            'methods': method_summaries,
            'best_method': best_method_name
        }

        save_json(summary, self.json_dir / 'summary.json')

        return summary, best_method_name, trajectory_poses[best_method_name], all_scan_data[best_method_name], ekf_results, processor, all_detailed_results[best_method_name]

def main():
    base_path = Path(__file__).parent.parent
    dataset_path = base_path / 'FRA532_LAB1_DATASET'
    output_path = Path(__file__).parent

    sequences = [
        ('fibo_floor3_seq00', 'seq00'),
        ('fibo_floor3_seq01', 'seq01'),
        ('fibo_floor3_seq02', 'seq02')
    ]

    all_results = {}

    for bag_name, seq_name in sequences:
        bag_path = str(dataset_path / bag_name)

        if not os.path.exists(bag_path):
            print(f"Bag not found: {bag_path}")
            continue

        print(f"\n{'='*60}")
        print(f"Processing: {seq_name}")
        print(f"{'='*60}")

        exp = Experiment(bag_path, output_path, seq_name)

        result = exp.run_all_methods()
        if result:
            summary, best_method, best_traj, best_scan_data, ekf_results, processor, detailed_results = result
            all_results[seq_name] = summary

    if all_results:
        output_json = output_path / 'figures' / 'all_sequences_results.json'
        save_json(all_results, output_json)

    print("\n" + "="*60)
    print("All experiments completed!")
    print("="*60)


if __name__ == '__main__':
    main()
