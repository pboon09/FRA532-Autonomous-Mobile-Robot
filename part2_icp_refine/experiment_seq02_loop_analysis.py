#!/usr/bin/env python3

import os
import sys
import math
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt

part1_path = Path(__file__).parent.parent / 'part1_ekf_odom'
sys.path.insert(0, str(part1_path))
from ekf import EKF
from wheel_odometry import WheelOdometry
sys.path.pop(0)

from bag_reader import BagReader
from lidar_processor import LidarProcessor
from icp_point_to_point import PointToPointICP
from icp_point_to_plane import PointToPlaneICP
from icp_point_to_line import PointToLineICP
from icp_gicp import GICP
from loop_closure import LoopClosure
from utils import save_figure, save_json, save_csv, get_method_color


def run_ekf_odometry(data):
    wheel_radius = 0.033
    track_width = 0.160
    Q = [0.0, 0.0, 0.01]
    R = 0.1
    P0 = [0.0, 0.0, 0.1]

    wheel_odom = WheelOdometry(wheel_radius, track_width)
    ekf = EKF(Q=Q, P0=P0)
    R_matrix = np.array([[R]])

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
            'scan_ranges': d['scan_ranges'],
            'scan_angles': d['scan_angles']
        })

        prev_time = t

    return results


def run_icp_method(ekf_results, icp_method, processor):
    trajectory = []
    detailed_results = []
    scan_data = []
    point_clouds = []

    prev_pcd = None
    prev_ekf_pose = None
    current_icp_pose = [0.0, 0.0, 0.0]

    for i, res in enumerate(ekf_results):
        current_ekf_pose = res['ekf_pose']

        if not processor.is_keyframe(current_ekf_pose):
            continue

        current_pcd = processor.preprocess(
            res['scan_ranges'],
            res['scan_angles'],
            compute_normals=(icp_method.name in ["Point-to-Plane", "GICP"])
        )

        if len(current_pcd.points) < 10:
            continue

        if prev_pcd is None:
            prev_pcd = current_pcd
            prev_ekf_pose = current_ekf_pose
            trajectory.append([0.0, 0.0, 0.0])
            current_icp_pose = [0.0, 0.0, 0.0]
            scan_data.append({'ranges': res['scan_ranges'], 'angles': res['scan_angles']})
            point_clouds.append(current_pcd)
            continue

        dx_ekf = current_ekf_pose[0] - prev_ekf_pose[0]
        dy_ekf = current_ekf_pose[1] - prev_ekf_pose[1]
        dtheta_ekf = current_ekf_pose[2] - prev_ekf_pose[2]
        dtheta_ekf = math.atan2(math.sin(dtheta_ekf), math.cos(dtheta_ekf))

        cos_prev = math.cos(-prev_ekf_pose[2])
        sin_prev = math.sin(-prev_ekf_pose[2])
        dx_rel = dx_ekf * cos_prev - dy_ekf * sin_prev
        dy_rel = dx_ekf * sin_prev + dy_ekf * cos_prev

        init_transform = processor.pose_to_transform(dx_rel, dy_rel, dtheta_ekf)
        icp_result = icp_method.register(current_pcd, prev_pcd, init_transform)

        dx_icp, dy_icp, dtheta_icp = processor.transform_to_pose(icp_result['transformation'])

        cos_curr = math.cos(current_icp_pose[2])
        sin_curr = math.sin(current_icp_pose[2])
        current_icp_pose[0] += dx_icp * cos_curr - dy_icp * sin_curr
        current_icp_pose[1] += dx_icp * sin_curr + dy_icp * cos_curr
        current_icp_pose[2] += dtheta_icp
        current_icp_pose[2] = math.atan2(math.sin(current_icp_pose[2]), math.cos(current_icp_pose[2]))

        trajectory.append([current_icp_pose[0], current_icp_pose[1], current_icp_pose[2]])
        scan_data.append({'ranges': res['scan_ranges'], 'angles': res['scan_angles']})
        point_clouds.append(current_pcd)

        detailed_results.append({
            'timestamp': res['timestamp'],
            'x': current_icp_pose[0],
            'y': current_icp_pose[1],
            'theta': current_icp_pose[2],
            'fitness': icp_result['fitness'],
            'inlier_rmse': icp_result['inlier_rmse'],
            'iterations': icp_result['iterations'],
            'runtime_ms': icp_result['runtime_ms']
        })

        prev_pcd = current_pcd
        prev_ekf_pose = current_ekf_pose

    processor.reset_keyframe()
    return trajectory, detailed_results, scan_data, point_clouds


def detect_and_optimize_loops(trajectory, ekf_results, processor, icp_method, scan_data):
    loop_closure = LoopClosure(search_radius=2.5, temporal_threshold=30.0,
                               fitness_threshold=0.65, rmse_threshold=0.12)

    keyframe_idx = 0
    for i, res in enumerate(ekf_results):
        current_pose = res['ekf_pose']

        if not processor.is_keyframe(current_pose):
            continue

        if keyframe_idx >= len(trajectory):
            break

        current_pcd = processor.preprocess(
            res['scan_ranges'],
            res['scan_angles'],
            compute_normals=(icp_method.name in ["Point-to-Plane", "Point-to-Line", "GICP"])
        )

        if len(current_pcd.points) < 10:
            continue

        loop_closure.add_keyframe(res['timestamp'], trajectory[keyframe_idx], current_pcd)

        if len(loop_closure.keyframes) >= 10:
            new_loops = loop_closure.detect_loops(icp_method)

        keyframe_idx += 1

    loop_info = loop_closure.get_loop_info()

    if loop_info['num_loops'] > 0:
        optimized_trajectory = loop_closure.optimize_pose_graph(trajectory)
    else:
        optimized_trajectory = trajectory

    processor.reset_keyframe()
    return optimized_trajectory, loop_info


def evaluate_trajectory(optimized_trajectory, point_clouds, icp_method):
    detailed_results = []

    for i in range(1, len(optimized_trajectory)):
        prev_pose = optimized_trajectory[i-1]
        curr_pose = optimized_trajectory[i]

        dx = curr_pose[0] - prev_pose[0]
        dy = curr_pose[1] - prev_pose[1]
        dtheta = curr_pose[2] - prev_pose[2]
        dtheta = math.atan2(math.sin(dtheta), math.cos(dtheta))

        cos_prev = math.cos(prev_pose[2])
        sin_prev = math.sin(prev_pose[2])
        dx_rel = dx * cos_prev + dy * sin_prev
        dy_rel = -dx * sin_prev + dy * cos_prev

        init_transform = np.eye(3)
        init_transform[0, 2] = dx_rel
        init_transform[1, 2] = dy_rel
        init_transform[0:2, 0:2] = np.array([
            [np.cos(dtheta), -np.sin(dtheta)],
            [np.sin(dtheta), np.cos(dtheta)]
        ])

        icp_result = icp_method.register(point_clouds[i], point_clouds[i-1], init_transform)

        detailed_results.append({
            'fitness': icp_result['fitness'],
            'inlier_rmse': icp_result['inlier_rmse'],
            'iterations': icp_result['iterations'],
            'runtime_ms': icp_result['runtime_ms']
        })

    return detailed_results


def plot_comparison(before_data, after_data, title):
    methods = list(before_data.keys())
    x = np.arange(len(methods))
    width = 0.35

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle(title, fontsize=16, fontweight='bold')

    fitness_before = [before_data[m]['avg_fitness'] for m in methods]
    fitness_after = [after_data[m]['avg_fitness'] for m in methods]
    ax = axes[0, 0]
    bars1 = ax.bar(x - width/2, fitness_before, width, label='Before Loop Closure', alpha=0.8, color='#3498db')
    bars2 = ax.bar(x + width/2, fitness_after, width, label='After Loop Closure', alpha=0.8, color='#2ecc71')
    ax.set_ylabel('Average Fitness', fontweight='bold')
    ax.set_title('Fitness Score Comparison', fontweight='bold')
    ax.set_xticks(x)
    ax.set_xticklabels(methods, rotation=15, ha='right')
    ax.legend()
    ax.grid(True, alpha=0.3)

    rmse_before = [before_data[m]['avg_rmse'] for m in methods]
    rmse_after = [after_data[m]['avg_rmse'] for m in methods]
    ax = axes[0, 1]
    bars1 = ax.bar(x - width/2, rmse_before, width, label='Before Loop Closure', alpha=0.8, color='#3498db')
    bars2 = ax.bar(x + width/2, rmse_after, width, label='After Loop Closure', alpha=0.8, color='#2ecc71')
    ax.set_ylabel('Average RMSE [m]', fontweight='bold')
    ax.set_title('RMSE Comparison', fontweight='bold')
    ax.set_xticks(x)
    ax.set_xticklabels(methods, rotation=15, ha='right')
    ax.legend()
    ax.grid(True, alpha=0.3)

    runtime_before = [before_data[m]['avg_runtime_ms'] for m in methods]
    runtime_after = [after_data[m]['avg_runtime_ms'] for m in methods]
    ax = axes[1, 0]
    bars1 = ax.bar(x - width/2, runtime_before, width, label='Before Loop Closure', alpha=0.8, color='#3498db')
    bars2 = ax.bar(x + width/2, runtime_after, width, label='After Loop Closure', alpha=0.8, color='#2ecc71')
    ax.set_ylabel('Average Runtime [ms]', fontweight='bold')
    ax.set_title('Runtime Comparison', fontweight='bold')
    ax.set_xticks(x)
    ax.set_xticklabels(methods, rotation=15, ha='right')
    ax.legend()
    ax.grid(True, alpha=0.3)

    iterations_before = [before_data[m]['avg_iterations'] for m in methods]
    iterations_after = [after_data[m]['avg_iterations'] for m in methods]
    ax = axes[1, 1]
    bars1 = ax.bar(x - width/2, iterations_before, width, label='Before Loop Closure', alpha=0.8, color='#3498db')
    bars2 = ax.bar(x + width/2, iterations_after, width, label='After Loop Closure', alpha=0.8, color='#2ecc71')
    ax.set_ylabel('Average Iterations', fontweight='bold')
    ax.set_title('Convergence Comparison', fontweight='bold')
    ax.set_xticks(x)
    ax.set_xticklabels(methods, rotation=15, ha='right')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    return fig


def plot_trajectory_comparison(trajectories_before, trajectories_after, loop_info, title):
    fig, axes = plt.subplots(1, 2, figsize=(18, 8))
    fig.suptitle(title, fontsize=16, fontweight='bold')

    ax = axes[0]
    for method_name, traj in trajectories_before.items():
        x = [p[0] for p in traj]
        y = [p[1] for p in traj]
        color = get_method_color(method_name)
        ax.plot(x, y, color=color, label=method_name, linewidth=2, alpha=0.85)
    ax.plot(x[0], y[0], 'go', markersize=12, label='Start', zorder=10, markeredgecolor='black', markeredgewidth=2)
    ax.set_xlabel('X [m]', fontsize=13, fontweight='bold')
    ax.set_ylabel('Y [m]', fontsize=13, fontweight='bold')
    ax.set_title('Before Loop Closure', fontsize=14, fontweight='bold')
    ax.legend(loc='best', fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')

    ax = axes[1]
    for method_name, traj in trajectories_after.items():
        x = [p[0] for p in traj]
        y = [p[1] for p in traj]
        color = get_method_color(method_name)
        ax.plot(x, y, color=color, label=method_name, linewidth=2, alpha=0.85)

    if loop_info['num_loops'] > 0:
        for loop in loop_info['loops']:
            i, j = loop['from_idx'], loop['to_idx']
            if i < len(list(trajectories_after.values())[0]) and j < len(list(trajectories_after.values())[0]):
                first_traj = list(trajectories_after.values())[0]
                ax.plot([first_traj[i][0], first_traj[j][0]],
                       [first_traj[i][1], first_traj[j][1]],
                       'r--', linewidth=1.5, alpha=0.6)

    ax.plot(x[0], y[0], 'go', markersize=12, label='Start', zorder=10, markeredgecolor='black', markeredgewidth=2)
    ax.set_xlabel('X [m]', fontsize=13, fontweight='bold')
    ax.set_ylabel('Y [m]', fontsize=13, fontweight='bold')
    ax.set_title(f'After Loop Closure ({loop_info["num_loops"]} loops)', fontsize=14, fontweight='bold')
    ax.legend(loc='best', fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')

    plt.tight_layout()
    return fig


def main():
    base_path = Path(__file__).parent.parent
    dataset_path = base_path / 'FRA532_LAB1_DATASET'
    output_path = Path(__file__).parent

    bag_path = str(dataset_path / 'fibo_floor3_seq02')
    sequence_name = 'seq02'

    if not os.path.exists(bag_path):
        print(f"Bag not found: {bag_path}")
        return

    print(f"{'='*60}")
    print(f"Processing: {sequence_name} - Loop Closure Analysis")
    print(f"{'='*60}")

    seq_dir = output_path / 'figures' / sequence_name
    json_dir = seq_dir / 'json'
    csv_dir = seq_dir / 'csv'
    os.makedirs(seq_dir, exist_ok=True)
    os.makedirs(json_dir, exist_ok=True)
    os.makedirs(csv_dir, exist_ok=True)

    reader = BagReader(bag_path)
    data = reader.get_synchronized_data()

    if not data:
        print(f"No data found in {bag_path}")
        return

    print(f"Total data points: {len(data)}")

    ekf_results = run_ekf_odometry(data)
    print(f"EKF odometry computed: {len(ekf_results)} keyframes")

    processor = LidarProcessor(voxel_size=0.1, translation_threshold=0.3, rotation_threshold=0.174)

    icp_methods = [
        PointToPointICP(),
        PointToPlaneICP(),
        PointToLineICP(),
        GICP()
    ]

    before_stats = {}
    after_stats = {}
    trajectories_before = {}
    trajectories_after = {}

    print("\n" + "="*60)
    print("PHASE 1: Before Loop Closure")
    print("="*60)

    all_point_clouds = {}
    all_scan_data = {}

    for icp_method in icp_methods:
        print(f"\nProcessing {icp_method.name}...")

        trajectory, detailed_results, scan_data, point_clouds = run_icp_method(ekf_results, icp_method, processor)

        if len(trajectory) == 0:
            print(f"Warning: No trajectory generated for {icp_method.name}")
            continue

        trajectories_before[icp_method.name] = trajectory
        all_point_clouds[icp_method.name] = point_clouds
        all_scan_data[icp_method.name] = scan_data

        fitness_scores = [r['fitness'] for r in detailed_results]
        rmse_scores = [r['inlier_rmse'] for r in detailed_results]
        runtimes = [r['runtime_ms'] for r in detailed_results]
        iterations = [r['iterations'] for r in detailed_results]

        before_stats[icp_method.name] = {
            'avg_fitness': np.mean(fitness_scores),
            'std_fitness': np.std(fitness_scores),
            'avg_rmse': np.mean(rmse_scores),
            'std_rmse': np.std(rmse_scores),
            'avg_runtime_ms': np.mean(runtimes),
            'std_runtime_ms': np.std(runtimes),
            'avg_iterations': np.mean(iterations),
            'num_keyframes': len(trajectory)
        }

        print(f"  Keyframes: {len(trajectory)}")
        print(f"  Avg Fitness: {np.mean(fitness_scores):.3f}")
        print(f"  Avg RMSE: {np.mean(rmse_scores):.4f} m")
        print(f"  Avg Runtime: {np.mean(runtimes):.2f} ms")

    print("\n" + "="*60)
    print("PHASE 2: Loop Closure Detection & Optimization")
    print("="*60)

    loop_infos = {}

    for icp_method in icp_methods:
        method_name = icp_method.name

        if method_name not in trajectories_before:
            continue

        print(f"\nRunning loop closure for {method_name}...")

        optimized_trajectory, loop_info = detect_and_optimize_loops(
            trajectories_before[method_name],
            ekf_results,
            processor,
            icp_method,
            all_scan_data[method_name]
        )

        trajectories_after[method_name] = optimized_trajectory
        loop_infos[method_name] = loop_info

        print(f"  Loops detected: {loop_info['num_loops']}")

    print("\n" + "="*60)
    print("PHASE 3: Re-evaluation After Loop Closure")
    print("="*60)

    for icp_method in icp_methods:
        method_name = icp_method.name

        if method_name not in trajectories_after:
            continue

        print(f"\nRe-evaluating {method_name}...")

        detailed_results_after = evaluate_trajectory(
            trajectories_after[method_name],
            all_point_clouds[method_name],
            icp_method
        )

        if len(detailed_results_after) == 0:
            after_stats[method_name] = before_stats[method_name]
            continue

        fitness_scores = [r['fitness'] for r in detailed_results_after]
        rmse_scores = [r['inlier_rmse'] for r in detailed_results_after]
        runtimes = [r['runtime_ms'] for r in detailed_results_after]
        iterations = [r['iterations'] for r in detailed_results_after]

        after_stats[method_name] = {
            'avg_fitness': np.mean(fitness_scores),
            'std_fitness': np.std(fitness_scores),
            'avg_rmse': np.mean(rmse_scores),
            'std_rmse': np.std(rmse_scores),
            'avg_runtime_ms': np.mean(runtimes),
            'std_runtime_ms': np.std(runtimes),
            'avg_iterations': np.mean(iterations),
            'num_keyframes': len(trajectories_after[method_name])
        }

        print(f"  Avg Fitness: {np.mean(fitness_scores):.3f}")
        print(f"  Avg RMSE: {np.mean(rmse_scores):.4f} m")
        print(f"  Avg Runtime: {np.mean(runtimes):.2f} ms")

    print("\n" + "="*60)
    print("PHASE 4: Generating Comparison Plots")
    print("="*60)

    fig = plot_comparison(before_stats, after_stats,
                         f'{sequence_name}: Performance Before vs After Loop Closure')
    save_figure(fig, seq_dir / 'loop_closure_performance_comparison.png')
    print("  Performance comparison plot saved")

    first_loop_info = list(loop_infos.values())[0]
    fig = plot_trajectory_comparison(trajectories_before, trajectories_after, first_loop_info,
                                    f'{sequence_name}: Trajectory Before vs After Loop Closure')
    save_figure(fig, seq_dir / 'loop_closure_trajectory_comparison.png')
    print("  Trajectory comparison plot saved")

    summary = {
        'sequence': sequence_name,
        'before_loop_closure': before_stats,
        'after_loop_closure': after_stats,
        'loop_info': {method: info for method, info in loop_infos.items()}
    }

    save_json(summary, json_dir / 'loop_closure_analysis_summary.json')
    print("  Summary JSON saved")

    print("\n" + "="*60)
    print("Loop Closure Analysis Completed!")
    print("="*60)

    print("\nSummary:")
    for method_name in before_stats.keys():
        print(f"\n{method_name}:")
        print(f"  Before - Fitness: {before_stats[method_name]['avg_fitness']:.3f}, RMSE: {before_stats[method_name]['avg_rmse']:.4f}m")
        print(f"  After  - Fitness: {after_stats[method_name]['avg_fitness']:.3f}, RMSE: {after_stats[method_name]['avg_rmse']:.4f}m")
        if method_name in loop_infos:
            print(f"  Loops: {loop_infos[method_name]['num_loops']}")


if __name__ == '__main__':
    main()
