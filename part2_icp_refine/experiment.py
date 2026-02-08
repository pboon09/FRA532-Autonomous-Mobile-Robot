#!/usr/bin/env python3

import os
import sys
import math
import numpy as np
from pathlib import Path

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
from utils import (
    save_figure, save_json, save_csv,
    plot_all_trajectories, plot_icp_performance,
    plot_loop_closure_comparison,
    plot_runtime_boxplot, compute_trajectory_drift,
    compute_final_position_error, rank_methods, plot_map_with_scans,
    compute_convergence_rate, compute_consistency_score,
    plot_loop_closure_analysis, get_method_color
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

        os.makedirs(self.seq_dir, exist_ok=True)
        os.makedirs(self.json_dir, exist_ok=True)
        os.makedirs(self.csv_dir, exist_ok=True)

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
                'scan_ranges': d['scan_ranges'],
                'scan_angles': d['scan_angles']
            })

            prev_time = t

        return results

    def run_icp_method(self, ekf_results, icp_method, processor):
        trajectory = []
        detailed_results = []
        scan_data = []

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

        return trajectory, detailed_results, scan_data

    def reevaluate_trajectory(self, optimized_trajectory, ekf_results, icp_method, processor):
        detailed_results = []
        point_clouds = []

        keyframe_idx = 0
        for i, res in enumerate(ekf_results):
            current_pose = res['ekf_pose']

            if not processor.is_keyframe(current_pose):
                continue

            if keyframe_idx >= len(optimized_trajectory):
                break

            current_pcd = processor.preprocess(
                res['scan_ranges'],
                res['scan_angles'],
                compute_normals=(icp_method.name in ["Point-to-Plane", "GICP"])
            )

            if len(current_pcd.points) < 10:
                continue

            point_clouds.append(current_pcd)
            keyframe_idx += 1

        for i in range(1, len(optimized_trajectory)):
            if i >= len(point_clouds):
                break

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

            init_transform = processor.pose_to_transform(dx_rel, dy_rel, dtheta)
            icp_result = icp_method.register(point_clouds[i], point_clouds[i-1], init_transform)

            detailed_results.append({
                'fitness': icp_result['fitness'],
                'inlier_rmse': icp_result['inlier_rmse'],
                'iterations': icp_result['iterations'],
                'runtime_ms': icp_result['runtime_ms']
            })

        processor.reset_keyframe()
        return detailed_results

    def plot_loop_closure_performance(self, before_stats, after_stats, num_loops, title):
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(1, 2, figsize=(14, 6))

        x = np.arange(2)
        width = 0.6

        ax = axes[0]
        fitness_vals = [before_stats['avg_fitness'], after_stats['avg_fitness']]
        colors = ['#e74c3c', '#2ecc71']
        bars = ax.bar(x, fitness_vals, width, color=colors, edgecolor='black', linewidth=1.5)
        ax.set_ylabel('Average Fitness', fontsize=13, fontweight='bold')
        ax.set_title(f'Fitness Comparison ({num_loops} loops detected)', fontsize=14, fontweight='bold')
        ax.set_xticks(x)
        ax.set_xticklabels(['Before Loop Closure', 'After Loop Closure'], fontsize=11)
        ax.grid(True, alpha=0.3, axis='y')
        ax.set_ylim(min(fitness_vals) * 0.95, 1.02)

        for i, (bar, val) in enumerate(zip(bars, fitness_vals)):
            ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.01,
                   f'{val:.4f}', ha='center', va='bottom', fontsize=11, fontweight='bold')

        ax = axes[1]
        rmse_vals = [before_stats['avg_rmse'], after_stats['avg_rmse']]
        bars = ax.bar(x, rmse_vals, width, color=colors, edgecolor='black', linewidth=1.5)
        ax.set_ylabel('Average RMSE [m]', fontsize=13, fontweight='bold')
        ax.set_title(f'RMSE Comparison ({num_loops} loops detected)', fontsize=14, fontweight='bold')
        ax.set_xticks(x)
        ax.set_xticklabels(['Before Loop Closure', 'After Loop Closure'], fontsize=11)
        ax.grid(True, alpha=0.3, axis='y')
        ax.set_ylim(0, max(rmse_vals) * 1.15)

        for i, (bar, val) in enumerate(zip(bars, rmse_vals)):
            ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + max(rmse_vals)*0.03,
                   f'{val:.4f}', ha='center', va='bottom', fontsize=11, fontweight='bold')

        fig.suptitle(title, fontsize=16, fontweight='bold')
        plt.tight_layout()

        return fig

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

        processor = LidarProcessor(voxel_size=0.1, translation_threshold=0.3, rotation_threshold=0.174)

        icp_methods = [
            PointToPointICP(),
            PointToPlaneICP(),
            PointToLineICP(),
            GICP()
        ]

        all_trajectories = {}
        all_detailed_results = {}
        all_scan_data = {}
        method_summaries = {}

        wheel_traj = [[r['wheel_pose'][0], r['wheel_pose'][1], r['wheel_pose'][2]] for r in ekf_results]
        ekf_traj = [[r['ekf_pose'][0], r['ekf_pose'][1], r['ekf_pose'][2]] for r in ekf_results]
        all_trajectories['Wheel'] = wheel_traj
        all_trajectories['EKF'] = ekf_traj

        for icp_method in icp_methods:
            print(f"  Processing {icp_method.name}...")

            trajectory, detailed_results, scan_data = self.run_icp_method(ekf_results, icp_method, processor)

            if len(trajectory) == 0:
                print(f"    Warning: No trajectory generated for {icp_method.name}")
                continue

            all_trajectories[icp_method.name] = trajectory
            all_detailed_results[icp_method.name] = detailed_results
            all_scan_data[icp_method.name] = scan_data

            runtimes = [r['runtime_ms'] for r in detailed_results]
            fitness_scores = [r['fitness'] for r in detailed_results]
            rmse_scores = [r['inlier_rmse'] for r in detailed_results]
            iterations = [r['iterations'] for r in detailed_results]

            method_summaries[icp_method.name] = {
                'num_keyframes': len(trajectory),
                'final_pose': {
                    'x': trajectory[-1][0],
                    'y': trajectory[-1][1],
                    'theta_deg': math.degrees(trajectory[-1][2])
                },
                'avg_runtime_ms': np.mean(runtimes),
                'std_runtime_ms': np.std(runtimes),
                'total_runtime_s': np.sum(runtimes) / 1000.0,
                'avg_fitness': np.mean(fitness_scores),
                'std_fitness': np.std(fitness_scores),
                'avg_rmse': np.mean(rmse_scores),
                'std_rmse': np.std(rmse_scores),
                'avg_iterations': compute_convergence_rate(detailed_results),
                'consistency_score': compute_consistency_score(detailed_results),
                'trajectory_length_m': compute_trajectory_drift(trajectory)
            }

            print(f"    {icp_method.name}: {len(trajectory)} poses, "
                  f"avg_fitness={np.mean(fitness_scores):.3f}, "
                  f"avg_runtime={np.mean(runtimes):.1f}ms")

        ranked_methods, rankings = rank_methods(method_summaries)
        best_method_name = ranked_methods[0][0]

        print(f"\n  Ranking (best to worst):")
        for i, (method, scores) in enumerate(ranked_methods):
            print(f"    {i+1}. {method}: score={scores['total_score']:.3f}")

        print(f"\n  Best method: {best_method_name}")

        fig = plot_all_trajectories(all_trajectories, f'{self.sequence_name}: All Methods Comparison')
        save_figure(fig, self.seq_dir / 'all_trajectories.png')

        fig = plot_icp_performance(method_summaries, all_detailed_results,
                                   f'{self.sequence_name}: ICP Performance')
        save_figure(fig, self.seq_dir / 'icp_performance.png')

        runtime_data = {m: [r['runtime_ms'] for r in all_detailed_results[m]] for m in all_detailed_results.keys()}
        fig = plot_runtime_boxplot(runtime_data, f'{self.sequence_name}: Runtime Distribution')
        save_figure(fig, self.seq_dir / 'runtime_boxplot.png')

        for method_name, details in all_detailed_results.items():
            save_csv(details, self.csv_dir / f'{method_name.lower().replace(" ", "_")}_data.csv')

        for method_name in all_scan_data.keys():
            print(f"  Generating map for {method_name}...")
            fig = plot_map_with_scans(
                all_trajectories[method_name],
                all_scan_data[method_name],
                f'{self.sequence_name}: {method_name} Map',
                downsample=3
            )
            save_figure(fig, self.seq_dir / f'map_{method_name.lower().replace(" ", "_").replace("-", "_")}.png')

        summary = {
            'sequence': self.sequence_name,
            'methods': method_summaries,
            'rankings': rankings,
            'best_method': best_method_name
        }

        save_json(summary, self.json_dir / 'all_methods_summary.json')

        return summary, best_method_name, all_trajectories[best_method_name], all_scan_data[best_method_name], ekf_results, processor, all_detailed_results[best_method_name]

    def run_loop_closure(self, best_method_name, best_trajectory, best_scan_data, ekf_results, processor, detailed_results):
        print(f"\nRunning loop closure with {best_method_name}...")

        fitness_before = np.mean([r['fitness'] for r in detailed_results])
        rmse_before = np.mean([r['inlier_rmse'] for r in detailed_results])

        icp_methods_map = {
            'Point-to-Point': PointToPointICP(),
            'Point-to-Plane': PointToPlaneICP(),
            'Point-to-Line': PointToLineICP(),
            'GICP': GICP()
        }

        best_icp_method = icp_methods_map[best_method_name]

        loop_closure = LoopClosure(search_radius=2.5, temporal_threshold=30.0,
                                   fitness_threshold=0.65, rmse_threshold=0.12)

        keyframe_idx = 0
        for i, res in enumerate(ekf_results):
            current_pose = res['ekf_pose']

            if not processor.is_keyframe(current_pose):
                continue

            if keyframe_idx >= len(best_trajectory):
                break

            current_pcd = processor.preprocess(
                res['scan_ranges'],
                res['scan_angles'],
                compute_normals=(best_method_name in ["Point-to-Plane", "Point-to-Line", "GICP"])
            )

            if len(current_pcd.points) < 10:
                continue

            loop_closure.add_keyframe(res['timestamp'], best_trajectory[keyframe_idx], current_pcd)

            if len(loop_closure.keyframes) >= 10:
                new_loops = loop_closure.detect_loops(best_icp_method)
                if new_loops:
                    print(f"  Detected {len(new_loops)} new loop(s)")

            keyframe_idx += 1

        loop_info = loop_closure.get_loop_info()
        print(f"  Total loops detected: {loop_info['num_loops']}")

        if loop_info['num_loops'] > 0:
            optimized_trajectory = loop_closure.optimize_pose_graph(best_trajectory)

            fig = plot_loop_closure_comparison(
                best_trajectory, optimized_trajectory, loop_info['loops'],
                scan_data=best_scan_data,
                title=f'{self.sequence_name}: Loop Closure Effect',
                downsample=5
            )
            save_figure(fig, self.seq_dir / 'loop_closure_comparison.png')

            loop_closure_results = []
            for i, pose in enumerate(optimized_trajectory):
                loop_closure_results.append({
                    'keyframe_idx': i,
                    'x': pose[0],
                    'y': pose[1],
                    'theta': pose[2]
                })

            save_csv(loop_closure_results, self.csv_dir / 'loop_closure_optimized.csv')

            loop_summary = {
                'num_keyframes': loop_info['num_keyframes'],
                'num_loops': loop_info['num_loops'],
                'loops': loop_info['loops']
            }
            save_json(loop_summary, self.json_dir / 'loop_closure_summary.json')

            print(f"  Loop closure completed successfully!")
            print(f"  Re-evaluating trajectory after loop closure...")

            detailed_results_after = self.reevaluate_trajectory(
                optimized_trajectory, ekf_results, best_icp_method, processor
            )

            if detailed_results_after:
                fitness_after = np.mean([r['fitness'] for r in detailed_results_after])
                rmse_after = np.mean([r['inlier_rmse'] for r in detailed_results_after])

                before_stats = {
                    'avg_fitness': fitness_before,
                    'avg_rmse': rmse_before
                }
                after_stats = {
                    'avg_fitness': fitness_after,
                    'avg_rmse': rmse_after
                }

                fig = self.plot_loop_closure_performance(
                    before_stats, after_stats, loop_info['num_loops'],
                    f'{self.sequence_name}: Loop Closure Performance Impact'
                )
                save_figure(fig, self.seq_dir / 'loop_closure_performance.png')
                print(f"  Loop closure performance comparison saved!")

                print(f"  Before - Fitness: {fitness_before:.4f}, RMSE: {rmse_before:.4f}m")
                print(f"  After  - Fitness: {fitness_after:.4f}, RMSE: {rmse_after:.4f}m")
            else:
                fitness_after = fitness_before
                rmse_after = rmse_before

            loop_stats = {
                'num_loops': loop_info['num_loops'],
                'fitness_before': fitness_before,
                'fitness_after': fitness_after,
                'rmse_before': rmse_before,
                'rmse_after': rmse_after
            }
        else:
            print(f"  No loops detected (trajectory may not form a loop)")

            loop_stats = {
                'num_loops': 0,
                'fitness_before': fitness_before,
                'fitness_after': fitness_before,
                'rmse_before': rmse_before,
                'rmse_after': rmse_before
            }

        processor.reset_keyframe()
        return loop_stats


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
    loop_analysis_data = {}

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

            loop_stats = exp.run_loop_closure(best_method, best_traj, best_scan_data, ekf_results, processor, detailed_results)

            # Collect data for loop closure analysis
            loop_analysis_data[seq_name] = {
                'best_method': best_method,
                'num_loops': loop_stats['num_loops'],
                'fitness_before': loop_stats['fitness_before'],
                'fitness_after': loop_stats['fitness_after'],
                'rmse_before': loop_stats['rmse_before'],
                'rmse_after': loop_stats['rmse_after']
            }

    if all_results:
        output_json = output_path / 'figures' / 'all_sequences_results.json'
        save_json(all_results, output_json)

    # Generate loop closure analysis plot
    if loop_analysis_data:
        fig = plot_loop_closure_analysis(loop_analysis_data, 'Loop Closure Impact Analysis')
        save_figure(fig, output_path / 'figures' / 'loop_closure_analysis.png')
        print("\n  Loop closure analysis plot saved!")

    print("\n" + "="*60)
    print("All experiments completed!")
    print("="*60)


if __name__ == '__main__':
    main()
