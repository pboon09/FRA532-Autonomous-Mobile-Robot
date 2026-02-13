#!/usr/bin/env python3

import os
import math
import numpy as np
from pathlib import Path

from ekf import EKF
from wheel_odometry import WheelOdometry
from bag_reader import BagReader
from utils import (
    save_figure, save_json, save_csv,
    plot_trajectory, plot_time_series_combined,
    plot_innovation_analysis, compute_heading_rmse,
    plot_velocity_comparison, plot_velocity_comparison_last_minute
)


class Experiment:
    def __init__(self, bag_path, output_dir, sequence_name):
        self.bag_path = bag_path
        self.output_dir = Path(output_dir)
        self.sequence_name = sequence_name

        self.seq_dir = self.output_dir / 'figures' / sequence_name
        self.json_dir = self.seq_dir / 'json'
        self.csv_dir = self.seq_dir / 'csv'

        self.default_Q = [0.001, 0.001, 0.01, 0.05, 0.05, 0.01]
        self.default_R = 0.1
        self.default_P0 = [0.01, 0.01, 0.1, 0.5, 0.5, 0.1]

        self.wheel_radius = 0.033
        self.track_width = 0.160

        os.makedirs(self.seq_dir, exist_ok=True)
        os.makedirs(self.json_dir, exist_ok=True)
        os.makedirs(self.csv_dir, exist_ok=True)

    def run_single(self, Q=None, R=None, P0=None):
        if Q is None:
            Q = self.default_Q
        if R is None:
            R = self.default_R
        if P0 is None:
            P0 = self.default_P0

        reader = BagReader(self.bag_path)
        data = reader.get_synchronized_data()

        if not data:
            print(f"No data found in {self.bag_path}")
            return None

        wheel_odom = WheelOdometry(self.wheel_radius, self.track_width)
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

            v = np.clip(v, -0.22, 0.22)
            omega = np.clip(omega, -2.84, 2.84)

            ekf.predict(v, omega, dt)

            if imu_offset is None:
                imu_offset = d['imu_yaw']

            imu_corrected = d['imu_yaw'] - imu_offset
            imu_corrected = math.atan2(math.sin(imu_corrected), math.cos(imu_corrected))

            z = np.array([imu_corrected])
            k_theta, innovation, S = ekf.correct(z, R_matrix)

            wheel_pose = wheel_odom.get_pose()
            ekf_state = ekf.get_state()

            nis = (innovation ** 2) / S if S > 0 else 0.0
            P_cov = ekf.get_covariance()

            theta_ctrl = ekf_state[2]
            vx_ctrl = v * math.cos(theta_ctrl)
            vy_ctrl = v * math.sin(theta_ctrl)

            results.append({
                'timestamp': t,
                'x_wheel': wheel_pose[0],
                'y_wheel': wheel_pose[1],
                'theta_wheel': wheel_pose[2],
                'x_ekf': ekf_state[0],
                'y_ekf': ekf_state[1],
                'theta_ekf': ekf_state[2],
                'vx_ekf': ekf_state[3],
                'vy_ekf': ekf_state[4],
                'wz_ekf': ekf_state[5],
                'vx_ctrl': vx_ctrl,
                'vy_ctrl': vy_ctrl,
                'wz_ctrl': omega,
                'imu_corrected': imu_corrected,
                'innovation': innovation,
                'innovation_cov': S,
                'nis': nis,
                'kalman_gain': k_theta,
                'P': P_cov.copy()
            })

            prev_time = t

        return results

    def run_baseline(self):
        print(f"Running baseline for {self.sequence_name}...")

        results = self.run_single()
        if not results:
            return None

        times = [r['timestamp'] - results[0]['timestamp'] for r in results]
        wheel_x = [r['x_wheel'] for r in results]
        wheel_y = [r['y_wheel'] for r in results]
        wheel_theta = [r['theta_wheel'] for r in results]
        ekf_x = [r['x_ekf'] for r in results]
        ekf_y = [r['y_ekf'] for r in results]
        ekf_theta = [r['theta_ekf'] for r in results]
        vx_ekf = [r['vx_ekf'] for r in results]
        vy_ekf = [r['vy_ekf'] for r in results]
        wz_ekf = [r['wz_ekf'] for r in results]
        vx_ctrl = [r['vx_ctrl'] for r in results]
        vy_ctrl = [r['vy_ctrl'] for r in results]
        wz_ctrl = [r['wz_ctrl'] for r in results]
        imu_corrected = [r['imu_corrected'] for r in results]
        innovations = [r['innovation'] for r in results]
        wheel_heading_rmse = compute_heading_rmse(wheel_theta, imu_corrected)
        ekf_heading_rmse = compute_heading_rmse(ekf_theta, imu_corrected)

        fig = plot_trajectory(wheel_x, wheel_y, ekf_x, ekf_y,
                              f'{self.sequence_name}: Trajectory Comparison')
        save_figure(fig, self.seq_dir / 'trajectory.png')

        fig = plot_time_series_combined(times, wheel_x, wheel_y, wheel_theta,
                                        ekf_x, ekf_y, ekf_theta, imu_corrected,
                                        f'{self.sequence_name}: Time Series Comparison')
        save_figure(fig, self.seq_dir / 'time_series.png')

        fig = plot_innovation_analysis(times, innovations,
                                       f'{self.sequence_name}: Innovation Analysis')
        save_figure(fig, self.seq_dir / 'innovation_analysis.png')

        fig = plot_velocity_comparison(times, vx_ctrl, vy_ctrl, wz_ctrl,
                                      vx_ekf, vy_ekf, wz_ekf,
                                      f'{self.sequence_name}: Velocity Comparison')
        save_figure(fig, self.seq_dir / 'velocity_comparison.png')

        save_csv(results, self.csv_dir / 'baseline_data.csv')

        innovation_mean = float(np.mean(innovations))
        innovation_std = float(np.std(innovations))

        summary = {
            'sequence': self.sequence_name,
            'parameters': {
                'Q': self.default_Q,
                'R': self.default_R,
                'P0': self.default_P0,
                'wheel_radius': self.wheel_radius,
                'track_width': self.track_width
            },
            'data': {
                'num_samples': len(results),
                'duration_s': times[-1]
            },
            'wheel_odometry': {
                'final_x': wheel_x[-1],
                'final_y': wheel_y[-1],
                'final_theta_deg': math.degrees(wheel_theta[-1]),
                'deviation_from_imu_deg': math.degrees(wheel_heading_rmse)
            },
            'ekf': {
                'final_x': ekf_x[-1],
                'final_y': ekf_y[-1],
                'final_theta_deg': math.degrees(ekf_theta[-1]),
                'deviation_from_imu_deg': math.degrees(ekf_heading_rmse)
            },
            'innovation': {
                'mean_deg': math.degrees(innovation_mean),
                'std_deg': math.degrees(innovation_std)
            }
        }

        save_json(summary, self.json_dir / 'baseline.json')

        print(f"  Samples: {len(results)}, Duration: {times[-1]:.1f}s")
        print(f"  Wheel deviation from IMU: {math.degrees(wheel_heading_rmse):.2f}°")
        print(f"  EKF deviation from IMU:   {math.degrees(ekf_heading_rmse):.2f}°")
        print(f"  Innovation: mean={math.degrees(innovation_mean):.4f}°, std={math.degrees(innovation_std):.4f}°")

        return summary


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

        exp = Experiment(bag_path, output_path, seq_name)
        results = exp.run_baseline()
        if results:
            all_results[seq_name] = results

    if all_results:
        output_json = output_path / 'figures' / 'all_results.json'
        save_json(all_results, output_json)

    print("\nAll experiments completed!")


if __name__ == '__main__':
    main()
