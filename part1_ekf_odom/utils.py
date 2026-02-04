import json
import csv
import os
import numpy as np
import matplotlib.pyplot as plt


def save_figure(fig, path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fig.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)


def save_json(data, path):
    os.makedirs(os.path.dirname(path), exist_ok=True)

    def convert(obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if isinstance(obj, (np.float32, np.float64)):
            return float(obj)
        if isinstance(obj, (np.int32, np.int64)):
            return int(obj)
        return obj

    with open(path, 'w') as f:
        json.dump(data, f, indent=2, default=convert)


def save_csv(data, path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    if not data:
        return

    keys = data[0].keys()
    with open(path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(data)


def plot_trajectory(wheel_x, wheel_y, ekf_x, ekf_y, title='Trajectory Comparison'):
    fig, ax = plt.subplots(figsize=(10, 8))

    ax.plot(wheel_x, wheel_y, 'b-', label='Wheel Odometry', linewidth=1.5)
    ax.plot(ekf_x, ekf_y, 'r-', label='EKF', linewidth=1.5)

    ax.plot(wheel_x[0], wheel_y[0], 'go', markersize=10, label='Start')
    ax.plot(wheel_x[-1], wheel_y[-1], 'b^', markersize=8, label='Wheel End')
    ax.plot(ekf_x[-1], ekf_y[-1], 'r^', markersize=8, label='EKF End')

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(title)
    ax.legend()
    ax.grid(True)
    ax.set_aspect('equal')

    return fig


def plot_time_series_combined(times, wheel_x, wheel_y, wheel_theta, ekf_x, ekf_y, ekf_theta, imu_theta, bag_name):
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

    fig.suptitle(bag_name, fontsize=14, fontweight='bold')

    axes[0].plot(times, wheel_x, 'b-', label='Wheel Odometry', linewidth=1)
    axes[0].plot(times, ekf_x, 'r-', label='EKF', linewidth=1)
    axes[0].set_ylabel('X (m)')
    axes[0].legend(loc='upper right')
    axes[0].grid(True)

    axes[1].plot(times, wheel_y, 'b-', label='Wheel Odometry', linewidth=1)
    axes[1].plot(times, ekf_y, 'r-', label='EKF', linewidth=1)
    axes[1].set_ylabel('Y (m)')
    axes[1].legend(loc='upper right')
    axes[1].grid(True)

    axes[2].plot(times, np.rad2deg(wheel_theta), 'b-', label='Wheel Odometry', linewidth=1)
    axes[2].plot(times, np.rad2deg(ekf_theta), 'r-', label='EKF', linewidth=1)
    axes[2].plot(times, np.rad2deg(imu_theta), 'g--', label='IMU', linewidth=1, alpha=0.7)
    axes[2].set_xlabel('Time (s)')
    axes[2].set_ylabel('θ (deg)')
    axes[2].legend(loc='upper right')
    axes[2].grid(True)

    plt.tight_layout()
    return fig


def plot_innovation_analysis(times, innovations, title='Innovation Analysis'):
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))

    innovations_deg = np.rad2deg(innovations)
    mean_val = np.mean(innovations_deg)
    std_val = np.std(innovations_deg)

    ax1 = axes[0]
    ax1.plot(times, innovations_deg, 'b-', alpha=0.7, linewidth=0.8)
    ax1.axhline(y=0, color='k', linestyle='-', linewidth=1)
    ax1.axhline(y=mean_val, color='g', linestyle='--', linewidth=1.5, label=f'Mean: {mean_val:.4f}°')
    ax1.fill_between(times, -2*std_val, 2*std_val, alpha=0.2, color='orange', label=f'±2σ')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Innovation (°)')
    ax1.set_title('Innovation Time Series')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)

    ax2 = axes[1]
    ax2.hist(innovations_deg, bins=50, edgecolor='black', alpha=0.7, orientation='vertical')
    ax2.axvline(x=0, color='r', linestyle='--', linewidth=2, label='Zero')
    ax2.axvline(x=mean_val, color='g', linestyle='-', linewidth=2, label=f'Mean: {mean_val:.4f}°')
    ax2.set_xlabel('Innovation (°)')
    ax2.set_ylabel('Count')
    ax2.set_title('Innovation Distribution')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    fig.suptitle(f'{title}\nMean: {mean_val:.4f}°, Std: {std_val:.4f}°', fontsize=12, fontweight='bold')
    plt.tight_layout()

    return fig


def compute_heading_rmse(theta_method, theta_imu):
    diff = np.array(theta_method) - np.array(theta_imu)
    diff = np.arctan2(np.sin(diff), np.cos(diff))
    return np.sqrt(np.mean(diff**2))
