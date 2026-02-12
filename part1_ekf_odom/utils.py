import json
import csv
import os
import numpy as np
import matplotlib.pyplot as plt


METHOD_COLORS = {
    'Wheel': '#1f77b4',
    'EKF': '#d62728',
    'IMU': '#2ca02c'
}


def get_method_color(method_name):
    return METHOD_COLORS.get(method_name, '#888888')


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
    fig, ax = plt.subplots(figsize=(12, 10))

    wheel_color = get_method_color('Wheel')
    ekf_color = get_method_color('EKF')

    ax.plot(wheel_x, wheel_y, color=wheel_color, label='Wheel Odometry', linewidth=2, alpha=0.85)
    ax.plot(ekf_x, ekf_y, color=ekf_color, label='EKF', linewidth=2, alpha=0.85)

    ax.plot(wheel_x[0], wheel_y[0], 'go', markersize=15, label='Start',
            markeredgecolor='black', markeredgewidth=2, zorder=10)
    ax.plot(wheel_x[-1], wheel_y[-1], '^', color=wheel_color, markersize=10,
            label='Wheel End', markeredgecolor='black', markeredgewidth=1.5)
    ax.plot(ekf_x[-1], ekf_y[-1], '^', color=ekf_color, markersize=10,
            label='EKF End', markeredgecolor='black', markeredgewidth=1.5)

    ax.set_xlabel('X [m]', fontsize=13, fontweight='bold')
    ax.set_ylabel('Y [m]', fontsize=13, fontweight='bold')
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.legend(loc='best', fontsize=11, framealpha=0.95)
    ax.grid(True, alpha=0.3, linestyle='--')
    ax.xaxis.set_major_locator(plt.MultipleLocator(1.0))
    ax.yaxis.set_major_locator(plt.MultipleLocator(1.0))
    ax.set_aspect('equal')

    return fig


def plot_time_series_combined(times, wheel_x, wheel_y, wheel_theta, ekf_x, ekf_y, ekf_theta, imu_theta, title):
    fig, axes = plt.subplots(3, 1, figsize=(14, 12), sharex=True)

    fig.suptitle(title, fontsize=16, fontweight='bold')

    wheel_color = get_method_color('Wheel')
    ekf_color = get_method_color('EKF')
    imu_color = get_method_color('IMU')

    axes[0].plot(times, wheel_x, color=wheel_color, label='Wheel Odometry', linewidth=1.5, alpha=0.8)
    axes[0].plot(times, ekf_x, color=ekf_color, label='EKF', linewidth=1.5, alpha=0.9)
    axes[0].set_ylabel('X [m]', fontsize=13, fontweight='bold')
    axes[0].legend(loc='best', fontsize=10)
    axes[0].grid(True, alpha=0.3, linestyle='--')

    axes[1].plot(times, wheel_y, color=wheel_color, label='Wheel Odometry', linewidth=1.5, alpha=0.8)
    axes[1].plot(times, ekf_y, color=ekf_color, label='EKF', linewidth=1.5, alpha=0.9)
    axes[1].set_ylabel('Y [m]', fontsize=13, fontweight='bold')
    axes[1].legend(loc='best', fontsize=10)
    axes[1].grid(True, alpha=0.3, linestyle='--')

    axes[2].plot(times, np.rad2deg(wheel_theta), color=wheel_color, label='Wheel Odometry', linewidth=1.5, alpha=0.8)
    axes[2].plot(times, np.rad2deg(ekf_theta), color=ekf_color, label='EKF', linewidth=1.5, alpha=0.9)
    axes[2].plot(times, np.rad2deg(imu_theta), color=imu_color, linestyle='--', label='IMU', linewidth=1.5, alpha=0.7)
    axes[2].set_xlabel('Time [s]', fontsize=13, fontweight='bold')
    axes[2].set_ylabel('Theta [deg]', fontsize=13, fontweight='bold')
    axes[2].legend(loc='best', fontsize=10)
    axes[2].grid(True, alpha=0.3, linestyle='--')

    plt.tight_layout()
    return fig


def plot_innovation_analysis(times, innovations, title='Innovation Analysis'):
    fig, axes = plt.subplots(2, 1, figsize=(14, 10))

    innovations_deg = np.rad2deg(innovations)
    mean_val = np.mean(innovations_deg)
    std_val = np.std(innovations_deg)

    ax1 = axes[0]
    ax1.plot(times, innovations_deg, color='#1f77b4', alpha=0.7, linewidth=1.5)
    ax1.axhline(y=0, color='k', linestyle='-', linewidth=1)
    ax1.axhline(y=mean_val, color='#2ca02c', linestyle='--', linewidth=1.5, label=f'Mean: {mean_val:.4f}°')
    ax1.fill_between(times, -2*std_val, 2*std_val, alpha=0.2, color='orange', label=f'±2σ')
    ax1.set_xlabel('Time [s]', fontsize=13, fontweight='bold')
    ax1.set_ylabel('Innovation [deg]', fontsize=13, fontweight='bold')
    ax1.set_title('Innovation Time Series', fontsize=12, fontweight='bold')
    ax1.legend(loc='best', fontsize=10)
    ax1.grid(True, alpha=0.3, linestyle='--')

    ax2 = axes[1]
    ax2.hist(innovations_deg, bins=50, edgecolor='black', alpha=0.7, orientation='vertical', color='#1f77b4')
    ax2.axvline(x=0, color='r', linestyle='--', linewidth=2, label='Zero')
    ax2.axvline(x=mean_val, color='#2ca02c', linestyle='-', linewidth=2, label=f'Mean: {mean_val:.4f}°')
    ax2.set_xlabel('Innovation [deg]', fontsize=13, fontweight='bold')
    ax2.set_ylabel('Count', fontsize=13, fontweight='bold')
    ax2.set_title('Innovation Distribution', fontsize=12, fontweight='bold')
    ax2.legend(loc='best', fontsize=10)
    ax2.grid(True, alpha=0.3, linestyle='--')

    fig.suptitle(f'{title}\nMean: {mean_val:.4f}°, Std: {std_val:.4f}°', fontsize=16, fontweight='bold')
    plt.tight_layout()

    return fig


def compute_heading_rmse(theta_method, theta_imu):
    diff = np.array(theta_method) - np.array(theta_imu)
    diff = np.arctan2(np.sin(diff), np.cos(diff))
    return np.sqrt(np.mean(diff**2))
