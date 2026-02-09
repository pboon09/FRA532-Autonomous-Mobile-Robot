import json
import csv
import os
import numpy as np
import matplotlib.pyplot as plt


METHOD_COLORS = {
    'Point-to-Point': '#2ca02c',
    'Wheel': '#1f77b4',
    'EKF': '#d62728'
}


def get_method_color(method_name):
    return METHOD_COLORS.get(method_name, '#9b59b6')


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


def plot_all_trajectories(trajectories, title='Trajectory Comparison'):
    fig, ax = plt.subplots(figsize=(12, 10))

    markers = ['o', '^', 's', 'D', 'v', '<', '>', 'p', '*']

    for i, (name, traj) in enumerate(trajectories.items()):
        x = [p[0] for p in traj]
        y = [p[1] for p in traj]
        color = get_method_color(name)
        ax.plot(x, y, color=color, label=name, linewidth=2, alpha=0.85)
        ax.plot(x[-1], y[-1], marker=markers[i % len(markers)],
                color=color, markersize=10, markeredgecolor='black', markeredgewidth=1.5)

    first_traj = list(trajectories.values())[0]
    ax.plot(first_traj[0][0], first_traj[0][1], 'go', markersize=15,
            label='Start', zorder=10, markeredgecolor='black', markeredgewidth=2)

    ax.set_xlabel('X [m]', fontsize=13, fontweight='bold')
    ax.set_ylabel('Y [m]', fontsize=13, fontweight='bold')
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.legend(loc='best', fontsize=11, framealpha=0.95)
    ax.grid(True, alpha=0.3, linestyle='--')
    ax.set_aspect('equal')

    return fig


def compute_convergence_rate(detailed_results):
    iterations = [r['iterations'] for r in detailed_results]
    return np.mean(iterations)


def compute_consistency_score(detailed_results):
    fitness = np.array([r['fitness'] for r in detailed_results])
    rmse = np.array([r['inlier_rmse'] for r in detailed_results])

    fitness_std = np.std(fitness)
    rmse_std = np.std(rmse)

    consistency = 1.0 - (fitness_std + rmse_std) / 2.0
    return max(0.0, min(1.0, consistency))


def compute_trajectory_drift(trajectory):
    total_dist = 0.0
    for i in range(1, len(trajectory)):
        dx = trajectory[i][0] - trajectory[i-1][0]
        dy = trajectory[i][1] - trajectory[i-1][1]
        total_dist += np.sqrt(dx**2 + dy**2)
    return total_dist


def plot_map_with_scans(trajectory, scan_data, title='Map with LiDAR Scans', downsample=5):
    fig, ax = plt.subplots(figsize=(14, 12))

    all_points_x = []
    all_points_y = []

    for i in range(0, len(trajectory), downsample):
        if i >= len(scan_data):
            break

        pose = trajectory[i]
        x, y, theta = pose[0], pose[1], pose[2]

        ranges = scan_data[i]['ranges']
        angles = scan_data[i]['angles']

        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)

        for r, angle in zip(ranges, angles):
            if r < 0.1 or r > 10.0:
                continue

            local_x = r * np.cos(angle)
            local_y = r * np.sin(angle)

            global_x = x + cos_theta * local_x - sin_theta * local_y
            global_y = y + sin_theta * local_x + cos_theta * local_y

            all_points_x.append(global_x)
            all_points_y.append(global_y)

    if all_points_x:
        ax.scatter(all_points_x, all_points_y, c='gray', s=1.0, alpha=0.5, label='LiDAR Points')

    traj_x = [p[0] for p in trajectory]
    traj_y = [p[1] for p in trajectory]
    ax.plot(traj_x, traj_y, 'b-', linewidth=2.5, label='Trajectory', zorder=5)
    ax.plot(traj_x[0], traj_y[0], 'go', markersize=15, label='Start', zorder=10,
            markeredgecolor='black', markeredgewidth=2)
    ax.plot(traj_x[-1], traj_y[-1], 'r^', markersize=12, label='End', zorder=10,
            markeredgecolor='black', markeredgewidth=2)

    ax.set_xlabel('X [m]', fontsize=13, fontweight='bold')
    ax.set_ylabel('Y [m]', fontsize=13, fontweight='bold')
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.legend(loc='best', fontsize=11, framealpha=0.95)
    ax.grid(True, alpha=0.3, linestyle='--')
    ax.set_aspect('equal')

    return fig
