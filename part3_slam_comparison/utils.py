import json
import csv
import os
import numpy as np
import matplotlib.pyplot as plt


METHOD_COLORS = {
    'Wheel': '#1f77b4',
    'EKF': '#d62728',
    'ICP': '#9b59b6',
    'SLAM': '#2ca02c'
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


def align_trajectory_to_start(trajectory):
    if len(trajectory) == 0:
        return trajectory

    traj = np.array(trajectory)
    start_x, start_y, start_theta = traj[0]

    cos_theta = np.cos(-start_theta)
    sin_theta = np.sin(-start_theta)

    aligned = []
    for x, y, theta in traj:
        dx = x - start_x
        dy = y - start_y

        new_x = cos_theta * dx - sin_theta * dy
        new_y = sin_theta * dx + cos_theta * dy
        new_theta = theta - start_theta

        new_theta = np.arctan2(np.sin(new_theta), np.cos(new_theta))

        aligned.append([new_x, new_y, new_theta])

    return np.array(aligned)


def plot_all_trajectories(trajectories, title='Trajectory Comparison', align_to_start=True):
    fig, ax = plt.subplots(figsize=(12, 10))

    markers = ['o', '^', 's', 'D']

    for i, (name, traj) in enumerate(trajectories.items()):
        if align_to_start:
            traj = align_trajectory_to_start(traj)

        x = [p[0] for p in traj]
        y = [p[1] for p in traj]
        color = get_method_color(name)

        if name == 'SLAM' and len(x) > 10:
            step = max(1, len(x) // 100)
            x_down = x[::step]
            y_down = y[::step]
            ax.plot(x_down, y_down, color=color, label=name, linewidth=2, alpha=0.85)
        else:
            ax.plot(x, y, color=color, label=name, linewidth=2, alpha=0.85)

        ax.plot(x[-1], y[-1], marker=markers[i % len(markers)],
                color=color, markersize=10, markeredgecolor='black', markeredgewidth=1.5)

    ax.plot(0, 0, 'go', markersize=15,
            label='Start', zorder=10, markeredgecolor='black', markeredgewidth=2)

    ax.set_xlabel('X [m]', fontsize=13, fontweight='bold')
    ax.set_ylabel('Y [m]', fontsize=13, fontweight='bold')
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.legend(loc='best', fontsize=11, framealpha=0.95)
    ax.grid(True, alpha=0.3, linestyle='--')
    ax.xaxis.set_major_locator(plt.MultipleLocator(1.0))
    ax.yaxis.set_major_locator(plt.MultipleLocator(1.0))
    ax.set_aspect('equal')

    return fig


def plot_time_series(trajectories_ts, title='Time Series Comparison'):
    fig, axes = plt.subplots(3, 1, figsize=(14, 12))
    fig.suptitle(title, fontsize=16, fontweight='bold')

    labels = ['X Position [m]', 'Y Position [m]', 'Heading [deg]']

    # Find the earliest timestamp to use as t0
    t0 = None
    for traj_ts in trajectories_ts.values():
        if len(traj_ts) > 0:
            if t0 is None or traj_ts[0, 0] < t0:
                t0 = traj_ts[0, 0]

    for i, label in enumerate(labels):
        ax = axes[i]

        for name, traj_ts in trajectories_ts.items():
            if len(traj_ts) == 0:
                continue

            # Convert to relative time
            timestamps = traj_ts[:, 0] - t0
            data = traj_ts[:, i+1]

            if i == 2:
                data = np.degrees(data)

            color = get_method_color(name)
            ax.plot(timestamps, data, color=color, label=name, linewidth=1.5, alpha=0.85)

        ax.set_ylabel(label, fontsize=12, fontweight='bold')
        ax.legend(loc='best', fontsize=10)
        ax.grid(True, alpha=0.3, linestyle='--')

    axes[-1].set_xlabel('Time [s]', fontsize=12, fontweight='bold')
    plt.tight_layout()

    return fig


def compute_trajectory_length(trajectory):
    total_dist = 0.0
    for i in range(1, len(trajectory)):
        dx = trajectory[i][0] - trajectory[i-1][0]
        dy = trajectory[i][1] - trajectory[i-1][1]
        total_dist += np.sqrt(dx**2 + dy**2)
    return total_dist


def compute_drift(trajectory):
    if len(trajectory) < 2:
        return 0.0
    start = trajectory[0]
    end = trajectory[-1]
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    return np.sqrt(dx**2 + dy**2)


def compute_drift_metrics(trajectory):
    if len(trajectory) < 2:
        return {
            'translational_error_m': 0.0,
            'trajectory_length_m': 0.0,
            'drift_rate_percent': 0.0
        }
    translational_error = compute_drift(trajectory)
    trajectory_length = compute_trajectory_length(trajectory)
    drift_rate = (translational_error / trajectory_length * 100.0) if trajectory_length > 0 else 0.0
    return {
        'translational_error_m': float(translational_error),
        'trajectory_length_m': float(trajectory_length),
        'drift_rate_percent': float(drift_rate)
    }


def compute_trajectory_errors(traj_ref, traj_test):
    if len(traj_ref) == 0 or len(traj_test) == 0:
        return None, None

    min_len = min(len(traj_ref), len(traj_test))
    errors = []

    for i in range(min_len):
        dx = traj_test[i][0] - traj_ref[i][0]
        dy = traj_test[i][1] - traj_ref[i][1]
        error = np.sqrt(dx**2 + dy**2)
        errors.append(error)

    if len(errors) == 0:
        return None, None

    return float(np.mean(errors)), float(np.max(errors))


def compute_map_metrics(map_data):
    if map_data is None:
        return None

    total_cells = map_data.size
    occupied = np.sum(map_data == 100)
    free = np.sum(map_data == 0)
    known = occupied + free

    return {
        'total_cells': int(total_cells),
        'occupied_cells': int(occupied),
        'free_cells': int(free),
        'known_cells': int(known),
        'occupied_percent': float(100 * occupied / total_cells) if total_cells > 0 else 0.0,
        'known_percent': float(100 * known / total_cells) if total_cells > 0 else 0.0
    }


def compute_common_boundary_metrics(icp_map, icp_metadata, slam_map, slam_metadata):
    if icp_map is None or slam_map is None:
        return None

    def get_content_bounds(map_data, metadata):
        known_mask = (map_data == 0) | (map_data == 100)
        if not np.any(known_mask):
            return None

        rows, cols = np.where(known_mask)
        min_row, max_row = rows.min(), rows.max()
        min_col, max_col = cols.min(), cols.max()

        min_x = metadata['origin_x'] + min_col * metadata['resolution']
        max_x = metadata['origin_x'] + (max_col + 1) * metadata['resolution']
        min_y = metadata['origin_y'] + min_row * metadata['resolution']
        max_y = metadata['origin_y'] + (max_row + 1) * metadata['resolution']

        return [min_x, max_x, min_y, max_y]

    icp_bounds = get_content_bounds(icp_map, icp_metadata)
    slam_bounds = get_content_bounds(slam_map, slam_metadata)

    if icp_bounds is None or slam_bounds is None:
        return None

    common_bounds = [
        min(icp_bounds[0], slam_bounds[0]),
        max(icp_bounds[1], slam_bounds[1]),
        min(icp_bounds[2], slam_bounds[2]),
        max(icp_bounds[3], slam_bounds[3])
    ]

    resolution = min(icp_metadata['resolution'], slam_metadata['resolution'])
    common_width = int(np.ceil((common_bounds[1] - common_bounds[0]) / resolution))
    common_height = int(np.ceil((common_bounds[3] - common_bounds[2]) / resolution))
    common_total_cells = common_width * common_height

    def count_known_in_bounds(map_data, metadata, bounds, res):
        count = 0
        for row in range(map_data.shape[0]):
            for col in range(map_data.shape[1]):
                if map_data[row, col] == 0 or map_data[row, col] == 100:
                    x = metadata['origin_x'] + col * metadata['resolution']
                    y = metadata['origin_y'] + row * metadata['resolution']
                    if bounds[0] <= x < bounds[1] and bounds[2] <= y < bounds[3]:
                        count += 1
        return count

    icp_known_in_common = count_known_in_bounds(icp_map, icp_metadata, common_bounds, resolution)
    slam_known_in_common = count_known_in_bounds(slam_map, slam_metadata, common_bounds, resolution)

    return {
        'common_boundary': {
            'min_x': float(common_bounds[0]),
            'max_x': float(common_bounds[1]),
            'min_y': float(common_bounds[2]),
            'max_y': float(common_bounds[3]),
            'width': common_width,
            'height': common_height,
            'total_cells': common_total_cells,
            'resolution': float(resolution)
        },
        'icp': {
            'known_cells_in_common': int(icp_known_in_common),
            'coverage_percent': float(100 * icp_known_in_common / common_total_cells) if common_total_cells > 0 else 0.0
        },
        'slam': {
            'known_cells_in_common': int(slam_known_in_common),
            'coverage_percent': float(100 * slam_known_in_common / common_total_cells) if common_total_cells > 0 else 0.0
        }
    }
