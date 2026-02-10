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


def bresenham(x0, y0, x1, y1):
    cells = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    x, y = x0, y0
    while True:
        cells.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy

    return cells


def create_occupancy_grid(trajectory, scan_data, resolution=0.05,
                         min_range=0.12, max_range=3.5,
                         occupied_threshold=0.6, free_threshold=0.4):
    hit_count = {}
    miss_count = {}

    def world_to_map(x, y):
        map_x = int(np.floor(x / resolution))
        map_y = int(np.floor(y / resolution))
        return map_x, map_y

    for i in range(len(trajectory)):
        if i >= len(scan_data):
            break

        pose = trajectory[i]
        robot_x, robot_y, robot_theta = pose[0], pose[1], pose[2]
        robot_mx, robot_my = world_to_map(robot_x, robot_y)

        ranges = np.array(scan_data[i]['ranges'])
        angles = np.array(scan_data[i]['angles'])

        valid = np.isfinite(ranges) & (ranges > min_range) & (ranges < max_range)
        ranges = ranges[valid]
        angles = angles[valid]

        for r, angle in zip(ranges, angles):
            global_angle = robot_theta + angle
            end_x = robot_x + r * np.cos(global_angle)
            end_y = robot_y + r * np.sin(global_angle)

            end_mx, end_my = world_to_map(end_x, end_y)

            hit_count[(end_mx, end_my)] = hit_count.get((end_mx, end_my), 0) + 1

            ray_cells = bresenham(robot_mx, robot_my, end_mx, end_my)
            for cx, cy in ray_cells[:-1]:
                miss_count[(cx, cy)] = miss_count.get((cx, cy), 0) + 1

    all_cells = set(hit_count.keys()) | set(miss_count.keys())
    if not all_cells:
        return np.array([]), {}

    all_x = [cell[0] for cell in all_cells]
    all_y = [cell[1] for cell in all_cells]
    min_mx, max_mx = min(all_x), max(all_x)
    min_my, max_my = min(all_y), max(all_y)

    width = max_mx - min_mx + 1
    height = max_my - min_my + 1
    occupancy_grid = np.full((height, width), -1, dtype=np.int8)

    for (mx, my) in all_cells:
        if min_mx <= mx <= max_mx and min_my <= my <= max_my:
            hits = hit_count.get((mx, my), 0)
            misses = miss_count.get((mx, my), 0)
            total = hits + misses

            if total > 0:
                hit_ratio = hits / total
                grid_x = mx - min_mx
                grid_y = my - min_my

                if hit_ratio > occupied_threshold:
                    occupancy_grid[grid_y, grid_x] = 100
                elif hit_ratio < free_threshold:
                    occupancy_grid[grid_y, grid_x] = 0

    grid_info = {
        'resolution': resolution,
        'width': width,
        'height': height,
        'origin_x': min_mx * resolution,
        'origin_y': min_my * resolution,
        'min_mx': min_mx,
        'max_mx': max_mx,
        'min_my': min_my,
        'max_my': max_my
    }

    return occupancy_grid, grid_info


def plot_occupancy_grid(occupancy_grid, grid_info, trajectory=None,
                       title='Occupancy Grid Map', show_trajectory=True):
    fig, ax = plt.subplots(figsize=(14, 12))

    display_grid = np.zeros_like(occupancy_grid, dtype=np.float32)
    display_grid[occupancy_grid == 0] = -1
    display_grid[occupancy_grid == -1] = 50
    display_grid[occupancy_grid == 100] = 100

    cmap = plt.cm.colors.ListedColormap(['white', 'gray', 'black'])
    bounds = [-1.5, -0.5, 50.5, 100.5]
    norm = plt.cm.colors.BoundaryNorm(bounds, cmap.N)

    extent = [
        grid_info['origin_x'],
        grid_info['origin_x'] + grid_info['width'] * grid_info['resolution'],
        grid_info['origin_y'],
        grid_info['origin_y'] + grid_info['height'] * grid_info['resolution']
    ]

    im = ax.imshow(display_grid, cmap=cmap, norm=norm,
                   origin='lower', extent=extent, interpolation='nearest')

    if show_trajectory and trajectory is not None:
        traj_x = [p[0] for p in trajectory]
        traj_y = [p[1] for p in trajectory]
        ax.plot(traj_x, traj_y, 'b-', linewidth=2, label='Trajectory', alpha=0.7, zorder=5)
        ax.plot(traj_x[0], traj_y[0], 'go', markersize=12, label='Start',
                zorder=10, markeredgecolor='black', markeredgewidth=1.5)
        ax.plot(traj_x[-1], traj_y[-1], 'r^', markersize=10, label='End',
                zorder=10, markeredgecolor='black', markeredgewidth=1.5)
        ax.legend(loc='best', fontsize=11, framealpha=0.9)

    cbar = plt.colorbar(im, ax=ax, ticks=[-1, 25, 75], shrink=0.8)
    cbar.ax.set_yticklabels(['Free', 'Unknown', 'Occupied'])

    ax.set_xlabel('X [m]', fontsize=13, fontweight='bold')
    ax.set_ylabel('Y [m]', fontsize=13, fontweight='bold')
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.2, linestyle='--', color='blue', linewidth=0.5)

    return fig
