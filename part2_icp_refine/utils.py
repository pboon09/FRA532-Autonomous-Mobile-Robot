import json
import csv
import os
import numpy as np
import matplotlib.pyplot as plt


METHOD_COLORS = {
    'Point-to-Point': '#3498db',
    'Point-to-Plane': '#e74c3c',
    'Point-to-Line': '#2ecc71',
    'GICP': '#f39c12',
    'Wheel': '#95a5a6',
    'EKF': '#34495e'
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


def compute_ate(trajectory, ground_truth=None):
    if ground_truth is None or len(trajectory) != len(ground_truth):
        return 0.0

    errors = []
    for i in range(len(trajectory)):
        dx = trajectory[i][0] - ground_truth[i][0]
        dy = trajectory[i][1] - ground_truth[i][1]
        errors.append(np.sqrt(dx**2 + dy**2))

    return np.sqrt(np.mean(np.array(errors)**2))


def compute_convergence_rate(detailed_results):
    iterations = [r['iterations'] for r in detailed_results]
    return np.mean(iterations)


def compute_consistency_score(detailed_results):
    fitness = np.array([r['fitness'] for r in detailed_results])
    rmse = np.array([r['inlier_rmse'] for r in detailed_results])

    fitness_std = np.std(fitness)
    rmse_std = np.std(rmse)

    consistency = 1.0 / (1.0 + fitness_std + rmse_std)
    return consistency


def plot_icp_performance(method_results, all_detailed_results, title='ICP Performance Analysis'):
    fig, axes = plt.subplots(2, 3, figsize=(20, 12))

    methods = list(method_results.keys())
    colors = [get_method_color(m) for m in methods]

    ax1 = axes[0, 0]
    runtimes = [method_results[m]['avg_runtime_ms'] for m in methods]
    bars = ax1.bar(methods, runtimes, color=colors, edgecolor='black', linewidth=1.5)
    for bar, val in zip(bars, runtimes):
        ax1.text(bar.get_x() + bar.get_width()/2, bar.get_height() + max(runtimes)*0.03,
                f'{val:.1f}ms', ha='center', va='bottom', fontsize=11, fontweight='bold')
    ax1.set_ylabel('Runtime [ms]', fontsize=13, fontweight='bold')
    ax1.set_title('Speed', fontsize=14, fontweight='bold')
    ax1.tick_params(axis='x', rotation=45, labelsize=12)
    ax1.tick_params(axis='y', labelsize=11)
    ax1.grid(True, alpha=0.25, axis='y', linestyle='--')
    ax1.set_ylim(0, max(runtimes) * 1.2)

    ax2 = axes[0, 1]
    fitness = [method_results[m]['avg_fitness'] for m in methods]
    bars = ax2.bar(methods, fitness, color=colors, edgecolor='black', linewidth=1.5)
    for bar, val in zip(bars, fitness):
        ax2.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.015,
                f'{val:.3f}', ha='center', va='bottom', fontsize=11, fontweight='bold')
    ax2.set_ylabel('Fitness Score', fontsize=13, fontweight='bold')
    ax2.set_title('Accuracy', fontsize=14, fontweight='bold')
    ax2.tick_params(axis='x', rotation=45, labelsize=12)
    ax2.tick_params(axis='y', labelsize=11)
    ax2.grid(True, alpha=0.25, axis='y', linestyle='--')
    ax2.set_ylim(min(fitness) * 0.95, 1.02)

    ax3 = axes[0, 2]
    rmse = [method_results[m]['avg_rmse'] for m in methods]
    bars = ax3.bar(methods, rmse, color=colors, edgecolor='black', linewidth=1.5)
    for bar, val in zip(bars, rmse):
        ax3.text(bar.get_x() + bar.get_width()/2, bar.get_height() + max(rmse)*0.03,
                f'{val:.3f}m', ha='center', va='bottom', fontsize=11, fontweight='bold')
    ax3.set_ylabel('RMSE [m]', fontsize=13, fontweight='bold')
    ax3.set_title('Precision', fontsize=14, fontweight='bold')
    ax3.tick_params(axis='x', rotation=45, labelsize=12)
    ax3.tick_params(axis='y', labelsize=11)
    ax3.grid(True, alpha=0.25, axis='y', linestyle='--')
    ax3.set_ylim(0, max(rmse) * 1.2)

    ax4 = axes[1, 0]
    for method in methods:
        fitness_vals = [r['fitness'] for r in all_detailed_results[method]]
        color = get_method_color(method)
        ax4.plot(fitness_vals, label=method, color=color, alpha=0.8, linewidth=2)
    ax4.set_xlabel('Frame', fontsize=13, fontweight='bold')
    ax4.set_ylabel('Fitness', fontsize=13, fontweight='bold')
    ax4.set_title('Fitness Over Time', fontsize=14, fontweight='bold')
    ax4.legend(fontsize=10, loc='best', framealpha=0.95)
    ax4.tick_params(labelsize=11)
    ax4.grid(True, alpha=0.25, linestyle='--')

    ax5 = axes[1, 1]
    for method in methods:
        rmse_vals = [r['inlier_rmse'] for r in all_detailed_results[method]]
        color = get_method_color(method)
        ax5.plot(rmse_vals, label=method, color=color, alpha=0.8, linewidth=2)
    ax5.set_xlabel('Frame', fontsize=13, fontweight='bold')
    ax5.set_ylabel('RMSE [m]', fontsize=13, fontweight='bold')
    ax5.set_title('RMSE Over Time', fontsize=14, fontweight='bold')
    ax5.legend(fontsize=10, loc='best', framealpha=0.95)
    ax5.tick_params(labelsize=11)
    ax5.grid(True, alpha=0.25, linestyle='--')

    ax6 = axes[1, 2]
    for method in methods:
        runtime_vals = [r['runtime_ms'] for r in all_detailed_results[method]]
        color = get_method_color(method)
        ax6.plot(runtime_vals, label=method, color=color, alpha=0.8, linewidth=2)
    ax6.set_xlabel('Frame', fontsize=13, fontweight='bold')
    ax6.set_ylabel('Runtime [ms]', fontsize=13, fontweight='bold')
    ax6.set_title('Runtime Over Time', fontsize=14, fontweight='bold')
    ax6.legend(fontsize=10, loc='best', framealpha=0.95)
    ax6.tick_params(labelsize=11)
    ax6.grid(True, alpha=0.25, linestyle='--')

    fig.suptitle(title, fontsize=16, fontweight='bold')
    plt.tight_layout()

    return fig


def plot_loop_closure_comparison(before_poses, after_poses, loops, scan_data=None, title='Loop Closure Effect', downsample=5):
    fig, axes = plt.subplots(1, 2, figsize=(18, 8))

    ax1 = axes[0]
    if scan_data is not None:
        all_points_x = []
        all_points_y = []
        for i in range(0, len(before_poses), downsample):
            if i >= len(scan_data):
                break
            pose = before_poses[i]
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
            ax1.scatter(all_points_x, all_points_y, c='gray', s=0.5, alpha=0.4)

    x_before = [p[0] for p in before_poses]
    y_before = [p[1] for p in before_poses]
    ax1.plot(x_before, y_before, 'b-', label='Trajectory', linewidth=2)
    ax1.plot(x_before[0], y_before[0], 'go', markersize=12, label='Start', markeredgecolor='black', markeredgewidth=1.5)
    ax1.plot(x_before[-1], y_before[-1], 'r^', markersize=10, label='End', markeredgecolor='black', markeredgewidth=1.5)
    ax1.set_xlabel('X [m]', fontsize=12, fontweight='bold')
    ax1.set_ylabel('Y [m]', fontsize=12, fontweight='bold')
    ax1.set_title('Before', fontsize=13, fontweight='bold')
    ax1.legend(fontsize=10)
    ax1.grid(True, alpha=0.3, linestyle='--')
    ax1.set_aspect('equal')

    ax2 = axes[1]
    if scan_data is not None:
        all_points_x = []
        all_points_y = []
        for i in range(0, len(after_poses), downsample):
            if i >= len(scan_data):
                break
            pose = after_poses[i]
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
            ax2.scatter(all_points_x, all_points_y, c='gray', s=0.5, alpha=0.4)

    x_after = [p[0] for p in after_poses]
    y_after = [p[1] for p in after_poses]
    ax2.plot(x_after, y_after, 'b-', label='Trajectory', linewidth=2)
    ax2.plot(x_after[0], y_after[0], 'go', markersize=12, label='Start', markeredgecolor='black', markeredgewidth=1.5)
    ax2.plot(x_after[-1], y_after[-1], 'r^', markersize=10, label='End', markeredgecolor='black', markeredgewidth=1.5)

    for loop in loops:
        from_idx = loop['from_idx']
        to_idx = loop['to_idx']
        if from_idx < len(after_poses) and to_idx < len(after_poses):
            ax2.plot([x_after[from_idx], x_after[to_idx]],
                    [y_after[from_idx], y_after[to_idx]],
                    'g--', linewidth=1.5, alpha=0.7, label='Loop' if loop == loops[0] else '')

    ax2.set_xlabel('X [m]', fontsize=12, fontweight='bold')
    ax2.set_ylabel('Y [m]', fontsize=12, fontweight='bold')
    ax2.set_title('After', fontsize=13, fontweight='bold')
    ax2.legend(fontsize=10)
    ax2.grid(True, alpha=0.3, linestyle='--')
    ax2.set_aspect('equal')

    fig.suptitle(title, fontsize=14, fontweight='bold')
    plt.tight_layout()

    return fig


def plot_fitness_time_series(times, fitness_data, title='Fitness Score Time Series'):
    fig, ax = plt.subplots(figsize=(12, 6))

    for method, fitness_values in fitness_data.items():
        color = get_method_color(method)
        ax.plot(times, fitness_values, color=color, label=method, linewidth=2, alpha=0.85)

    ax.set_xlabel('Time [s]', fontsize=12, fontweight='bold')
    ax.set_ylabel('Fitness Score', fontsize=12, fontweight='bold')
    ax.set_title(title, fontsize=13, fontweight='bold')
    ax.legend(loc='best', fontsize=10)
    ax.grid(True, alpha=0.3, linestyle='--')

    return fig


def plot_loop_closure_analysis(all_results, title='Loop Closure Impact Analysis'):
    """
    Compare BEFORE vs AFTER loop closure performance for each sequence
    all_results: dict with sequence names as keys, each containing:
        - 'best_method': name of best ICP method
        - 'num_loops': number of loops detected
        - 'fitness_before': avg fitness before loop closure
        - 'fitness_after': avg fitness after loop closure (same as before for now)
        - 'rmse_before': avg RMSE before loop closure
        - 'rmse_after': avg RMSE after loop closure (same as before for now)
    """
    sequences = list(all_results.keys())
    seq_with_loops = [s for s in sequences if all_results[s].get('num_loops', 0) > 0]

    # Determine layout: if we have sequences with loops, use 2x2, otherwise 1x3
    if seq_with_loops:
        fig, axes = plt.subplots(2, 2, figsize=(16, 12))
        axes = axes.flatten()
    else:
        fig, axes = plt.subplots(1, 3, figsize=(18, 6))

    # Plot 1: Number of loops per sequence
    ax1 = axes[0]
    num_loops = [all_results[s].get('num_loops', 0) for s in sequences]
    colors_loops = ['#2ecc71' if n > 0 else '#95a5a6' for n in num_loops]
    bars = ax1.bar(sequences, num_loops, color=colors_loops, edgecolor='black', linewidth=1.5)
    for bar, val in zip(bars, num_loops):
        ax1.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.5,
                f'{int(val)}', ha='center', va='bottom', fontsize=12, fontweight='bold')
    ax1.set_ylabel('Number of Loops', fontsize=13, fontweight='bold')
    ax1.set_title('Loop Closures Detected', fontsize=14, fontweight='bold')
    ax1.tick_params(axis='x', rotation=0, labelsize=12)
    ax1.tick_params(axis='y', labelsize=11)
    ax1.grid(True, alpha=0.25, axis='y', linestyle='--')
    ax1.set_ylim(0, max(num_loops + [1]) * 1.2)

    # Plot 2: Best method per sequence
    ax2 = axes[1]
    best_methods = [all_results[s].get('best_method', 'Unknown') for s in sequences]
    colors_methods = [get_method_color(m) for m in best_methods]
    y_pos = np.arange(len(sequences))

    bars = ax2.barh(y_pos, [1]*len(sequences), color=colors_methods, edgecolor='black', linewidth=1.5)
    ax2.set_yticks(y_pos)
    ax2.set_yticklabels(sequences, fontsize=12)
    ax2.set_xlim(0, 1)
    ax2.set_xticks([])
    ax2.set_title('Best ICP Method per Sequence', fontsize=14, fontweight='bold')

    for i, (seq, method) in enumerate(zip(sequences, best_methods)):
        loops = all_results[seq].get('num_loops', 0)
        loop_text = f" ({loops} loops)" if loops > 0 else " (no loops)"
        ax2.text(0.5, i, f'{method}{loop_text}',
                ha='center', va='center', fontsize=11, fontweight='bold', color='white')

    if seq_with_loops:
        # Plot 3: Fitness comparison (before vs after) for sequences WITH loops
        ax3 = axes[2]
        x_pos = np.arange(len(seq_with_loops))
        width = 0.35
        fitness_before = [all_results[s].get('fitness_before', 0) for s in seq_with_loops]
        fitness_after = [all_results[s].get('fitness_after', 0) for s in seq_with_loops]

        bars1 = ax3.bar(x_pos - width/2, fitness_before, width, label='Before',
                       color='#e74c3c', edgecolor='black', linewidth=1.5)
        bars2 = ax3.bar(x_pos + width/2, fitness_after, width, label='After',
                       color='#2ecc71', edgecolor='black', linewidth=1.5)

        # Add values on bars
        for i, (before, after) in enumerate(zip(fitness_before, fitness_after)):
            ax3.text(x_pos[i] - width/2, before + 0.01, f'{before:.3f}',
                    ha='center', va='bottom', fontsize=10, fontweight='bold')
            ax3.text(x_pos[i] + width/2, after + 0.01, f'{after:.3f}',
                    ha='center', va='bottom', fontsize=10, fontweight='bold')

        ax3.set_ylabel('Fitness Score', fontsize=13, fontweight='bold')
        ax3.set_title('Fitness: Before vs After Loop Closure', fontsize=14, fontweight='bold')
        ax3.set_xticks(x_pos)
        ax3.set_xticklabels(seq_with_loops, fontsize=12)
        ax3.legend(fontsize=11, loc='lower right')
        ax3.tick_params(labelsize=11)
        ax3.grid(True, alpha=0.25, axis='y', linestyle='--')
        ax3.set_ylim(min(fitness_before + fitness_after) * 0.95, 1.02)

        # Plot 4: RMSE comparison (before vs after) for sequences WITH loops
        ax4 = axes[3]
        rmse_before = [all_results[s].get('rmse_before', 0) for s in seq_with_loops]
        rmse_after = [all_results[s].get('rmse_after', 0) for s in seq_with_loops]

        bars1 = ax4.bar(x_pos - width/2, rmse_before, width, label='Before',
                       color='#e74c3c', edgecolor='black', linewidth=1.5)
        bars2 = ax4.bar(x_pos + width/2, rmse_after, width, label='After',
                       color='#2ecc71', edgecolor='black', linewidth=1.5)

        # Add values on bars
        for i, (before, after) in enumerate(zip(rmse_before, rmse_after)):
            ax4.text(x_pos[i] - width/2, before + max(rmse_before)*0.02, f'{before:.3f}',
                    ha='center', va='bottom', fontsize=10, fontweight='bold')
            ax4.text(x_pos[i] + width/2, after + max(rmse_after)*0.02, f'{after:.3f}',
                    ha='center', va='bottom', fontsize=10, fontweight='bold')

        ax4.set_ylabel('RMSE [m]', fontsize=13, fontweight='bold')
        ax4.set_title('RMSE: Before vs After Loop Closure', fontsize=14, fontweight='bold')
        ax4.set_xticks(x_pos)
        ax4.set_xticklabels(seq_with_loops, fontsize=12)
        ax4.legend(fontsize=11, loc='upper right')
        ax4.tick_params(labelsize=11)
        ax4.grid(True, alpha=0.25, axis='y', linestyle='--')
        ax4.set_ylim(0, max(rmse_before + rmse_after) * 1.15)
    else:
        # Plot 3: Summary statistics for all sequences (no loop closures)
        ax3 = axes[2]
        fitness_vals = [all_results[s].get('fitness_before', 0) for s in sequences]
        bars = ax3.bar(sequences, fitness_vals, color='#95a5a6', edgecolor='black', linewidth=1.5)
        for bar, val in zip(bars, fitness_vals):
            ax3.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.01,
                    f'{val:.3f}', ha='center', va='bottom', fontsize=10, fontweight='bold')
        ax3.set_ylabel('Fitness Score', fontsize=13, fontweight='bold')
        ax3.set_title('ICP Fitness (No Loop Closures Detected)', fontsize=14, fontweight='bold')
        ax3.tick_params(axis='x', rotation=0, labelsize=12)
        ax3.grid(True, alpha=0.25, axis='y', linestyle='--')
        ax3.set_ylim(min(fitness_vals) * 0.95, 1.02)

    fig.suptitle(title, fontsize=16, fontweight='bold')
    plt.tight_layout()

    return fig


def plot_runtime_boxplot(runtime_data, title='Runtime Distribution'):
    fig, ax = plt.subplots(figsize=(10, 6))

    methods = list(runtime_data.keys())
    data = [runtime_data[m] for m in methods]
    colors = [get_method_color(m) for m in methods]

    bp = ax.boxplot(data, labels=methods, patch_artist=True)

    for patch, color in zip(bp['boxes'], colors):
        patch.set_facecolor(color)
        patch.set_edgecolor('black')
        patch.set_linewidth(1.5)

    ax.set_ylabel('Runtime [ms]', fontsize=12, fontweight='bold')
    ax.set_title(title, fontsize=13, fontweight='bold')
    ax.grid(True, alpha=0.3, axis='y', linestyle='--')
    ax.tick_params(axis='x', rotation=45, labelsize=11)
    ax.tick_params(axis='y', labelsize=11)

    return fig


def compute_trajectory_drift(trajectory):
    if len(trajectory) < 2:
        return 0.0

    total_distance = 0.0
    for i in range(1, len(trajectory)):
        dx = trajectory[i][0] - trajectory[i-1][0]
        dy = trajectory[i][1] - trajectory[i-1][1]
        total_distance += np.sqrt(dx**2 + dy**2)

    return total_distance


def compute_final_position_error(traj1, traj2):
    if len(traj1) == 0 or len(traj2) == 0:
        return float('inf')

    dx = traj1[-1][0] - traj2[-1][0]
    dy = traj1[-1][1] - traj2[-1][1]

    return np.sqrt(dx**2 + dy**2)


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


def rank_methods(method_results, weights={'accuracy': 0.35, 'robustness': 0.25, 'speed': 0.20, 'consistency': 0.15, 'convergence': 0.05}):
    methods = list(method_results.keys())

    fitness_scores = [method_results[m]['avg_fitness'] for m in methods]
    rmse_scores = [method_results[m]['avg_rmse'] for m in methods]
    runtime_scores = [method_results[m]['avg_runtime_ms'] for m in methods]
    consistency_scores = [method_results[m].get('consistency_score', 0.5) for m in methods]
    convergence_scores = [method_results[m].get('avg_iterations', 50) for m in methods]

    max_fitness = max(fitness_scores) if max(fitness_scores) > 0 else 1
    min_rmse = min(rmse_scores) if min(rmse_scores) > 0 else 1
    min_runtime = min(runtime_scores) if min(runtime_scores) > 0 else 1
    max_consistency = max(consistency_scores) if max(consistency_scores) > 0 else 1
    min_convergence = min(convergence_scores) if min(convergence_scores) > 0 else 1

    rankings = {}
    for i, method in enumerate(methods):
        accuracy_score = (fitness_scores[i] / max_fitness) * (min_rmse / rmse_scores[i])
        robustness_score = fitness_scores[i] / max_fitness
        speed_score = min_runtime / runtime_scores[i]
        consistency_score = consistency_scores[i] / max_consistency
        convergence_score = min_convergence / convergence_scores[i]

        total_score = (accuracy_score * weights['accuracy'] +
                      robustness_score * weights['robustness'] +
                      speed_score * weights['speed'] +
                      consistency_score * weights['consistency'] +
                      convergence_score * weights['convergence'])

        rankings[method] = {
            'total_score': total_score,
            'accuracy_score': accuracy_score,
            'robustness_score': robustness_score,
            'speed_score': speed_score,
            'consistency_score': consistency_score,
            'convergence_score': convergence_score
        }

    sorted_methods = sorted(rankings.items(), key=lambda x: x[1]['total_score'], reverse=True)

    return sorted_methods, rankings
