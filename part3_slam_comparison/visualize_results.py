#!/usr/bin/env python3

import numpy as np
import argparse
from pathlib import Path
from utils import save_figure, plot_all_trajectories, plot_time_series
import matplotlib.pyplot as plt
import json
from process_bag import BagProcessor


def load_data(data_dir, sequence_name):
    seq_dir = data_dir / sequence_name
    npz_file = seq_dir / 'trajectories.npz'
    if not npz_file.exists():
        raise FileNotFoundError(f'Trajectory file not found: {npz_file}')

    data = np.load(npz_file, allow_pickle=True)

    trajectories = {}
    trajectories_ts = {}

    if 'wheel' in data and len(data['wheel']) > 0:
        trajectories['Wheel'] = data['wheel']
        trajectories_ts['Wheel'] = data['wheel_ts']

    if 'ekf' in data and len(data['ekf']) > 0:
        trajectories['EKF'] = data['ekf']
        trajectories_ts['EKF'] = data['ekf_ts']

    if 'icp' in data and len(data['icp']) > 0:
        trajectories['ICP'] = data['icp']
        trajectories_ts['ICP'] = data['icp_ts']

    if 'slam' in data and len(data['slam']) > 0:
        trajectories['SLAM'] = data['slam']
        trajectories_ts['SLAM'] = data['slam_ts']

    return trajectories, trajectories_ts


def process_bag_file(script_dir, data_dir, sequence_name):
    bag_dir = script_dir / 'bags' / sequence_name

    if not bag_dir.exists():
        print(f'Bag directory not found: {bag_dir}')
        return False

    print(f'Processing bag: {bag_dir}')
    processor = BagProcessor(str(bag_dir))
    processor.process()

    print(f'Saving to NPZ...')
    summary = processor.save_to_npz(data_dir, sequence_name)

    print(f'\nSummary:')
    for method, info in summary['methods'].items():
        print(f'  {method}: {info["num_poses"]} poses, drift: {info["drift_from_start_m"]:.3f}m, length: {info["trajectory_length_m"]:.3f}m')

    return True


def load_map(data_dir, sequence_name, map_type):
    seq_dir = data_dir / sequence_name
    npz_file = seq_dir / f'{map_type}_map.npz'
    if not npz_file.exists():
        return None, None

    data = np.load(npz_file, allow_pickle=True)
    return data['map'], data['metadata'].item()


def plot_map(map_data, metadata, trajectory, title):
    fig, ax = plt.subplots(figsize=(14, 12))

    display_map = np.zeros_like(map_data, dtype=np.float32)
    display_map[map_data == 0] = -1
    display_map[map_data == -1] = 50
    display_map[map_data == 100] = 100

    cmap = plt.cm.colors.ListedColormap(['white', 'gray', 'black'])
    bounds = [-1.5, -0.5, 50.5, 100.5]
    norm = plt.cm.colors.BoundaryNorm(bounds, cmap.N)

    extent = [
        metadata['origin_x'],
        metadata['origin_x'] + metadata['width'] * metadata['resolution'],
        metadata['origin_y'],
        metadata['origin_y'] + metadata['height'] * metadata['resolution']
    ]

    im = ax.imshow(display_map, cmap=cmap, norm=norm,
                   origin='lower', extent=extent, interpolation='nearest')

    if trajectory is not None and len(trajectory) > 0:
        ax.plot(trajectory[:, 0], trajectory[:, 1], 'b-', linewidth=2, label='Trajectory', alpha=0.7, zorder=5)
        ax.plot(trajectory[0, 0], trajectory[0, 1], 'go', markersize=12, label='Start',
                zorder=10, markeredgecolor='black', markeredgewidth=1.5)
        ax.plot(trajectory[-1, 0], trajectory[-1, 1], 'r^', markersize=10, label='End',
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


def main():
    parser = argparse.ArgumentParser(description='Visualize SLAM comparison results')
    parser.add_argument('--sequence', type=str, required=True, help='Sequence name (e.g., seq00)')
    args = parser.parse_args()

    script_dir = Path(__file__).parent
    data_dir = script_dir / 'data'
    output_dir = script_dir / 'figures' / args.sequence
    output_dir.mkdir(parents=True, exist_ok=True)

    process_bag_file(script_dir, data_dir, args.sequence)

    print(f'\nLoading data for sequence: {args.sequence}')
    trajectories, trajectories_ts = load_data(data_dir, args.sequence)

    summary_file = data_dir / args.sequence / 'summary.json'
    if summary_file.exists():
        with open(summary_file, 'r') as f:
            summary = json.load(f)
        print(f'\nTrajectory Summary:')
        for method, info in summary['methods'].items():
            print(f'  {method}: {info["num_poses"]} poses, drift: {info["drift_from_start_m"]:.3f}m, length: {info["trajectory_length_m"]:.3f}m')

        if 'comparisons' in summary and len(summary['comparisons']) > 0:
            print(f'\nTrajectory Errors (vs SLAM):')
            for comp_name, metrics in summary['comparisons'].items():
                method = comp_name.replace('_vs_SLAM', '')
                print(f'  {method}: mean={metrics["mean_error_m"]:.3f}m, max={metrics["max_error_m"]:.3f}m')

        if 'slam_map_metrics' in summary or 'icp_map_metrics' in summary:
            print(f'\nMap Metrics:')
            if 'icp_map_metrics' in summary:
                m = summary['icp_map_metrics']
                print(f'  ICP:  known={m["known_percent"]:.1f}%, occupied={m["occupied_percent"]:.1f}%')
            if 'slam_map_metrics' in summary:
                m = summary['slam_map_metrics']
                print(f'  SLAM: known={m["known_percent"]:.1f}%, occupied={m["occupied_percent"]:.1f}%')

    if len(trajectories) > 0:
        print(f'\nGenerating aligned trajectory plot...')
        fig = plot_all_trajectories(trajectories, f'{args.sequence}: Trajectory Comparison', align_to_start=True)
        save_figure(fig, output_dir / 'all_trajectories.png')
        print(f'Saved to {output_dir}/all_trajectories.png')

    if len(trajectories_ts) > 0:
        print(f'Generating time series plot...')
        fig = plot_time_series(trajectories_ts, f'{args.sequence}: Time Series Comparison')
        save_figure(fig, output_dir / 'time_series.png')
        print(f'Saved to {output_dir}/time_series.png')

    slam_map, slam_metadata = load_map(data_dir, args.sequence, 'slam')
    if slam_map is not None:
        print(f'Generating SLAM map plot...')
        slam_traj = trajectories.get('SLAM', None)
        fig = plot_map(slam_map, slam_metadata, slam_traj, f'{args.sequence}: SLAM Occupancy Grid Map')
        save_figure(fig, output_dir / 'slam_map.png')
        print(f'Saved to {output_dir}/slam_map.png')

    icp_map, icp_metadata = load_map(data_dir, args.sequence, 'icp')

    if icp_map is not None and slam_map is not None:
        print(f'\nMap Quality Analysis:')
        print(f'ICP map typically has more detail because:')
        print(f'  - ICP mapper processes ALL laser scans in real-time')
        print(f'  - SLAM toolbox skips scans (minimum_travel_distance: 0.2m)')
        print(f'  - ICP focuses on dense mapping, SLAM focuses on trajectory accuracy')

    if slam_map is not None and icp_map is not None:
        print(f'Generating map comparison...')
        fig, axes = plt.subplots(1, 2, figsize=(24, 12))

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

        if icp_bounds and slam_bounds:
            common_extent = [
                min(icp_bounds[0], slam_bounds[0]),
                max(icp_bounds[1], slam_bounds[1]),
                min(icp_bounds[2], slam_bounds[2]),
                max(icp_bounds[3], slam_bounds[3])
            ]
        else:
            common_extent = [
                min(icp_metadata['origin_x'], slam_metadata['origin_x']),
                max(icp_metadata['origin_x'] + icp_metadata['width'] * icp_metadata['resolution'],
                    slam_metadata['origin_x'] + slam_metadata['width'] * slam_metadata['resolution']),
                min(icp_metadata['origin_y'], slam_metadata['origin_y']),
                max(icp_metadata['origin_y'] + icp_metadata['height'] * icp_metadata['resolution'],
                    slam_metadata['origin_y'] + slam_metadata['height'] * slam_metadata['resolution'])
            ]

        for ax, map_data, metadata, title in zip(
            axes,
            [icp_map, slam_map],
            [icp_metadata, slam_metadata],
            ['ICP Map', 'SLAM Map']
        ):
            if title == 'SLAM Map':
                from matplotlib.patches import Rectangle
                grey_bg = Rectangle((common_extent[0], common_extent[2]),
                                  common_extent[1] - common_extent[0],
                                  common_extent[3] - common_extent[2],
                                  facecolor='gray', edgecolor='none', zorder=0)
                ax.add_patch(grey_bg)

            display_map = np.zeros_like(map_data, dtype=np.float32)
            display_map[map_data == 0] = -1
            display_map[map_data == -1] = 50
            display_map[map_data == 100] = 100

            cmap = plt.cm.colors.ListedColormap(['white', 'gray', 'black'])
            bounds = [-1.5, -0.5, 50.5, 100.5]
            norm = plt.cm.colors.BoundaryNorm(bounds, cmap.N)

            extent = [
                metadata['origin_x'],
                metadata['origin_x'] + metadata['width'] * metadata['resolution'],
                metadata['origin_y'],
                metadata['origin_y'] + metadata['height'] * metadata['resolution']
            ]

            im = ax.imshow(display_map, cmap=cmap, norm=norm,
                           origin='lower', extent=extent, interpolation='nearest', zorder=1)

            ax.set_xlim(common_extent[0], common_extent[1])
            ax.set_ylim(common_extent[2], common_extent[3])
            ax.set_xlabel('X [m]', fontsize=13, fontweight='bold')
            ax.set_ylabel('Y [m]', fontsize=13, fontweight='bold')
            ax.set_title(title, fontsize=14, fontweight='bold')
            ax.set_aspect('equal')

            ax.xaxis.set_major_locator(plt.MultipleLocator(1.0))
            ax.yaxis.set_major_locator(plt.MultipleLocator(1.0))
            ax.grid(True, alpha=0.2, linestyle='--', color='blue', linewidth=0.5)

        plt.suptitle(f'{args.sequence}: Map Comparison (ICP vs SLAM)', fontsize=16, fontweight='bold')
        plt.tight_layout()

        save_figure(fig, output_dir / 'map_comparison.png')
        print(f'Saved to {output_dir}/map_comparison.png')

    print('\nDone!')


if __name__ == '__main__':
    main()
