import numpy as np
import time
from scipy.spatial import KDTree


class PointToPointICP:
    def __init__(self, max_correspondence_distance=0.5, max_iteration=50,
                 tolerance=1e-6, outlier_rejection_percentile=80):
        self.max_correspondence_distance = max_correspondence_distance
        self.max_iteration = max_iteration
        self.tolerance = tolerance
        self.outlier_rejection_percentile = outlier_rejection_percentile
        self.name = "Point-to-Point"
        self.min_correspondences = 30

    def register_scan_to_map(self, scan_points_body, map_points_odom, init_x, init_y, init_theta):
        start_time = time.time()

        x = init_x
        y = init_y
        theta = init_theta

        scan_2d = scan_points_body[:, :2] if scan_points_body.shape[1] > 2 else scan_points_body
        map_2d = map_points_odom[:, :2] if map_points_odom.shape[1] > 2 else map_points_odom

        map_tree = KDTree(map_2d)
        prev_error = float('inf')
        iteration = 0

        for iteration in range(self.max_iteration):
            c = np.cos(theta)
            s = np.sin(theta)
            transformed = np.column_stack([
                scan_2d[:, 0] * c - scan_2d[:, 1] * s + x,
                scan_2d[:, 0] * s + scan_2d[:, 1] * c + y
            ])

            dists, indices = map_tree.query(transformed)
            valid = (dists < self.max_correspondence_distance) & np.isfinite(dists)
            n_valid = np.sum(valid)

            if n_valid < self.min_correspondences:
                runtime_ms = (time.time() - start_time) * 1000.0
                return {
                    'x': init_x,
                    'y': init_y,
                    'theta': init_theta,
                    'fitness': 0.0,
                    'inlier_rmse': 1.0,
                    'iterations': iteration + 1,
                    'runtime_ms': runtime_ms,
                    'success': False
                }

            if n_valid > 30:
                valid_dists = dists[valid]
                trim_thresh = np.percentile(valid_dists, self.outlier_rejection_percentile)
                trim_mask = np.zeros(len(dists), dtype=bool)
                trim_mask[valid] = dists[valid] <= trim_thresh
                valid = trim_mask
                n_valid = np.sum(valid)
                if n_valid < self.min_correspondences:
                    runtime_ms = (time.time() - start_time) * 1000.0
                    return {
                        'x': init_x,
                        'y': init_y,
                        'theta': init_theta,
                        'fitness': 0.0,
                        'inlier_rmse': 1.0,
                        'iterations': iteration + 1,
                        'runtime_ms': runtime_ms,
                        'success': False
                    }

            mean_error = np.mean(dists[valid])
            if np.isnan(mean_error) or abs(prev_error - mean_error) < self.tolerance:
                break
            prev_error = mean_error

            src_m = scan_2d[valid]
            tgt_m = map_2d[indices[valid]]

            src_c = np.mean(src_m, axis=0)
            tgt_c = np.mean(tgt_m, axis=0)

            src_centered = src_m - src_c
            tgt_centered = tgt_m - tgt_c

            W = tgt_centered.T @ src_centered

            try:
                U, _, Vt = np.linalg.svd(W)
                d = np.linalg.det(U @ Vt)
                R_opt = U @ np.diag([1.0, np.sign(d)]) @ Vt
                t_opt = tgt_c - R_opt @ src_c

                theta = np.arctan2(R_opt[1, 0], R_opt[0, 0])
                x = t_opt[0]
                y = t_opt[1]
            except np.linalg.LinAlgError:
                break

        c = np.cos(theta)
        s = np.sin(theta)
        final_transformed = np.column_stack([
            scan_2d[:, 0] * c - scan_2d[:, 1] * s + x,
            scan_2d[:, 0] * s + scan_2d[:, 1] * c + y
        ])

        final_dists, _ = map_tree.query(final_transformed)
        final_valid = (final_dists < self.max_correspondence_distance) & np.isfinite(final_dists)

        if np.sum(final_valid) > 0:
            inlier_rmse = np.sqrt(np.mean(final_dists[final_valid] ** 2))
        else:
            inlier_rmse = 1.0

        fitness = np.sum(final_valid) / len(scan_2d) if len(scan_2d) > 0 else 0.0

        runtime_ms = (time.time() - start_time) * 1000.0

        return {
            'x': x,
            'y': y,
            'theta': theta,
            'fitness': fitness,
            'inlier_rmse': inlier_rmse,
            'iterations': iteration + 1,
            'runtime_ms': runtime_ms,
            'success': True
        }

    def register(self, source, target, init_transform=np.eye(4)):
        start_time = time.time()

        source_points = np.asarray(source.points)
        target_points = np.asarray(target.points)

        if len(source_points) < 10 or len(target_points) < 10:
            return {
                'transformation': init_transform,
                'fitness': 0.0,
                'inlier_rmse': 1.0,
                'iterations': 0,
                'runtime_ms': 0.0
            }

        source_2d = source_points[:, :2]
        target_2d = target_points[:, :2]

        target_tree = KDTree(target_2d)

        transform = init_transform.copy()
        prev_error = float('inf')

        for iteration in range(self.max_iteration):
            R = transform[:2, :2]
            t = transform[:2, 3]

            transformed = (R @ source_2d.T).T + t

            dists, indices = target_tree.query(transformed)
            valid = (dists < self.max_correspondence_distance) & np.isfinite(dists)
            n_valid = np.sum(valid)

            if n_valid < self.min_correspondences:
                break

            if n_valid > 30:
                valid_dists = dists[valid]
                trim_thresh = np.percentile(valid_dists, self.outlier_rejection_percentile)
                valid = valid & (dists <= trim_thresh)
                n_valid = np.sum(valid)
                if n_valid < self.min_correspondences:
                    break

            mean_error = np.mean(dists[valid])
            if np.isnan(mean_error) or abs(prev_error - mean_error) < self.tolerance:
                break
            prev_error = mean_error

            src_m = source_2d[valid]
            tgt_m = target_2d[indices[valid]]

            src_c = np.mean(src_m, axis=0)
            tgt_c = np.mean(tgt_m, axis=0)

            src_centered = src_m - src_c
            tgt_centered = tgt_m - tgt_c

            W = tgt_centered.T @ src_centered

            try:
                U, _, Vt = np.linalg.svd(W)
                d = np.linalg.det(U @ Vt)
                R_opt = U @ np.diag([1.0, np.sign(d)]) @ Vt
                t_opt = tgt_c - R_opt @ src_c

                transform[:2, :2] = R_opt
                transform[:2, 3] = t_opt
            except np.linalg.LinAlgError:
                break

        R = transform[:2, :2]
        t = transform[:2, 3]
        final_transformed = (R @ source_2d.T).T + t

        final_dists, _ = target_tree.query(final_transformed)
        final_valid = (final_dists < self.max_correspondence_distance) & np.isfinite(final_dists)

        if np.sum(final_valid) > 0:
            inlier_rmse = np.sqrt(np.mean(final_dists[final_valid] ** 2))
        else:
            inlier_rmse = 1.0

        fitness = np.sum(final_valid) / len(source_2d) if len(source_2d) > 0 else 0.0

        runtime_ms = (time.time() - start_time) * 1000.0

        return {
            'transformation': transform,
            'fitness': fitness,
            'inlier_rmse': inlier_rmse,
            'iterations': iteration + 1,
            'runtime_ms': runtime_ms
        }
