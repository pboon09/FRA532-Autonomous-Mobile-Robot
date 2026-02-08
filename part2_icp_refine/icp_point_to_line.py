import time
import numpy as np
import open3d as o3d
from scipy.spatial import KDTree


class PointToLineICP:
    def __init__(self, max_iterations=50, tolerance=1e-6, max_correspondence_distance=1.0):
        self.name = "Point-to-Line"
        self.max_iterations = max_iterations
        self.tolerance = tolerance
        self.max_correspondence_distance = max_correspondence_distance

    def register(self, source, target, init_transform):
        start_time = time.perf_counter()

        source_points = np.asarray(source.points)
        target_points = np.asarray(target.points)

        if not hasattr(target, 'normals') or len(target.normals) == 0:
            target.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=30))

        target_normals = np.asarray(target.normals)

        T = init_transform.copy()

        tree = KDTree(target_points)

        prev_error = float('inf')

        for iteration in range(self.max_iterations):
            source_transformed = (T[:3, :3] @ source_points.T).T + T[:3, 3]

            distances, indices = tree.query(source_transformed)

            valid_mask = distances < self.max_correspondence_distance
            valid_source = source_transformed[valid_mask]
            valid_target_pts = target_points[indices[valid_mask]]
            valid_target_normals = target_normals[indices[valid_mask]]

            if len(valid_source) < 3:
                break

            A = []
            b = []

            for i in range(len(valid_source)):
                p_s = valid_source[i]
                p_t = valid_target_pts[i]
                n_t = valid_target_normals[i]

                diff = p_s - p_t
                error = np.dot(diff, n_t)

                n_x, n_y = n_t[0], n_t[1]
                p_x, p_y = p_s[0], p_s[1]

                row = [n_x, n_y, -p_x * n_y + p_y * n_x]
                A.append(row)
                b.append(-error)

            A = np.array(A)
            b = np.array(b)

            delta = np.linalg.lstsq(A, b, rcond=None)[0]

            dT = np.eye(4)
            dT[0, 3] = delta[0]
            dT[1, 3] = delta[1]
            cos_theta = np.cos(delta[2])
            sin_theta = np.sin(delta[2])
            dT[0, 0] = cos_theta
            dT[0, 1] = -sin_theta
            dT[1, 0] = sin_theta
            dT[1, 1] = cos_theta

            T = dT @ T

            current_error = np.mean(np.abs(b))

            if abs(prev_error - current_error) < self.tolerance:
                break

            prev_error = current_error

        runtime = (time.perf_counter() - start_time) * 1000.0

        source_final = (T[:3, :3] @ source_points.T).T + T[:3, 3]
        distances_final, _ = tree.query(source_final)
        valid_final = distances_final < self.max_correspondence_distance
        inlier_rmse = np.sqrt(np.mean(distances_final[valid_final] ** 2)) if np.sum(valid_final) > 0 else 1.0
        fitness = np.sum(valid_final) / len(source_points)

        return {
            'transformation': T,
            'fitness': fitness,
            'inlier_rmse': inlier_rmse,
            'iterations': iteration + 1,
            'runtime_ms': runtime
        }
