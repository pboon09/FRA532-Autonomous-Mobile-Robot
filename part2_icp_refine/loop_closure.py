import numpy as np
import math
from scipy.optimize import least_squares


class LoopClosure:
    def __init__(self, search_radius=2.5, temporal_threshold=30.0,
                 fitness_threshold=0.65, rmse_threshold=0.12):
        self.search_radius = search_radius
        self.temporal_threshold = temporal_threshold
        self.fitness_threshold = fitness_threshold
        self.rmse_threshold = rmse_threshold
        self.keyframes = []
        self.loop_constraints = []

    def add_keyframe(self, timestamp, pose, scan_pcd):
        self.keyframes.append({
            'timestamp': timestamp,
            'pose': pose.copy(),
            'scan': scan_pcd,
            'index': len(self.keyframes)
        })

    def detect_loops(self, icp_method):
        if len(self.keyframes) < 10:
            return []

        new_loops = []
        current_kf = self.keyframes[-1]

        for past_kf in self.keyframes[:-10]:
            time_diff = current_kf['timestamp'] - past_kf['timestamp']
            if time_diff < self.temporal_threshold:
                continue

            distance = self._compute_distance(current_kf['pose'], past_kf['pose'])

            if distance < self.search_radius:
                init_transform = np.eye(4)

                result = icp_method.register(
                    current_kf['scan'],
                    past_kf['scan'],
                    init_transform
                )

                if (result['fitness'] > self.fitness_threshold and
                    result['inlier_rmse'] < self.rmse_threshold):

                    relative_pose = self._transform_to_pose(result['transformation'])

                    information = np.eye(3)
                    information[0, 0] = 1.0 / (result['inlier_rmse'] ** 2 + 1e-6)
                    information[1, 1] = 1.0 / (result['inlier_rmse'] ** 2 + 1e-6)
                    information[2, 2] = 1.0 / (result['inlier_rmse'] ** 2 + 1e-6)

                    loop_constraint = {
                        'from_idx': past_kf['index'],
                        'to_idx': current_kf['index'],
                        'relative_pose': relative_pose,
                        'information_matrix': information,
                        'fitness': result['fitness'],
                        'rmse': result['inlier_rmse']
                    }

                    self.loop_constraints.append(loop_constraint)
                    new_loops.append(loop_constraint)

        return new_loops

    def optimize_pose_graph(self, initial_poses):
        if len(self.loop_constraints) == 0:
            return initial_poses

        num_poses = len(initial_poses)

        x0 = []
        for pose in initial_poses:
            x0.extend([pose[0], pose[1], pose[2]])

        x0 = np.array(x0)

        odometry_edges = []
        for i in range(num_poses - 1):
            rel_pose = self._compute_relative_pose(
                initial_poses[i + 1], initial_poses[i]
            )
            odometry_edges.append({
                'from_idx': i,
                'to_idx': i + 1,
                'relative_pose': rel_pose,
                'information_matrix': np.eye(3) * 1.0
            })

        def residuals(x):
            poses = x.reshape(-1, 3)
            res = []

            poses[0] = initial_poses[0]

            for edge in odometry_edges:
                i = edge['from_idx']
                j = edge['to_idx']

                if i >= num_poses or j >= num_poses:
                    continue

                predicted = self._compute_relative_pose(poses[j], poses[i])
                measured = edge['relative_pose']
                info = edge['information_matrix']

                error = np.array([
                    predicted[0] - measured[0],
                    predicted[1] - measured[1],
                    self._normalize_angle(predicted[2] - measured[2])
                ])

                weighted_error = np.sqrt(info) @ error
                res.extend(weighted_error.tolist())

            for constraint in self.loop_constraints:
                i = constraint['from_idx']
                j = constraint['to_idx']

                if i >= num_poses or j >= num_poses:
                    continue

                predicted = self._compute_relative_pose(poses[j], poses[i])
                measured = constraint['relative_pose']
                info = constraint['information_matrix']

                error = np.array([
                    predicted[0] - measured[0],
                    predicted[1] - measured[1],
                    self._normalize_angle(predicted[2] - measured[2])
                ])

                weighted_error = np.sqrt(info) @ error * 10.0
                res.extend(weighted_error.tolist())

            return np.array(res)

        result = least_squares(residuals, x0, method='lm', max_nfev=200, ftol=1e-6)

        optimized_poses = result.x.reshape(-1, 3)

        return optimized_poses.tolist()

    def _compute_distance(self, pose1, pose2):
        dx = pose1[0] - pose2[0]
        dy = pose1[1] - pose2[1]
        return math.sqrt(dx**2 + dy**2)

    def _transform_to_pose(self, transform):
        x = transform[0, 3]
        y = transform[1, 3]
        theta = math.atan2(transform[1, 0], transform[0, 0])
        return [x, y, theta]

    def _compute_relative_pose(self, pose_to, pose_from):
        dx = pose_to[0] - pose_from[0]
        dy = pose_to[1] - pose_from[1]

        cos_from = math.cos(-pose_from[2])
        sin_from = math.sin(-pose_from[2])

        rel_x = dx * cos_from - dy * sin_from
        rel_y = dx * sin_from + dy * cos_from
        rel_theta = self._normalize_angle(pose_to[2] - pose_from[2])

        return [rel_x, rel_y, rel_theta]

    def _normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def reset(self):
        self.keyframes = []
        self.loop_constraints = []

    def get_loop_info(self):
        return {
            'num_keyframes': len(self.keyframes),
            'num_loops': len(self.loop_constraints),
            'loops': self.loop_constraints
        }
