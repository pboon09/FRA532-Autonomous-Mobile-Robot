import numpy as np
import open3d as o3d
import math


class LidarProcessor:
    def __init__(self, voxel_size=0.05, translation_threshold=0.1, rotation_threshold=0.087):
        self.voxel_size = voxel_size
        self.translation_threshold = translation_threshold
        self.rotation_threshold = rotation_threshold
        self.last_keyframe_pose = None

    def scan_to_pointcloud(self, ranges, angles):
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        z = np.zeros_like(x)

        points = np.column_stack((x, y, z))

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        return pcd

    def downsample(self, pcd):
        if len(pcd.points) == 0:
            return pcd

        downsampled = pcd.voxel_down_sample(voxel_size=self.voxel_size)

        return downsampled

    def remove_outliers(self, pcd, nb_neighbors=20, std_ratio=2.0):
        if len(pcd.points) < nb_neighbors:
            return pcd

        filtered, _ = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)

        return filtered

    def estimate_normals(self, pcd, search_radius=0.1):
        if len(pcd.points) < 3:
            return pcd

        points = np.asarray(pcd.points)
        normals = np.zeros_like(points)

        for i in range(len(points)):
            distances = np.linalg.norm(points - points[i], axis=1)
            neighbors = np.argsort(distances)[1:min(11, len(points))]

            if len(neighbors) >= 2:
                neighbor_points = points[neighbors]

                dx = neighbor_points[:, 0] - points[i, 0]
                dy = neighbor_points[:, 1] - points[i, 1]

                tangent = np.array([np.mean(dx), np.mean(dy)])
                tangent_norm = np.linalg.norm(tangent)

                if tangent_norm > 1e-6:
                    tangent = tangent / tangent_norm
                    normal_2d = np.array([-tangent[1], tangent[0]])
                    normals[i] = [normal_2d[0], normal_2d[1], 0]
                else:
                    normals[i] = [0, 0, 1]
            else:
                normals[i] = [0, 0, 1]

        pcd.normals = o3d.utility.Vector3dVector(normals)

        return pcd

    def preprocess(self, ranges, angles, compute_normals=False):
        pcd = self.scan_to_pointcloud(ranges, angles)

        if len(pcd.points) == 0:
            return pcd

        pcd = self.downsample(pcd)

        pcd = self.remove_outliers(pcd)

        if compute_normals and len(pcd.points) >= 3:
            pcd = self.estimate_normals(pcd)

        return pcd

    def is_keyframe(self, current_pose):
        if self.last_keyframe_pose is None:
            self.last_keyframe_pose = current_pose
            return True

        dx = current_pose[0] - self.last_keyframe_pose[0]
        dy = current_pose[1] - self.last_keyframe_pose[1]
        translation = math.sqrt(dx**2 + dy**2)

        dtheta = abs(current_pose[2] - self.last_keyframe_pose[2])
        dtheta = math.atan2(math.sin(dtheta), math.cos(dtheta))
        rotation = abs(dtheta)

        if translation > self.translation_threshold or rotation > self.rotation_threshold:
            self.last_keyframe_pose = current_pose
            return True

        return False

    def reset_keyframe(self):
        self.last_keyframe_pose = None

    @staticmethod
    def pose_to_transform(x, y, theta):
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)

        transform = np.array([
            [cos_theta, -sin_theta, 0, x],
            [sin_theta,  cos_theta, 0, y],
            [0,          0,          1, 0],
            [0,          0,          0, 1]
        ])

        return transform

    @staticmethod
    def transform_to_pose(transform):
        x = transform[0, 3]
        y = transform[1, 3]
        theta = math.atan2(transform[1, 0], transform[0, 0])

        return x, y, theta

    @staticmethod
    def compose_transforms(T1, T2):
        return T1 @ T2
