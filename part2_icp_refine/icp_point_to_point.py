import numpy as np
import open3d as o3d
import time


class PointToPointICP:
    def __init__(self, max_correspondence_distance=0.5, max_iteration=30,
                 relative_fitness=1e-6, relative_rmse=1e-6):
        self.max_correspondence_distance = max_correspondence_distance
        self.max_iteration = max_iteration
        self.relative_fitness = relative_fitness
        self.relative_rmse = relative_rmse
        self.name = "Point-to-Point"

    def register(self, source, target, init_transform=np.eye(4)):
        if len(source.points) < 3 or len(target.points) < 3:
            return {
                'transformation': init_transform,
                'fitness': 0.0,
                'inlier_rmse': float('inf'),
                'iterations': 0,
                'runtime_ms': 0.0
            }

        start_time = time.time()

        result = o3d.pipelines.registration.registration_icp(
            source, target,
            self.max_correspondence_distance,
            init_transform,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                relative_fitness=self.relative_fitness,
                relative_rmse=self.relative_rmse,
                max_iteration=self.max_iteration
            )
        )

        runtime_ms = (time.time() - start_time) * 1000.0

        return {
            'transformation': result.transformation,
            'fitness': result.fitness,
            'inlier_rmse': result.inlier_rmse,
            'iterations': self.max_iteration,
            'runtime_ms': runtime_ms
        }
