"""Preprocessing notes.

The easy mistake here is to run outlier removal on the huge raw scan first.
That works in theory, but it makes Open3D do nearest-neighbour work on far more
points than needed. I downsample first because it turns the cloud from something
like millions of points into something closer to tens of thousands. Same idea,
much less work.

The other small gotcha is `remove_statistical_outlier`. It returns two things:
the cleaned cloud first, then the indices of the points it kept. I only need the
cloud here, so the code is `clean, _ = ...`. The underscore is just the normal
Python way of saying, "I know this value exists, but I am not using it here."
"""

import logging

import open3d as o3d

from src.config import Config

logger = logging.getLogger(__name__)


class Preprocessor:
    """Filters and density-reduces a raw point cloud.

    Two operations, run in sequence:
    1. Voxel downsampling - reduces point count while preserving shape.
    2. Statistical outlier removal - removes scanner ghost points.

    Responsibility: cleaning the raw cloud so normals and clustering are stable.
    """

    def __init__(self, config: Config) -> None:
        """
        Args:
            config: Pipeline configuration object. All parameters are read from here.
        """
        self.voxel_size = config.voxel_size
        self.nb_neighbors = config.sor_nb_neighbors
        self.std_ratio = config.sor_std_ratio

    def downsample(self, pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        """Voxel-grid downsample the cloud.

        Divides 3D space into cubes of side voxel_size. All points inside
        each cube are replaced with their centroid - one point per cube.

        Args:
            pcd: Input point cloud.

        Returns:
            A new PointCloud with at most one point per voxel cell.
        """
        before = len(pcd.points)
        downsampled = pcd.voxel_down_sample(self.voxel_size)
        after = len(downsampled.points)

        logger.info("Voxel downsample: %s -> %s points", f"{before:,}", f"{after:,}")
        return downsampled

    def remove_outliers(self, pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        """Statistical outlier removal.

        For every point, computes the mean distance to its nb_neighbors
        nearest neighbours. Points whose mean distance exceeds
        (global_mean + std_ratio * std_deviation) are removed.

        Args:
            pcd: Downsampled point cloud.

        Returns:
            A new PointCloud with noise points removed.
        """
        before = len(pcd.points)
        # Open3D returns (cleaned_cloud, kept_point_indices).
        # I am deliberately keeping only the cleaned cloud here.
        clean, _ = pcd.remove_statistical_outlier(
            nb_neighbors=self.nb_neighbors,
            std_ratio=self.std_ratio,
        )
        removed = before - len(clean.points)

        logger.info("SOR removed %d outlier points", removed)
        return clean

    def preprocess(self, pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        """Run the full preprocessing chain: downsample then remove outliers.

        This is the method the Pipeline calls. Keeping downsample and
        remove_outliers separate lets unit tests verify each step independently.

        Args:
            pcd: Raw loaded point cloud.

        Returns:
            Clean, downsampled point cloud ready for normal estimation.
        """
        # Order matters: doing SOR after downsampling makes the neighbour search
        # much cheaper, while still cleaning the points used by later stages.
        downsampled = self.downsample(pcd)
        return self.remove_outliers(downsampled)
