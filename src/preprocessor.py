"""Preprocessing: make the raw cloud smaller and less noisy before analysis.

Main thing I had to understand here:

- Downsample first. The Eagle scan starts huge, so doing nearest-neighbour
  outlier removal immediately would make Open3D do a lot of expensive search
  work on points I am going to throw away anyway.
- Then run SOR. At that point the cloud is already much smaller, so the same
  cleanup step is way cheaper.
- `clean, _ = ...` is not magic. Open3D returns two values: the cleaned cloud and
  the inlier indices. I only need the cloud here, so `_` is just Python shorthand
  for "yep, I know this exists, but I am intentionally ignoring it."
"""

import logging

import open3d as o3d

from src.config import Config

logger = logging.getLogger(__name__)


class Preprocessor:
    """Filters and density-reduces a raw point cloud before analysis.

    Two operations run in this specific order:
    1. Voxel downsampling first - cuts the raw cloud down to a smaller one.
    2. Statistical Outlier Removal second - removes scanner ghost points.

    Why this order matters:
    SOR computes nearest-neighbour distances. That search is expensive, so I want
    it running on the reduced cloud, not the full raw scan.

    Design choice:
    Each operation is a separate method. That way tests can check downsampling
    and outlier removal independently instead of blaming the whole pipeline.
    """

    def __init__(self, config: Config) -> None:
        # Private-ish attrs: underscore means "internal, pls don't poke directly".
        self._voxel_size = config.voxel_size
        self._nb_neighbors = config.sor_nb_neighbors
        self._std_ratio = config.sor_std_ratio

    def downsample(self, pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        """Voxel-grid downsample: one centroid per occupied voxel cube.

        Args:
            pcd: Input cloud.

        Returns:
            New PointCloud with the same or fewer points than the input.
        """
        before = len(pcd.points)
        result = pcd.voxel_down_sample(self._voxel_size)
        after = len(result.points)

        reduction_pct = 0.0
        if before:
            reduction_pct = (1 - after / before) * 100

        logger.info(
            "Voxel downsample (%.3fm): %s -> %s pts (%.1f%% reduction)",
            self._voxel_size,
            f"{before:,}",
            f"{after:,}",
            reduction_pct,
        )
        return result

    def remove_outliers(self, pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        """Statistical Outlier Removal: trims floating ghost points.

        For each point P:
        1. Find the nb_neighbors nearest neighbours.
        2. Compute mean distance from P to those neighbours.
        3. Remove P if that distance is too far from the global pattern.

        std_ratio=2.0 is the 2-sigma rule: it aims at the outer edge of the
        distance distribution instead of attacking normal surface points.

        Args:
            pcd: Downsampled cloud.

        Returns:
            New PointCloud with outlier points removed.
        """
        before = len(pcd.points)

        # Returns (clean_pcd, inlier_index_list).
        # `_` discards the index list because this stage only needs clean_pcd.
        clean, _ = pcd.remove_statistical_outlier(
            nb_neighbors=self._nb_neighbors,
            std_ratio=self._std_ratio,
        )

        removed = before - len(clean.points)
        logger.info(
            "SOR (nb_neighbors=%d, std_ratio=%.1f): removed %d ghost points",
            self._nb_neighbors,
            self._std_ratio,
            removed,
        )
        return clean

    def preprocess(self, pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        """Full preprocessing chain: downsample, then remove outliers.

        This is what Pipeline calls. The separate methods stay public because
        tests should be able to verify each step on its own.
        """
        return self.remove_outliers(self.downsample(pcd))
