"""Preprocessing: trim, shrink, and denoise the raw cloud before analysis.

The spatial processing happens in three layers, each with a different job:

Layer 1 — Voxel downsample.
The Eagle scan starts huge (~800k points). Doing nearest-neighbour outlier
removal on the full raw cloud is expensive. Downsampling first cuts the cloud
down to a fraction of its size, then SOR runs way cheaper on the smaller cloud.
Rule: downsample before any search-heavy step.

Layer 2 — Statistical Outlier Removal.
`clean, _ = pcd.remove_statistical_outlier(...)` returns two things: the clean
cloud and the surviving point indices. I only need the cloud, so `_` is Python's
"I know this exists and I am intentionally ignoring it" idiom.

Layer 3 — Crop.
The assignment asks for normals on the cropped point cloud. So after the cloud is
already smaller and cleaner, I carve a data-driven axis-aligned ROI before normal
estimation. Open3D's `pcd.crop(AxisAlignedBoundingBox)` is clean and fast because
it's just a point-in-box membership test.
"""

import logging

import numpy as np
import open3d as o3d

from src.config import Config

logger = logging.getLogger(__name__)


class Preprocessor:
    """Spatial crop, density reduction, and denoising for raw point clouds.

    Three operations are available — downsample, remove_outliers, crop_to_roi — each as
    its own public method so any stage can be tested in isolation.

    Pipeline order: downsample → remove_outliers → crop_to_roi.

    - Downsample before SOR: SOR computes nearest-neighbour distances, which is
      expensive. Running it on the already-reduced cloud reduces that cost
      significantly.
    - SOR before crop: removes floating ghost points before the crop bounds are
      used for the final normal-estimation ROI.
    - Crop before normals: makes the "cropped point cloud" requirement explicit.
    """

    def __init__(self, config: Config) -> None:
        # Underscore prefix marks these as internal stage parameters.
        self._crop_padding = config.crop_padding
        self._voxel_size = config.voxel_size
        self._nb_neighbors = config.sor_nb_neighbors
        self._std_ratio = config.sor_std_ratio

    def crop_to_roi(self, pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        """Axis-aligned bounding box crop: carve out the spatial region of interest.

        The AABB of the raw scan is computed from the data itself, then shrunk
        inward by `crop_padding` on every face. Any point that falls outside the
        reduced box is dropped.

        Why compute the AABB from data instead of hardcoding bounds:
        Hardcoded bounds break the moment the dataset changes. Data-driven bounds
        adapt automatically to whatever scan comes in. The only assumption I make
        is that the interesting geometry sits within a tight shell of the AABB
        center — the fringe is junk.

        Args:
            pcd: Raw input cloud.

        Returns:
            New PointCloud containing only points inside the padded AABB.
        """
        if pcd.is_empty():
            raise RuntimeError("Cannot crop an empty point cloud.")

        before = len(pcd.points)
        aabb = pcd.get_axis_aligned_bounding_box()

        if self._crop_padding <= 0:
            logger.info("AABB crop skipped (padding=0.000m): %s pts", f"{before:,}")
            return o3d.geometry.PointCloud(pcd)

        # Shrink the bounding box inward on all six faces.
        padding = np.full(3, self._crop_padding)
        min_bound = np.asarray(aabb.min_bound) + padding
        max_bound = np.asarray(aabb.max_bound) - padding

        if np.any(min_bound >= max_bound):
            raise ValueError(
                "crop_padding is too large for this point cloud's bounding box. "
                f"padding={self._crop_padding}, min={min_bound}, max={max_bound}"
            )

        roi = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=min_bound,
            max_bound=max_bound,
        )
        cropped = pcd.crop(roi)
        after = len(cropped.points)

        logger.info(
            "AABB crop (padding=%.3fm per face): %s -> %s pts (%d removed)",
            self._crop_padding,
            f"{before:,}",
            f"{after:,}",
            before - after,
        )
        return cropped

    def crop(self, pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        """Backward-compatible alias for crop_to_roi."""
        return self.crop_to_roi(pcd)

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
        """Full preprocessing chain: downsample, remove outliers, then crop ROI.

        This is what Pipeline calls. The separate methods stay public because
        tests should be able to verify each step on its own.
        """
        downsampled = self.downsample(pcd)
        clean = self.remove_outliers(downsampled)
        return self.crop_to_roi(clean)
