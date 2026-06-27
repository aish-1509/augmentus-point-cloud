"""Unit tests for Preprocessor.

Strictly using synthetic in-memory data: zero network calls, zero file I/O.
"""

import numpy as np
import open3d as o3d

from src.config import Config
from src.preprocessor import Preprocessor

# ── THE 200-IQ TESTING BRAIN DUMP ─────────────────────────────────────────────
# Network calls inside unit tests are how CI/CD pipelines get bricked for no good
# reason. Downloading the Eagle scan just to test a voxel filter would be a total
# L: slow, flaky, and not even testing the thing I care about.
#
# Real unit tests should be pure, deterministic, and fast. So we yeet the real
# dataset here and generate synthetic numpy point clouds with default_rng. Same
# geometry ideas, no network, no files, no drama. W architecture tbh.
# ──────────────────────────────────────────────────────────────────────────────


def _make_cloud(n: int = 10_000, seed: int = 0) -> o3d.geometry.PointCloud:
    """Spawn a uniform random cloud inside a unit cube."""
    rng = np.random.default_rng(seed)
    pts = rng.uniform(-1.0, 1.0, (n, 3))
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    return pcd


def _make_cloud_with_outliers(
    n_inliers: int = 1_000,
    n_outliers: int = 50,
    seed: int = 42,
) -> o3d.geometry.PointCloud:
    """Simulate a dense target surface plus a few rogue scanner ghost points."""
    rng = np.random.default_rng(seed)
    core = rng.normal(0.0, 0.01, (n_inliers, 3))
    outliers = rng.uniform(50.0, 100.0, (n_outliers, 3))

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.vstack([core, outliers]))
    return pcd


class TestVoxelDownsampling:
    """
    Vibes are not enough. These tests check actual numeric behavior:
    fewer points, non-empty output, monotonic-ish voxel behavior, and sane bounds.
    Keeping receipts fr.
    """

    def test_reduces_point_count(self) -> None:
        """Primary assignment requirement: downsampling must reduce point count."""
        pcd = _make_cloud(10_000)
        prep = Preprocessor(Config(voxel_size=0.1))
        result = prep.downsample(pcd)

        assert len(result.points) < len(pcd.points), (
            f"Bro it didn't downsample. Expected fewer pts, got {len(result.points)}."
        )

    def test_output_is_not_empty(self) -> None:
        """Sanity check: don't accidentally snap 100% of points out of existence."""
        pcd = _make_cloud(5_000)
        result = Preprocessor(Config(voxel_size=0.05)).downsample(pcd)

        assert len(result.points) > 0, "Cloud is literally empty. Complete L."

    def test_larger_voxel_gives_fewer_points(self) -> None:
        """
        Geometry check: bigger voxels should merge more points.

        As voxel volume gets larger, the number of occupied voxel cells should go
        down. If a bigger voxel creates more points, math is mathing backwards.
        """
        pcd = _make_cloud(10_000)
        small = Preprocessor(Config(voxel_size=0.05)).downsample(pcd)
        large = Preprocessor(Config(voxel_size=0.20)).downsample(pcd)

        assert len(large.points) < len(small.points), "Math is mathing backwards rn."

    def test_points_stay_within_original_bounds(self) -> None:
        """
        Downsampled centroids should stay inside the original bounding box.

        If a centroid escapes the source bounds, spatial topology got corrupted.
        """
        pcd = _make_cloud(10_000)
        result = Preprocessor(Config(voxel_size=0.1)).downsample(pcd)

        orig = np.asarray(pcd.points)
        res = np.asarray(result.points)

        # 1e-6 tolerance bc float64 math can be a lil quirky at the edges.
        assert np.all(res >= orig.min(axis=0) - 1e-6)
        assert np.all(res <= orig.max(axis=0) + 1e-6)


class TestStatisticalOutlierRemoval:
    """
    SOR is basically the data sanitation layer.

    If floating garbage points survive, normals/clustering can get cooked later.
    These tests prove it removes obvious distant noise without nuking the valid
    inlier surface.
    """

    def test_removes_some_points_from_noisy_cloud(self) -> None:
        """SOR must remove at least some distant outlier points."""
        pcd = _make_cloud_with_outliers()
        result = Preprocessor(
            Config(sor_nb_neighbors=10, sor_std_ratio=1.5)
        ).remove_outliers(pcd)

        assert len(result.points) < len(pcd.points), "Noise wasn't removed tbh."

    def test_preserves_most_inlier_points(self) -> None:
        """
        With std_ratio=2.0, SOR should trim extremes, not delete the whole core.

        If it deletes half the valid surface, the params are way too aggressive.
        """
        pcd = _make_cloud_with_outliers(n_inliers=1_000, n_outliers=50)
        result = Preprocessor(
            Config(sor_nb_neighbors=10, sor_std_ratio=2.0)
        ).remove_outliers(pcd)

        assert len(result.points) >= 900, (
            f"SOR was way too aggressive bro. Expected >=900 inliers, got {len(result.points)}."
        )
