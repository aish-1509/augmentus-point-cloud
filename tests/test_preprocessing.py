"""Unit tests for Preprocessor.

Strictly using synthetic in-memory data: zero network calls, zero file I/O.
"""

import numpy as np
import open3d as o3d

from src.config import Config
from src.preprocessor import Preprocessor

# ── Test Data Strategy ────────────────────────────────────────────────────────
# Network calls inside unit tests make CI slower and less reliable. Downloading
# the Eagle scan just to test a voxel filter would make these tests depend on
# external state instead of the function under test.
#
# These tests use synthetic numpy point clouds with default_rng so they stay
# deterministic, fast, and offline while still checking the same geometry ideas.
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
    These tests check measurable downsampling behavior: fewer points, non-empty
    output, monotonic voxel behavior, and valid output bounds.
    """

    def test_reduces_point_count(self) -> None:
        """Primary assignment requirement: downsampling must reduce point count."""
        pcd = _make_cloud(10_000)
        prep = Preprocessor(Config(voxel_size=0.1))
        result = prep.downsample(pcd)

        assert len(result.points) < len(pcd.points), (
            f"Downsampling did not reduce point count; got {len(result.points)} points."
        )

    def test_output_is_not_empty(self) -> None:
        """Sanity check: don't accidentally snap 100% of points out of existence."""
        pcd = _make_cloud(5_000)
        result = Preprocessor(Config(voxel_size=0.05)).downsample(pcd)

        assert len(result.points) > 0, "Downsampled cloud should not be empty."

    def test_larger_voxel_gives_fewer_points(self) -> None:
        """
        Geometry check: bigger voxels should merge more points.

        As voxel volume gets larger, the number of occupied voxel cells should go
        down. If a bigger voxel creates more points, the voxel behavior is wrong.
        """
        pcd = _make_cloud(10_000)
        small = Preprocessor(Config(voxel_size=0.05)).downsample(pcd)
        large = Preprocessor(Config(voxel_size=0.20)).downsample(pcd)

        assert len(large.points) < len(small.points), (
            "Larger voxel size should produce fewer points."
        )

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

    If distant noise points survive, normals and clustering can become unstable.
    These tests verify that SOR removes obvious distant noise without deleting the
    valid inlier surface.
    """

    def test_removes_some_points_from_noisy_cloud(self) -> None:
        """SOR must remove at least some distant outlier points."""
        pcd = _make_cloud_with_outliers()
        result = Preprocessor(
            Config(sor_nb_neighbors=10, sor_std_ratio=1.5)
        ).remove_outliers(pcd)

        assert len(result.points) < len(pcd.points), (
            "Statistical outlier removal should remove distant noise points."
        )

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
            f"SOR removed too many inliers; expected >=900 points, got {len(result.points)}."
        )


class TestCrop:
    """
    Crop carves a spatial region of interest from the raw scan before any
    density-reduction or search-heavy step runs.

    Tests use the same synthetic unit-cube cloud. The cube spans [-1, 1]^3,
    so its AABB is exactly 2m per side. With crop_padding=0.3, the surviving
    box is [-0.7, 0.7]^3 — a 1.4m cube — which holds roughly (1.4/2.0)^3 ≈ 34%
    of uniformly distributed points, so about 66% should be removed. That makes
    the effect clearly measurable without needing the real Eagle scan.
    """

    def test_crop_never_increases_point_count(self) -> None:
        """Crop can only remove points — it must never add new ones."""
        pcd = _make_cloud(5_000)
        result = Preprocessor(Config(crop_padding=0.05)).crop(pcd)

        assert len(result.points) <= len(pcd.points), (
            "Crop returned more points than the input, which is geometrically impossible."
        )

    def test_crop_removes_boundary_points_with_large_padding(self) -> None:
        """
        With aggressive padding, crop should visibly shrink the cloud.

        Padding of 0.3m on a 2m cube removes the outer 15% shell on each side,
        which cuts roughly 66% of uniformly distributed points.
        """
        pcd = _make_cloud(10_000)
        result = Preprocessor(Config(crop_padding=0.3)).crop(pcd)

        assert len(result.points) < len(pcd.points), (
            "crop_padding=0.3 should remove a significant fraction of a unit-cube cloud."
        )
        # Expect at most 50% of points to survive (actual ~34% from volume math).
        assert len(result.points) <= len(pcd.points) * 0.50, (
            f"Expected <=50% of points to survive; got {len(result.points)} / {len(pcd.points)}."
        )

    def test_cropped_points_stay_inside_padded_aabb(self) -> None:
        """
        Every surviving point must sit strictly inside the padded bounding box.

        If any point escapes the padded AABB, the crop implementation has a
        boundary condition bug.
        """
        pcd = _make_cloud(5_000)
        prep = Preprocessor(Config(crop_padding=0.2))
        result = prep.crop(pcd)

        orig_pts = np.asarray(pcd.points)
        res_pts = np.asarray(result.points)

        # Recompute what the padded bounds should be.
        padding = np.full(3, 0.2)
        expected_min = orig_pts.min(axis=0) + padding
        expected_max = orig_pts.max(axis=0) - padding

        # 1e-9 tolerance for float64 boundary precision.
        assert np.all(res_pts >= expected_min - 1e-9), (
            "A cropped point falls below the expected padded minimum bound."
        )
        assert np.all(res_pts <= expected_max + 1e-9), (
            "A cropped point exceeds the expected padded maximum bound."
        )

    def test_zero_padding_preserves_all_points(self) -> None:
        """crop_padding=0.0 is a no-op — every point should survive."""
        pcd = _make_cloud(3_000)
        result = Preprocessor(Config(crop_padding=0.0)).crop(pcd)

        assert len(result.points) == len(pcd.points), (
            "crop_padding=0.0 should be a no-op and preserve every point."
        )
