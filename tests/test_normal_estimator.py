"""Unit tests for NormalEstimator using a synthetic plane."""

import numpy as np
import open3d as o3d
import pytest

from src.config import Config
from src.normal_estimator import NormalEstimator


def _make_plane(n: int = 30, spacing: float = 0.03) -> o3d.geometry.PointCloud:
    """Create an n x n near-plane with tiny deterministic depth noise."""
    xs = np.linspace(-spacing * n / 2, spacing * n / 2, n)
    ys = np.linspace(-spacing * n / 2, spacing * n / 2, n)
    xx, yy = np.meshgrid(xs, ys)
    rng = np.random.default_rng(123)
    zz = rng.normal(0.0, 1e-5, size=xx.shape)
    pts = np.column_stack([xx.ravel(), yy.ravel(), zz.ravel()])

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    return pcd


def test_normal_estimation_adds_one_finite_normal_per_point() -> None:
    """Normals must exist, match point count, and contain finite values."""
    pcd = _make_plane()
    result = NormalEstimator(Config(normal_radius=0.12, normal_max_nn=30)).estimate(pcd)

    normals = np.asarray(result.normals)
    assert result.has_normals()
    assert normals.shape == (len(result.points), 3)
    assert np.all(np.isfinite(normals))


def test_normal_estimation_rejects_empty_cloud() -> None:
    """Empty clouds should fail clearly instead of silently continuing."""
    empty = o3d.geometry.PointCloud()

    with pytest.raises(RuntimeError, match="empty point cloud"):
        NormalEstimator(Config()).estimate(empty)
