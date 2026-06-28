"""Unit tests for ClusterExtractor.

Strictly uses synthetic two-blob geometry: fast, deterministic, zero network
overhead.
"""

import numpy as np
import open3d as o3d

from src.cluster_extractor import ClusterExtractor
from src.config import Config

# ── Synthetic Test Data ───────────────────────────────────────────────────────
# The prompt explicitly requires testing that we get >1 segment.
# You can't test clustering with uniform random noise; it'll just become one
# connected component or many tiny fragments depending on the tolerance.
#
# The reliable move is to engineer two tight Gaussian blobs in 3D space, separated
# by a distance much larger than `eps` (2.0m >> 0.05m). Euclidean radius
# reachability must fail across that empty gap, so the blobs should split cleanly.
# ──────────────────────────────────────────────────────────────────────────────


def _make_two_blobs(
    n: int = 400,
    separation: float = 2.0,
    spread: float = 0.02,
    seed: int = 42,
) -> o3d.geometry.PointCloud:
    """Spawn two Gaussian blobs separated by `separation` metres."""
    rng = np.random.default_rng(seed)
    a = rng.normal([0.0, 0.0, 0.0], spread, (n, 3))
    b = rng.normal([separation, separation, separation], spread, (n, 3))

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.vstack([a, b]))
    return pcd


def _make_uneven_blobs(seed: int = 7) -> o3d.geometry.PointCloud:
    """Spawn blobs with different sizes so sorting largest-first is testable."""
    rng = np.random.default_rng(seed)
    big = rng.normal([0.0, 0.0, 0.0], 0.015, (500, 3))
    small = rng.normal([2.0, 2.0, 2.0], 0.015, (120, 3))

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.vstack([big, small]))
    return pcd


class TestClusterExtractor:
    """
    These tests use hard, deterministic geometry: two separated point islands.
    If Euclidean clustering merges them, tolerance is too big. If it makes extra
    clusters, min size or spread is off. Either way, the test catches the
    topology bug fast.
    """

    def test_produces_more_than_one_cluster(self) -> None:
        """Primary requirement: clustering must produce more than one segment."""
        pcd = _make_two_blobs()
        extractor = ClusterExtractor(
            Config(clustering_eps=0.05, clustering_min_points=10)
        )
        clusters = extractor.extract(pcd)

        assert len(clusters) > 1, (
            f"Expected more than one cluster, got {len(clusters)}. "
            "Check the clustering_eps threshold."
        )

    def test_finds_exactly_two_blobs(self) -> None:
        """
        Two distinct topological islands should yield exactly two clusters.

        If it finds 3, it's hallucinating fragments as clusters. If it finds 1,
        eps is too big and bridged the gap.
        """
        pcd = _make_two_blobs(spread=0.01, separation=3.0)
        extractor = ClusterExtractor(
            Config(clustering_eps=0.05, clustering_min_points=10)
        )
        clusters = extractor.extract(pcd)

        assert len(clusters) == 2, (
            f"Expected exactly two clusters, got {len(clusters)}."
        )

    def test_clusters_sorted_largest_first(self) -> None:
        """
        Downstream stages usually care about the main object first.

        If extract() doesn't sort descending, the pipeline might try to process a
        tiny dust cluster before the actual object.
        """
        pcd = _make_uneven_blobs()
        extractor = ClusterExtractor(
            Config(clustering_eps=0.05, clustering_min_points=10)
        )
        clusters = extractor.extract(pcd)

        sizes = [len(cluster.points) for cluster in clusters]
        assert sizes == sorted(sizes, reverse=True), (
            "List is not sorted largest-first. Downstream processing may inspect "
            "a tiny fragment before the main object."
        )

    def test_colored_cloud_preserves_point_count(self) -> None:
        """Painting the cloud should not change the number of points."""
        pcd = _make_two_blobs()
        extractor = ClusterExtractor(
            Config(clustering_eps=0.05, clustering_min_points=10)
        )
        colored = extractor.get_colored_cloud(pcd)

        assert len(colored.points) == len(pcd.points), (
            "Colored cloud should preserve point count."
        )

    def test_colored_cloud_has_valid_rgb(self) -> None:
        """
        RGB arrays must stay inside [0.0, 1.0].

        If Open3D gets invalid RGB floats, renders can look blank or clipped.
        """
        pcd = _make_two_blobs()
        extractor = ClusterExtractor(
            Config(clustering_eps=0.05, clustering_min_points=10)
        )
        colored = extractor.get_colored_cloud(pcd)

        assert colored.has_colors(), "Renderer returned a blank cloud."

        cols = np.asarray(colored.colors)
        assert cols.shape == (len(pcd.points), 3), "RGB matrix shape should be (N, 3)."
        assert np.all(cols >= 0.0) and np.all(cols <= 1.0), (
            "RGB values out of bounds. Must be [0, 1]."
        )

    def test_labels_mark_unkept_noise_as_minus_one(self) -> None:
        """Tiny components below min size should become noise label -1."""
        rng = np.random.default_rng(99)
        blob = rng.normal([0.0, 0.0, 0.0], 0.01, (80, 3))
        speck = np.array([[2.0, 2.0, 2.0]])
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.vstack([blob, speck]))

        extractor = ClusterExtractor(Config(clustering_eps=0.05, clustering_min_points=10))
        cluster_indices = extractor.extract_euclidean_clusters(pcd)
        labels = extractor.labels_from_clusters(len(pcd.points), cluster_indices)

        assert len(cluster_indices) == 1
        assert labels[-1] == -1

    def test_cluster_summary_contains_geometry_stats(self) -> None:
        """Cluster summaries should expose counts, bbox, and centroid data."""
        pcd = _make_two_blobs()
        extractor = ClusterExtractor(Config(clustering_eps=0.05, clustering_min_points=10))
        clusters = extractor.extract(pcd)
        summary = extractor.build_cluster_summary(clusters, len(pcd.points))

        assert len(summary) == 2
        assert summary[0]["point_count"] >= summary[1]["point_count"]
        assert "bbox_min" in summary[0]
        assert "bbox_max" in summary[0]
        assert "bbox_extent" in summary[0]
        assert "centroid" in summary[0]
