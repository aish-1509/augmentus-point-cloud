from __future__ import annotations

import json
import logging
import os
from collections import deque
from typing import Any

import numpy as np
import open3d as o3d

from src.config import Config

logger = logging.getLogger(__name__)

# Hand-picked palette of 20 vibrant colors tuned for dark-background renders.
# tab20 is fine on white plots, but several colors look muted on near-black.
# These are intentionally bright so tiny clusters still pop in the PNGs.
_VIBRANT_PALETTE: np.ndarray = np.array(
    [
        [0.00, 1.00, 0.80],
        [1.00, 0.40, 0.00],
        [0.70, 0.00, 1.00],
        [1.00, 1.00, 0.00],
        [0.00, 0.60, 1.00],
        [1.00, 0.00, 0.50],
        [0.20, 1.00, 0.00],
        [1.00, 0.80, 0.00],
        [0.00, 0.90, 0.90],
        [1.00, 0.20, 0.80],
        [0.50, 1.00, 0.00],
        [0.00, 0.40, 1.00],
        [1.00, 0.60, 0.00],
        [0.80, 1.00, 0.00],
        [0.00, 0.80, 0.50],
        [1.00, 0.00, 0.20],
        [0.40, 0.00, 1.00],
        [0.00, 1.00, 0.50],
        [1.00, 0.50, 0.50],
        [0.50, 0.00, 1.00],
    ],
    dtype=np.float64,
)


class ClusterExtractor:
    """Extracts Euclidean point-cloud clusters with KD-tree radius expansion.

    The assignment says "Euclidean clustering", so the default implementation is
    the classic PCL-style idea:

    1. Build a KD-tree over the point cloud.
    2. Pick an unvisited seed point.
    3. Radius-search all neighbours within `clustering_eps`.
    4. Grow the connected component with a queue.
    5. Keep the component if its size is inside the configured limits.

    DBSCAN is still available as an optional helper for comparison, but the main
    pipeline uses this explicit Euclidean radius-search implementation.
    """

    def __init__(self, config: Config) -> None:
        self._tolerance = config.clustering_eps
        self._min_points = config.clustering_min_points
        self._max_points = config.clustering_max_points

    def extract_euclidean_clusters(
        self,
        pcd: o3d.geometry.PointCloud,
    ) -> list[np.ndarray]:
        """Return sorted arrays of point indices, one array per valid cluster."""
        if pcd.is_empty():
            raise RuntimeError("Cannot cluster an empty point cloud.")

        points = np.asarray(pcd.points)
        tree = o3d.geometry.KDTreeFlann(pcd)
        visited = np.zeros(len(points), dtype=bool)
        clusters: list[np.ndarray] = []
        rejected_small = 0
        rejected_large = 0

        for seed_idx in range(len(points)):
            if visited[seed_idx]:
                continue

            queue: deque[int] = deque([seed_idx])
            visited[seed_idx] = True
            cluster_indices: list[int] = []

            while queue:
                current_idx = queue.popleft()
                cluster_indices.append(current_idx)

                _, neighbour_indices, _ = tree.search_radius_vector_3d(
                    points[current_idx],
                    self._tolerance,
                )

                for neighbour_idx in neighbour_indices:
                    if not visited[neighbour_idx]:
                        visited[neighbour_idx] = True
                        queue.append(int(neighbour_idx))

            size = len(cluster_indices)
            if size < self._min_points:
                rejected_small += 1
                continue
            if self._max_points is not None and size > self._max_points:
                rejected_large += 1
                continue

            clusters.append(np.asarray(cluster_indices, dtype=np.int64))

        clusters.sort(key=len, reverse=True)

        logger.info(
            (
                "Euclidean clustering (tolerance=%.3fm, min=%d, max=%s): "
                "%d clusters kept, %d small components dropped, %d large components dropped"
            ),
            self._tolerance,
            self._min_points,
            "None" if self._max_points is None else self._max_points,
            len(clusters),
            rejected_small,
            rejected_large,
        )
        return clusters

    def labels_from_clusters(
        self,
        point_count: int,
        cluster_indices: list[np.ndarray],
    ) -> np.ndarray:
        """Build a per-point label array from sorted cluster index arrays."""
        labels = np.full(point_count, -1, dtype=np.int32)
        for cluster_id, indices in enumerate(cluster_indices):
            labels[indices] = cluster_id
        return labels

    def get_cluster_point_clouds(
        self,
        pcd: o3d.geometry.PointCloud,
        cluster_indices: list[np.ndarray],
    ) -> list[o3d.geometry.PointCloud]:
        """Convert cluster index arrays into separate Open3D PointCloud objects."""
        return [pcd.select_by_index(indices.tolist()) for indices in cluster_indices]

    def extract(self, pcd: o3d.geometry.PointCloud) -> list[o3d.geometry.PointCloud]:
        """Return each valid Euclidean cluster as a separate PointCloud."""
        cluster_indices = self.extract_euclidean_clusters(pcd)
        clusters = self.get_cluster_point_clouds(pcd, cluster_indices)
        logger.info("Extracted %d valid Euclidean clusters.", len(clusters))
        return clusters

    def get_cluster_color(self, cluster_id: int) -> np.ndarray:
        """Return the deterministic RGB color assigned to a cluster ID."""
        return _VIBRANT_PALETTE[cluster_id % len(_VIBRANT_PALETTE)].copy()

    def get_colored_cloud(
        self,
        pcd: o3d.geometry.PointCloud,
        labels: np.ndarray | None = None,
    ) -> o3d.geometry.PointCloud:
        """Paint each point by Euclidean cluster label.

        Noise/unkept points use near-black so they remain visible on the dark
        renderer without pretending to be a real cluster.
        """
        if labels is None:
            cluster_indices = self.extract_euclidean_clusters(pcd)
            labels = self.labels_from_clusters(len(pcd.points), cluster_indices)

        colors = np.zeros((len(labels), 3), dtype=np.float64)
        for i, label in enumerate(labels):
            if label == -1:
                colors[i] = [0.08, 0.08, 0.08]
            else:
                colors[i] = self.get_cluster_color(int(label))

        colored = o3d.geometry.PointCloud(pcd)
        colored.colors = o3d.utility.Vector3dVector(colors)
        return colored

    def build_cluster_summary(
        self,
        clusters: list[o3d.geometry.PointCloud],
        total_points: int,
    ) -> list[dict[str, Any]]:
        """Build serializable cluster statistics for README/debugging."""
        summary: list[dict[str, Any]] = []
        for cluster_id, cluster in enumerate(clusters):
            pts = np.asarray(cluster.points)
            if len(pts) == 0:
                continue

            min_bound = pts.min(axis=0)
            max_bound = pts.max(axis=0)
            extent = max_bound - min_bound
            centroid = pts.mean(axis=0)

            summary.append(
                {
                    "cluster_id": cluster_id,
                    "point_count": int(len(pts)),
                    "percentage_of_cloud": float(len(pts) / total_points * 100.0),
                    "bbox_min": min_bound.round(6).tolist(),
                    "bbox_max": max_bound.round(6).tolist(),
                    "bbox_extent": extent.round(6).tolist(),
                    "centroid": centroid.round(6).tolist(),
                }
            )
        return summary

    def save_cluster_summary(
        self,
        summary: list[dict[str, Any]],
        path: str,
    ) -> str:
        """Write cluster summary JSON and return the absolute path."""
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(summary, f, indent=2)
            f.write("\n")
        logger.info("Cluster summary written to %s", path)
        return os.path.abspath(path)

    def run_dbscan_optional(self, pcd: o3d.geometry.PointCloud) -> np.ndarray:
        """Optional DBSCAN comparison path; not used by the main pipeline."""
        if pcd.is_empty():
            raise RuntimeError("Cannot cluster an empty point cloud.")

        labels = np.array(
            pcd.cluster_dbscan(
                eps=self._tolerance,
                min_points=self._min_points,
                print_progress=False,
            )
        )
        n_clusters = int(labels.max()) + 1 if labels.max() >= 0 else 0
        n_noise = int(np.sum(labels == -1))
        logger.info(
            "Optional DBSCAN comparison: %d clusters, %d noise points",
            n_clusters,
            n_noise,
        )
        return labels
