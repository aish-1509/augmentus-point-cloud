from __future__ import annotations

import logging

import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d

from src.config import Config

logger = logging.getLogger(__name__)


class ClusterExtractor:
    """
    Groups point cloud points into spatial clusters using DBSCAN.

    A note on the assignment requirement: "Perform Euclidean clustering".
    PCL has an explicit `EuclideanClusterExtraction` class, but Open3D gives us
    DBSCAN natively. For this assignment, DBSCAN matches the actual idea I need:
    group points by Euclidean distance while treating sparse points as noise.

    Why DBSCAN over K-means here:
    1. K-means needs `k` upfront. I do not know the number of components first.
    2. K-means assumes blob-ish clusters. Real scan shapes are not that polite.
    3. K-means forces every point into a cluster. Real scans have floating noise.
       DBSCAN labels that as noise (-1), which is exactly what I want.

    Responsibility:
    Segment the cleaned point cloud into individual components that can be
    tested, rendered, or saved separately.
    """

    def __init__(self, config: Config) -> None:
        self._eps = config.clustering_eps
        self._min_points = config.clustering_min_points

    def _run_dbscan(self, pcd: o3d.geometry.PointCloud) -> np.ndarray:
        """Run DBSCAN and return the per-point integer labels.

        Kept private so callers use `extract()` or `get_colored_cloud()` instead
        of depending on the raw numpy label array.
        """
        if pcd.is_empty():
            raise RuntimeError("Cannot cluster an empty point cloud.")

        labels = np.array(
            pcd.cluster_dbscan(
                eps=self._eps,
                min_points=self._min_points,
                print_progress=False,
            )
        )

        # labels.max() is the highest cluster index.
        # Example: max label 3 means clusters 0, 1, 2, 3 = four clusters.
        # Edge case: if everything is noise, max label is -1, so cluster count is 0.
        n_clusters = int(labels.max()) + 1 if labels.max() >= 0 else 0
        n_noise = int(np.sum(labels == -1))

        logger.info(
            "DBSCAN (eps=%.3f, min_points=%d): %d clusters found, %d noise points",
            self._eps,
            self._min_points,
            n_clusters,
            n_noise,
        )

        if n_clusters <= 1:
            logger.warning(
                "DBSCAN found %d cluster(s). If this looks wrong, tune "
                "clustering_eps in config.py first.",
                n_clusters,
            )

        return labels

    def extract(self, pcd: o3d.geometry.PointCloud) -> list[o3d.geometry.PointCloud]:
        """Return each cluster as a separate PointCloud.

        Noise points are excluded. The result is sorted largest-first, so
        `clusters[0]` should usually be the main body/component.

        Tiny Python note:
        `from __future__ import annotations` lets Python 3.9 understand the clean
        `list[o3d.geometry.PointCloud]` type hint without evaluating it too early.
        No runtime cost, just nicer annotations.
        """
        labels = self._run_dbscan(pcd)
        n_clusters = int(labels.max()) + 1 if labels.max() >= 0 else 0

        clusters: list[o3d.geometry.PointCloud] = []
        for cluster_id in range(n_clusters):
            # Get the actual point indices for this specific cluster.
            indices = np.where(labels == cluster_id)[0]

            # select_by_index creates a new cloud containing only these points.
            cluster_pcd = pcd.select_by_index(indices.tolist())
            clusters.append(cluster_pcd)

        # Largest first keeps downstream code predictable.
        clusters.sort(key=lambda cluster: len(cluster.points), reverse=True)

        logger.info("Extracted %d valid clusters; noise points were dropped.", len(clusters))
        return clusters

    def get_colored_cloud(self, pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        """Paint clusters for visual debugging.

        `tab20` gives clear repeated colours. Noise points stay black so they
        stand out immediately in the render.
        """
        labels = self._run_dbscan(pcd)
        n_clusters = max(int(labels.max()) + 1, 1)
        cmap = plt.get_cmap("tab20")

        colors = np.zeros((len(labels), 3))
        for i, label in enumerate(labels):
            if label == -1:
                colors[i] = [0.0, 0.0, 0.0]
            else:
                # Spread cluster IDs across the colour map instead of hand-picking.
                colors[i] = cmap(label / n_clusters)[:3]

        # Return a fresh cloud so this method never mutates the original data.
        colored = o3d.geometry.PointCloud(pcd)
        colored.colors = o3d.utility.Vector3dVector(colors)
        return colored
