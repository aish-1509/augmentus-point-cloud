from __future__ import annotations

import logging

import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d

from src.config import Config

logger = logging.getLogger(__name__)


class ClusterExtractor:
    """
    Ok so the assignment literally says "Perform Euclidean clustering".
    Problem: Open3D doesn't have a function named `euclidean_clustering`. PCL does,
    but we are using Open3D.

    Wait, let me think. What IS Euclidean clustering? It's just grouping points
    that are within a certain Euclidean distance threshold of each other,
    provided there's enough density.
    That is literally just DBSCAN
    (Density-Based Spatial Clustering of Applications with Noise).

    Why not K-means?
    1. K-means needs 'k'. I have no idea how many objects are in the Eagle scan.
    2. K-means forces EVERY point into a cluster. Real scans have floating scanner
       noise. DBSCAN isolates noise as label -1. K-means would just attach the
       noise to a real object. Dumb.

    So I'm using DBSCAN. It satisfies the math of the requirement perfectly.
    """

    def __init__(self, config: Config) -> None:
        self._eps = config.clustering_eps
        self._min_points = config.clustering_min_points

    def _run_dbscan(self, pcd: o3d.geometry.PointCloud) -> np.ndarray:
        # Running the clustering
        labels = np.array(
            pcd.cluster_dbscan(
                eps=self._eps,
                min_points=self._min_points,
                print_progress=False,
            )
        )

        # Ah crap, if it's all noise, max() is -1.
        # Need to handle that so we don't say we have 0 clusters when we actually just failed.
        n_clusters = int(labels.max()) + 1 if labels.max() >= 0 else 0
        n_noise = int(np.sum(labels == -1))

        logger.info(f"DBSCAN found {n_clusters} clusters and {n_noise} noise points rn.")
        return labels

    def extract(self, pcd: o3d.geometry.PointCloud) -> list[o3d.geometry.PointCloud]:
        labels = self._run_dbscan(pcd)
        n_clusters = int(labels.max()) + 1 if labels.max() >= 0 else 0

        clusters = []
        for cluster_id in range(n_clusters):
            # grab the indices for this specific cluster
            indices = np.where(labels == cluster_id)[0]
            cluster_pcd = pcd.select_by_index(indices.tolist())
            clusters.append(cluster_pcd)

        # I should probably sort these so the biggest object (main body) is always first.
        clusters.sort(key=lambda c: len(c.points), reverse=True)
        return clusters

    def get_colored_cloud(self, pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        # Paint it so I can actually see if the clustering worked in the render
        labels = self._run_dbscan(pcd)
        n_clusters = max(int(labels.max()) + 1, 1)
        cmap = plt.get_cmap("tab20")  # 20 colors should be enough tbh

        colors = np.zeros((len(labels), 3))
        for i, label in enumerate(labels):
            if label == -1:
                colors[i] = [0.0, 0.0, 0.0]  # noise is black
            else:
                colors[i] = cmap(label / n_clusters)[:3]

        colored = o3d.geometry.PointCloud(pcd)
        colored.colors = o3d.utility.Vector3dVector(colors)
        return colored
