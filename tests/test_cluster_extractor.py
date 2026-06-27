import numpy as np
import open3d as o3d

from src.cluster_extractor import ClusterExtractor
from src.config import Config

# Bro wait. The Augmentus PDF literally says I need a test to "verify clustering
# produces more than one segment".
# I can't just feed it a random cube of points, because DBSCAN might just classify
# the whole cube as ONE cluster if the density is uniform.
# I need to intentionally engineer two completely separate topological islands
# so I can mathematically guarantee the algorithm splits them.


def _make_two_blobs(
    n: int = 400,
    separation: float = 2.0,
    spread: float = 0.02,
    seed: int = 42,
):
    # Generating two tight Gaussian distributions separated by 2 meters.
    # Because 2.0m >> eps (0.05m), DBSCAN literally has no choice but to split them.
    rng = np.random.default_rng(seed)

    # Blob A at the origin
    a = rng.normal([0.0, 0.0, 0.0], spread, (n, 3))
    # Blob B translated away
    b = rng.normal([separation, separation, separation], spread, (n, 3))

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.vstack([a, b]))
    return pcd


class TestClusterExtractor:
    def test_produces_more_than_one_cluster(self):
        # Fulfilling the exact assignment requirement rn
        pcd = _make_two_blobs()
        clusters = ClusterExtractor(
            Config(clustering_eps=0.05, clustering_min_points=10)
        ).extract(pcd)

        # If this is 1, the eps is way too high or the data is overlapping.
        assert len(clusters) > 1

    # Gonna add tests for sorting and RGB validation next commit.
