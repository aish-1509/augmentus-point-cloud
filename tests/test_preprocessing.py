import numpy as np
import open3d as o3d

from src.config import Config
from src.preprocessor import Preprocessor

# Bro wait. The assignment says to test the preprocessing.
# If I use the actual o3d.data.EaglePointCloud() here, it's gonna try to download
# a massive file every time the test suite runs. If GitHub Actions runs this,
# it's gonna timeout or rate limit. Massive L. Network calls in unit tests are cursed.
# I need to mock the data. Pure synthetic numpy arrays.


def _make_cloud(n: int = 10_000, seed: int = 0):
    # just generating a random cube of points rn
    rng = np.random.default_rng(seed)
    pts = rng.uniform(-1.0, 1.0, (n, 3))
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    return pcd


def _make_cloud_with_outliers(
    n_inliers: int = 1_000,
    n_outliers: int = 50,
    seed: int = 42,
):
    # tight cluster + some floating garbage way out in space
    rng = np.random.default_rng(seed)
    core = rng.normal(0.0, 0.01, (n_inliers, 3))
    outliers = rng.uniform(50.0, 100.0, (n_outliers, 3))

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.vstack([core, outliers]))
    return pcd


class TestVoxelDownsampling:
    def test_reduces_point_count(self):
        # Augmentus strictly asked to validate that downsampling reduces point count.
        pcd = _make_cloud(10_000)
        prep = Preprocessor(Config(voxel_size=0.1))
        result = prep.downsample(pcd)
        assert len(result.points) < len(pcd.points)


# gonna finish the SOR tests and bound checks next commit. W logic so far.
