"""Loading notes for the Eagle point cloud.

A few things I had to get straight here:

- `o3d.data.EaglePointCloud()` is Open3D's built-in dataset manager. On the
  first run it downloads the `.pcd` file into `~/open3d_data/`, then after that
  it just reuses the cached file. The useful thing it gives me is `dataset.path`.
- `o3d.io.read_point_cloud(path)` is Open3D's general point cloud reader. It
  detects the format from the file extension and returns an
  `o3d.geometry.PointCloud`. The actual point storage is inside Open3D; when I
  need to inspect it in Python I can use `np.asarray(pcd.points)`.
- `pcd.is_empty()` is a guard against a silent bad run. Without it, the rest of
  the pipeline could process an empty cloud and still produce output files, but
  those outputs would be meaningless.
- `logging.getLogger(__name__)` keeps log messages tied to this module. When
  this file is imported as part of the package, `__name__` becomes `src.loader`,
  so the logs show where the message came from.
"""

import logging

import open3d as o3d

logger = logging.getLogger(__name__)


class Loader:
    """Loads point cloud datasets from Open3D's built-in data utilities.

    Responsibility: acquiring raw data only.
    This class knows nothing about preprocessing, normals, or clustering.
    """

    def load_eagle(self) -> o3d.geometry.PointCloud:
        """Load and return the Eagle point cloud dataset.

        Open3D downloads the file automatically on the first call and caches
        it locally (~50 MB). Subsequent calls use the local cache.

        Returns:
            o3d.geometry.PointCloud: The raw unprocessed point cloud.

        Raises:
            RuntimeError: If the dataset loads but contains zero points.
        """
        dataset = o3d.data.EaglePointCloud()
        # dataset.path is the local file path after download.
        # o3d.io.read_point_cloud reads .pcd, .ply, .xyz, .xyzn, and others.
        pcd = o3d.io.read_point_cloud(dataset.path)

        if pcd.is_empty():
            raise RuntimeError(
                "Loaded point cloud has no points. "
                "Check your Open3D installation and network access."
            )

        logger.info("Loaded Eagle dataset: %s points", f"{len(pcd.points):,}")
        return pcd
