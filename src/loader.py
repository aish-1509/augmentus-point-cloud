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
