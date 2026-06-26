"""Dataset loading for Open3D's Eagle point cloud sample."""
import logging
import open3d as o3d

logger = logging.getLogger(__name__)

class Loader:
    """Load the Eagle point cloud sample from Open3D's dataset nd just loads point cloud datasets from o3d's built-in data utilities."""

    def __init__(self):
        self._pcd = None

    def load(self) -> o3d.geometry.PointCloud:
        """Load the Eagle point cloud sample.

        Returns:
            o3d.geometry.PointCloud: The loaded point cloud.
        """
        if self._pcd is None:
            logger.info("Loading Eagle point cloud sample...")
            self._pcd = o3d.data.EaglePointCloud()
            logger.info("Eagle point cloud sample loaded.")
        return self._pcd