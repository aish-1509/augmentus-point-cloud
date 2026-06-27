"""Normal estimation: giving the point cloud a sense of surface direction.

This was the part I had to slow down for a bit.

A raw point cloud is basically a soup of xyz points. The points say where the
surface probably is, but not which way the surface is facing. Normals add that
"which way is this patch pointing?" info.

The mental model:
- Look at one point and its nearby neighbours.
- Those neighbours usually spread out like a tiny local sheet.
- PCA finds the main spread directions of that sheet.
- The direction with the least spread is the one sticking out of the sheet.
  That is the normal.

One important gotcha: PCA gives a direction line, but the sign can flip. So one
normal may point one way and the next may point the exact opposite way. Open3D's
orientation step makes nearby normals agree with each other, which is what later
geometry steps need.
"""

import logging

import open3d as o3d

from src.config import Config

logger = logging.getLogger(__name__)


class NormalEstimator:
    """Estimates surface normals for each point in a cloud using PCA.

    Responsibility:
    Take the cleaned, downsampled cloud and attach local surface direction to it.

    Why the hybrid KD-tree search:
    A pure radius search can accidentally grab way too many neighbours in dense
    scan patches. Hybrid search still uses the physical radius, but caps the
    count with max_nn so one dense patch does not hog the CPU.
    """

    def __init__(self, config: Config) -> None:
        # Keep the knobs in config.py, not scattered across the codebase.
        self._radius = config.normal_radius
        self._max_nn = config.normal_max_nn

    def estimate(self, pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        """Compute and consistently orient surface normals.

        KDTreeSearchParamHybrid means:
        "Use neighbours within this radius, but never more than max_nn."

        orient_normals_consistent_tangent_plane(100) means:
        "Use a local graph of nearby tangent planes and flip normals so adjacent
        normals agree." It makes orientation consistent, not magically perfect.

        Args:
            pcd: Preprocessed cloud. Open3D modifies it in place.

        Returns:
            The same PointCloud object, now with normals attached.
        """
        if pcd.is_empty():
            raise RuntimeError("Cannot estimate normals on an empty point cloud.")

        search_param = o3d.geometry.KDTreeSearchParamHybrid(
            radius=self._radius,
            max_nn=self._max_nn,
        )

        pcd.estimate_normals(search_param)

        # PCA normals are direction-ambiguous at first. This step flips them so
        # neighbouring surface patches mostly agree instead of looking random.
        pcd.orient_normals_consistent_tangent_plane(100)

        logger.info(
            "Normals estimated (radius=%.2fm, max_nn=%d) for %s pts",
            self._radius,
            self._max_nn,
            f"{len(pcd.points):,}",
        )
        return pcd
