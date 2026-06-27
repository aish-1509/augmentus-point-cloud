import logging
import os

import matplotlib

# CRITICAL: set Agg before pyplot, or CI/Docker/headless runs can crash fr.
# Open3D's native viewer wants a display manager. GitHub Actions usually has none.
# Agg renders straight to a file without opening a GUI. Massive W for portability.
matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d

logger = logging.getLogger(__name__)


class Visualizer:
    """
    Saves point clouds to PNGs so I can actually see what the pipeline is doing.

    Brain dump on why it's built like this:
    Matplotlib 3D scatter is CPU-bound. If I yeet 1.5M points into it, the
    script can hang for ages and create a chunky PNG. Total L.

    First render pass was technically correct but visually weak: pale background,
    gridlines stealing attention, and too few points for the object to feel solid.
    So this renderer treats the output more like a clean product shot: dark
    background, no axes, denser sampling, and a fixed camera angle.
    """

    def __init__(self, output_dir: str = "docs/renders") -> None:
        self._output_dir = output_dir
        # exist_ok=True is goated: run #2 shouldn't crash just bc the folder exists.
        os.makedirs(output_dir, exist_ok=True)
        logger.info("Renders dropping here: %s", os.path.abspath(output_dir))

    @property
    def output_dir(self) -> str:
        return self._output_dir

    def save_render(
        self,
        pcd: o3d.geometry.PointCloud,
        filename: str,
        title: str = "",
        subsample: int = 50000,
    ) -> str:
        """Save a 3D scatter view of the cloud as a PNG.

        Points are randomly dropped if the cloud is larger than `subsample`.
        That is render-only; the actual Open3D point cloud is not mutated.
        """
        pts = np.asarray(pcd.points)
        if len(pts) == 0:
            logger.warning("Cloud is totally empty ngl. Skipping %s", filename)
            return ""

        # Subsample for rendering ONLY. Never mutate the real pipeline data.
        if len(pts) > subsample:
            # Fixed seed = reproducible PNGs. Same data, same render, no random diff noise.
            rng = np.random.default_rng(42)
            idx = rng.choice(len(pts), subsample, replace=False)
            pts_r = pts[idx]
            cols_r = np.asarray(pcd.colors)[idx] if pcd.has_colors() else None
        else:
            pts_r = pts
            cols_r = np.asarray(pcd.colors) if pcd.has_colors() else None

        fig = plt.figure(figsize=(10, 8), facecolor="#111111")
        ax = fig.add_subplot(111, projection="3d", facecolor="#111111")

        if cols_r is not None:
            ax.scatter(
                pts_r[:, 0],
                pts_r[:, 1],
                pts_r[:, 2],
                c=cols_r,
                s=1.2,
                linewidths=0,
                alpha=0.9,
            )
        else:
            ax.scatter(
                pts_r[:, 0],
                pts_r[:, 1],
                pts_r[:, 2],
                c="#00ffcc",
                s=1.2,
                linewidths=0,
                alpha=0.6,
            )

        full_count = len(pts)
        if title:
            ax.set_title(
                f"{title}\n{full_count:,} pts total - {len(pts_r):,} shown",
                fontsize=11,
                color="white",
                pad=8,
            )

        # Kill the axes/grid completely. For this assignment render, the geometry is
        # the subject; the grey grid is just visual noise.
        ax.axis("off")
        ax.view_init(elev=25, azim=-45)

        # Preserve the actual spatial proportions instead of letting matplotlib
        # squash the cloud into a weird default 3D box.
        extents = np.ptp(pts_r, axis=0)
        extents[extents == 0] = 1.0
        ax.set_box_aspect(extents, zoom=1.25)

        out_path = os.path.join(self._output_dir, filename)
        plt.savefig(
            out_path,
            dpi=200,
            bbox_inches="tight",
            facecolor=fig.get_facecolor(),
        )

        # Critical memory leak fix:
        # If u don't close the figure, matplotlib keeps it alive in RAM.
        # Multiple renders = multiple hidden figures = bad time.
        plt.close(fig)

        logger.info("Saved cinema render -> %s", out_path)
        return os.path.abspath(out_path)

    def save_normals_render(
        self,
        pcd: o3d.geometry.PointCloud,
        filename: str,
        subsample: int = 50000,
    ) -> str:
        """Render normals by mapping vector direction into RGB colour.

        Normal vectors live in [-1, 1]. RGB needs [0, 1].
        If I map x/y/z directly, negative values get clipped to 0 and half the
        cloud goes weirdly dark.

        Fix: np.abs(normals).
        - X-facing surfaces show up red.
        - Y-facing surfaces show up green.
        - Z-facing surfaces show up blue.
        You can literally see the geometry orientation in colour. Heat fr.
        """
        if not pcd.has_normals():
            raise ValueError("Run NormalEstimator.estimate() before saving normals.")

        normals = np.asarray(pcd.normals)
        rgb = np.abs(normals)  # maps valid normal directions into valid RGB

        colored = o3d.geometry.PointCloud(pcd)
        colored.colors = o3d.utility.Vector3dVector(rgb)

        return self.save_render(
            colored,
            filename,
            title="Surface normals - R=|Nx|  G=|Ny|  B=|Nz|",
            subsample=subsample,
        )
