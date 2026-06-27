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

    The early render passes proved the pipeline worked, but the visuals were still
    not honest enough. Matplotlib can quietly warp a 3D object if the X/Y/Z limits
    are scaled independently, and large transparent points turn dense geometry
    into blurry sludge.

    Current fix: rotate the render-only numpy points from the scan's native
    sideways frame into a Z-up plotting frame, lock one uniform 3D bounding box,
    render the cleaned Eagle cloud without dropping points, and use tiny opaque
    points so the silhouette comes from real scan density.
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
        subsample: int = 400000,
    ) -> str:
        """Save a 3D scatter view of the cloud as a PNG.

        Points are randomly dropped if the cloud is larger than `subsample`.
        That is render-only; the actual Open3D point cloud is not mutated.

        Matplotlib's 3D scatter is not a real hardware renderer. If points are
        large and semi-transparent, far-side geometry bleeds through near-side
        geometry. Here I keep the points microscopic and opaque, then render all
        316k cleaned Eagle points so the silhouette comes from real data density.
        """
        pts = np.asarray(pcd.points)
        if len(pts) == 0:
            logger.warning("Cloud is empty. Skipping %s", filename)
            return ""

        # Subsample for rendering ONLY. Never mutate the real pipeline data.
        if len(pts) > subsample:
            # Fixed seed = reproducible PNGs. Same data, same render, no random diff noise.
            rng = np.random.default_rng(42)
            idx = rng.choice(len(pts), subsample, replace=False)
            pts_r = pts[idx].copy()
            cols_r = np.asarray(pcd.colors)[idx] if pcd.has_colors() else None
        else:
            pts_r = pts.copy()
            cols_r = np.asarray(pcd.colors) if pcd.has_colors() else None

        # The Eagle scan is not stored in the same "upright" frame that matplotlib
        # expects. Camera orbiting only changes where I look from; it cannot stand a
        # sideways object up. This render-only rotation maps the scan's up direction
        # into matplotlib's Z-up world without touching the actual pipeline data.
        theta_x = np.radians(90.0)
        rotation_x = np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, np.cos(theta_x), -np.sin(theta_x)],
                [0.0, np.sin(theta_x), np.cos(theta_x)],
            ]
        )
        pts_r = pts_r @ rotation_x.T

        fig = plt.figure(figsize=(12, 10), facecolor="#050505")
        ax = fig.add_subplot(111, projection="3d", facecolor="#050505")
        ax.set_proj_type("ortho")

        if cols_r is not None:
            ax.scatter(
                pts_r[:, 0],
                pts_r[:, 1],
                pts_r[:, 2],
                c=cols_r,
                s=0.05,
                linewidths=0,
                edgecolors="none",
                alpha=1.0,
                depthshade=False,
            )
        else:
            ax.scatter(
                pts_r[:, 0],
                pts_r[:, 1],
                pts_r[:, 2],
                c="#00ffcc",
                s=0.05,
                linewidths=0,
                edgecolors="none",
                alpha=1.0,
                depthshade=False,
            )

        full_count = len(pts)
        if title:
            ax.set_title(
                f"{title}\n{full_count:,} pts total - {len(pts_r):,} shown",
                fontsize=12,
                color="white",
                pad=10,
            )

        # Matplotlib auto-scales X/Y/Z independently. That can stretch the eagle
        # into a blob even when the actual point coordinates are correct. So I take
        # the largest physical range and use it for all three axes. Same scale in
        # every direction, no fake warping.
        mins = pts_r.min(axis=0)
        maxs = pts_r.max(axis=0)
        centers = (mins + maxs) * 0.5
        half_range = float(np.max(maxs - mins) * 0.5)
        if half_range == 0.0:
            half_range = 1.0

        ax.set_xlim(centers[0] - half_range, centers[0] + half_range)
        ax.set_ylim(centers[1] - half_range, centers[1] + half_range)
        ax.set_zlim(centers[2] - half_range, centers[2] + half_range)
        ax.set_box_aspect([1, 1, 1], zoom=1.05)

        # Clean inspection render: no grid, no panes, no tick labels. Just geometry.
        ax.axis("off")

        # Normal low 3/4 view now that the point cloud itself has been stood up.
        ax.view_init(elev=20, azim=-45)

        out_path = os.path.join(self._output_dir, filename)
        plt.savefig(
            out_path,
            dpi=300,
            bbox_inches="tight",
            facecolor=fig.get_facecolor(),
        )

        # Critical memory leak fix:
        # If u don't close the figure, matplotlib keeps it alive in RAM.
        # Multiple renders = multiple hidden figures = bad time.
        plt.close(fig)

        logger.info("Saved upright full-density render -> %s", out_path)
        return os.path.abspath(out_path)

    def save_normals_render(
        self,
        pcd: o3d.geometry.PointCloud,
        filename: str,
        subsample: int = 400000,
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
