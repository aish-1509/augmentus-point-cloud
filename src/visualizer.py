import logging
import os

import matplotlib

# CRITICAL: Set Agg before importing pyplot, otherwise CI/Docker/headless environments will crash.
# Open3D's native viewer requires a display manager (like X11). GitHub Actions has none.
# The Agg backend renders straight to a file without trying to open a GUI, which is excellent for portability.
matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d

logger = logging.getLogger(__name__)


class Visualizer:
    """
    Saves point clouds to PNGs so we can visually inspect the pipeline's output.

    Design Notes:
    Matplotlib's 3D scatter is strictly CPU-bound and lacks a true hardware Z-buffer.
    If we pass 1.5M points into it with transparency or large point sizes, the Painter's
    Algorithm simply draws background points over foreground points. This turns dense,
    intricate geometry into a blurry output, which isn't ideal.

    The Eagle dataset is stored Y-up (the bounding box Y range at ~8.6m is the tallest
    axis — the wing tip to plinth base span). Matplotlib is Z-up. Dropping Y-up data
    into a Z-up renderer lays the eagle on its side.

    The solution: We apply Rx(-90°) to the render-only arrays before plotting. This maps
    the scanned Y axis into matplotlib's Z-up frame so the eagle stands upright without
    touching the pipeline cloud.
    We also lock a uniform 3D bounding box to prevent matplotlib from stretching any axis.
    """

    def __init__(self, output_dir: str = "docs/renders") -> None:
        self._output_dir = output_dir
        # exist_ok=True ensures the script doesn't fail on subsequent runs if the directory already exists.
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
        """
        Saves a 3D scatter view of the cloud as a high-res PNG.

        The original Open3D point cloud should stay untouched just because I need
        a nicer PNG. All render-specific matrix math is applied to a `.copy()` of
        the numpy array, so the visual orientation fix cannot leak back into the
        processing pipeline.
        """
        pts = np.asarray(pcd.points)
        if len(pts) == 0:
            logger.warning("Cloud is empty. Skipping %s", filename)
            return ""

        # Subsample for rendering ONLY. Never mutate the real pipeline data.
        if len(pts) > subsample:
            # Fixed seed = reproducible PNGs. Same data, same render, preventing visual inconsistencies between runs.
            rng = np.random.default_rng(42)
            idx = rng.choice(len(pts), subsample, replace=False)
            pts_r = pts[idx].copy()
            cols_r = np.asarray(pcd.colors)[idx] if pcd.has_colors() else None
        else:
            pts_r = pts.copy()
            cols_r = np.asarray(pcd.colors) if pcd.has_colors() else None

        # ── COORDINATE FRAME CORRECTION ────────────────────────────────────────
        # The Eagle dataset is Y-up (measured: Y range ≈ 8.6m, the tallest axis).
        # Matplotlib is Z-up. Camera orbiting cannot fix this; the data itself must
        # be re-framed before scatter() receives it.
        #
        # Rx(-90°) maps:  X -> X,  Y -> -Z,  Z -> +Y
        # This stands the eagle upright in matplotlib's Z-up coordinate system.
        # All operations are applied to a copy — the pipeline cloud is never mutated.
        theta_x = np.radians(-90.0)
        rx = np.array([
            [1.0, 0.0,              0.0             ],
            [0.0, np.cos(theta_x), -np.sin(theta_x) ],
            [0.0, np.sin(theta_x),  np.cos(theta_x) ],
        ])
        pts_r = pts_r @ rx.T
        # ───────────────────────────────────────────────────────────────────────

        # Dark background for better contrast
        fig = plt.figure(figsize=(12, 10), facecolor="#050505")
        ax = fig.add_subplot(111, projection="3d", facecolor="#050505")
        ax.set_proj_type("ortho")

        # Scale point size inversely with density. Dense clouds (100K+) need microscopic
        # points so the silhouette emerges from density alone. Sparse clusters (< 1K)
        # need larger points or they become invisible at 300dpi on a dark background.
        n = len(pts_r)
        if n >= 100_000:
            pt_size = 0.05
        elif n >= 10_000:
            pt_size = 0.3
        elif n >= 1_000:
            pt_size = 1.5
        else:
            pt_size = 10.0

        if cols_r is not None:
            ax.scatter(
                pts_r[:, 0], pts_r[:, 1], pts_r[:, 2],
                c=cols_r, s=pt_size, linewidths=0, edgecolors="none", alpha=1.0, depthshade=False,
            )
        else:
            ax.scatter(
                pts_r[:, 0], pts_r[:, 1], pts_r[:, 2],
                c="#00ffcc", s=pt_size, linewidths=0, edgecolors="none", alpha=1.0, depthshade=False,
            )

        full_count = len(pts)
        if title:
            ax.set_title(
                f"{title}\n{full_count:,} pts total - {len(pts_r):,} shown",
                fontsize=12,
                color="white",
                pad=10,
            )

        # ── ANTI-WARPING BOUNDING BOX MATH ─────────────────────────────────────
        # Matplotlib auto-scales X/Y/Z independently. If the object is wider than it is 
        # tall, Matplotlib stretches the Z-axis to form a cube. 
        # To fix this, we find the absolute maximum physical range and force all three 
        # axes to use it, maintaining a 1:1 spatial ratio.
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
        # ───────────────────────────────────────────────────────────────────────

        # Clean inspection render: no grid, no panes, no numbers. Just geometry.
        ax.axis("off")

        # Low 3/4 front view after the render-only rotation. This keeps the
        # plinth grounded while still showing the wing/body geometry.
        ax.view_init(elev=15, azim=30)

        out_path = os.path.join(self._output_dir, filename)
        plt.savefig(
            out_path,
            dpi=300, # High resolution output
            bbox_inches="tight",
            facecolor=fig.get_facecolor(),
        )

        # Prevent memory leaks:
        # If the figure isn't closed, matplotlib keeps it alive in RAM.
        # Multiple renders can lead to an out-of-memory crash.
        plt.close(fig)

        logger.info("Saved upright full-density high-res render -> %s", out_path)
        return os.path.abspath(out_path)

    def save_normals_render(
        self,
        pcd: o3d.geometry.PointCloud,
        filename: str,
        subsample: int = 400000,
    ) -> str:
        """
        Renders normals by mapping vector directions strictly into RGB color space.

        Mathematical context: Normal vectors live in [-1, 1], but RGB requires [0, 1].
        If we map X/Y/Z directly to R/G/B, negative values are clipped to 0, which 
        turns half the cloud pitch black.

        The solution: np.abs(normals).
        - X-facing surfaces appear Red.
        - Y-facing surfaces appear Green.
        - Z-facing surfaces appear Blue.
        This allows us to visually interpret the shape's topology through color.
        """
        if not pcd.has_normals():
            raise ValueError("NormalEstimator.estimate() must be run before saving normals.")

        normals = np.asarray(pcd.normals)
        rgb = np.abs(normals)  # Map to valid RGB range

        colored = o3d.geometry.PointCloud(pcd)
        colored.colors = o3d.utility.Vector3dVector(rgb)

        return self.save_render(
            colored,
            filename,
            title="Surface normals - R=|Nx|  G=|Ny|  B=|Nz|",
            subsample=subsample,
        )
