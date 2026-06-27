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

    Additionally, the hardware scanner used for the Eagle dataset appears to have been 
    calibrated upside down (Z-axis inverted). Matplotlib is strictly Z-up. Dropping 
    inverted data into a Z-up environment causes the object to render completely upside down. 
    
    The solution: We apply a strict linear algebra affine transformation to the render-only 
    arrays. We flip the object upright, lock a uniform 3D bounding box to prevent Matplotlib 
    from stretching it, and render the cleaned Eagle cloud at max density with opaque, 
    microscopic points to preserve data fidelity.
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

        My experience writing generative AI pipelines at Panasonic reinforced the importance 
        of state management. Mutating the raw pipeline point cloud just to format a plot 
        can cause unexpected issues in downstream tasks. Therefore, all matrix math here 
        is applied strictly to a `.copy()` of the numpy array to ensure state immutability.
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

        # ── LINEAR ALGEBRA POSTURE FIX ─────────────────────────────────────────
        # Camera orbiting only changes the viewing angle; it cannot fix intrinsic 
        # object posture. We need to physically rotate the data for the render.
        # 
        # 1. Rx(180): 180-degree rotation around X to fix the inverted Z-axis.
        theta_x = np.radians(180.0)
        rx = np.array([
            [1.0, 0.0, 0.0],
            [0.0, np.cos(theta_x), -np.sin(theta_x)],
            [0.0, np.sin(theta_x), np.cos(theta_x)],
        ])
        
        # 2. Rz(90): 90-degree yaw around Z to rotate the eagle into a 3/4 profile.
        theta_z = np.radians(90.0)
        rz = np.array([
            [np.cos(theta_z), -np.sin(theta_z), 0.0],
            [np.sin(theta_z), np.cos(theta_z), 0.0],
            [0.0, 0.0, 1.0],
        ])
        
        # Matrix multiply the points: pts_r @ (Rz @ Rx)^T
        pts_r = pts_r @ (rz @ rx).T
        # ───────────────────────────────────────────────────────────────────────

        # Dark background for better contrast
        fig = plt.figure(figsize=(12, 10), facecolor="#050505")
        ax = fig.add_subplot(111, projection="3d", facecolor="#050505")
        ax.set_proj_type("ortho")

        # Microscopic point sizes (s=0.05), zero edges, zero depth shade.
        # We want the silhouette to emerge purely from the density of the points.
        if cols_r is not None:
            ax.scatter(
                pts_r[:, 0], pts_r[:, 1], pts_r[:, 2],
                c=cols_r, s=0.05, linewidths=0, edgecolors="none", alpha=1.0, depthshade=False,
            )
        else:
            ax.scatter(
                pts_r[:, 0], pts_r[:, 1], pts_r[:, 2],
                c="#00ffcc", s=0.05, linewidths=0, edgecolors="none", alpha=1.0, depthshade=False,
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

        # Straight-on camera (azim=0) slightly above eye-level (elev=20) 
        # since the render-only yaw handles the orientation.
        ax.view_init(elev=20, azim=0)

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