import logging
import os

import numpy as np
import open3d as o3d

# ok wait if I run o3d.visualization.draw_geometries on github actions it's gonna literally crash
# bc there's no display manager. Major L.
# Imma pivot to matplotlib to save it as a flat png.
import matplotlib.pyplot as plt
import matplotlib

# yo wait I need to set the backend BEFORE pyplot or it still tries to open a window
matplotlib.use("Agg")

logger = logging.getLogger(__name__)


class Visualizer:
    """
    Tbh trying to render 1.5M points with matplotlib scatter is gonna nuke my ram.
    Like it's strictly cpu rendered, no GPU accel.
    I gotta downsample it JUST for the render or this script will take 5 years to run.
    """

    def __init__(self, output_dir: str = "docs/renders") -> None:
        self._output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)  # exist_ok=True is a massive W so it doesn't crash if folder exists

    @property
    def output_dir(self) -> str:
        return self._output_dir

    def save_render(
        self,
        pcd: o3d.geometry.PointCloud,
        filename: str,
        title: str = "",
        subsample: int = 8000,
    ) -> str:
        pts = np.asarray(pcd.points)
        if len(pts) == 0:
            return ""

        # ngl 8000 points is enough to see the shape. Any more and matplotlib starts crying.
        if len(pts) > subsample:
            idx = np.random.choice(len(pts), subsample, replace=False)
            pts_r = pts[idx]
            cols_r = np.asarray(pcd.colors)[idx] if pcd.has_colors() else None
        else:
            pts_r = pts
            cols_r = np.asarray(pcd.colors) if pcd.has_colors() else None

        fig = plt.figure(figsize=(9, 7))
        ax = fig.add_subplot(111, projection="3d")

        if cols_r is not None:
            ax.scatter(pts_r[:, 0], pts_r[:, 1], pts_r[:, 2], c=cols_r, s=0.5, linewidths=0)
        else:
            ax.scatter(
                pts_r[:, 0],
                pts_r[:, 1],
                pts_r[:, 2],
                c="steelblue",
                s=0.5,
                linewidths=0,
                alpha=0.6,
            )

        # bruh if I don't set this the eagle looks like a cube lol. Need to preserve actual proportions.
        ax.set_box_aspect([1, 1, 1])

        out_path = os.path.join(self._output_dir, filename)
        plt.savefig(out_path, dpi=150, bbox_inches="tight", facecolor="white")

        # OMFG I FORGOT PLT.CLOSE() BEFORE AND IT LEAKED MEMORY SO BAD.
        # MUST DO THIS.
        plt.close(fig)

        return os.path.abspath(out_path)

    def save_normals_render(
        self,
        pcd: o3d.geometry.PointCloud,
        filename: str,
        subsample: int = 8000,
    ) -> str:
        # How do I even render normals? They are vectors.
        # Wait, if I just map x,y,z to r,g,b that works right?
        # nah wait, vectors go from -1 to 1. RGB stops at 0.
        # If I don't take the absolute value, half the cloud will just be pitch black.
        normals = np.asarray(pcd.normals)
        rgb = np.abs(normals)  # W fix right here

        colored = o3d.geometry.PointCloud(pcd)
        colored.colors = o3d.utility.Vector3dVector(rgb)

        return self.save_render(colored, filename, title="Normals rn", subsample=subsample)
