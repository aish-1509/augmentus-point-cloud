"""Point cloud processing pipeline entry point.

Run from the project root:

    python -m src.pipeline
"""

from __future__ import annotations

import logging
import os
from typing import Any

import numpy as np
import open3d as o3d

from src.cluster_extractor import ClusterExtractor
from src.config import Config
from src.loader import Loader
from src.normal_estimator import NormalEstimator
from src.preprocessor import Preprocessor
from src.visualizer import Visualizer

logging.basicConfig(
    level=logging.INFO,
    format="[%(levelname)s] %(name)s - %(message)s",
)
logger = logging.getLogger(__name__)


class Pipeline:
    """Orchestrates the complete point cloud processing lifecycle.

    Uses composition rather than inheritance: each processing stage is an
    independent class that owns its own logic. Pipeline only coordinates the
    handoff order and knows nothing about how each stage works internally.

    This keeps every stage independently testable — swapping out the clustering
    algorithm, changing the renderer, or adding a new preprocessing step should
    not require touching the other classes.

    OOP pattern in use here:
    Composition — Pipeline holds one instance of each component (Loader,
    Preprocessor, NormalEstimator, ClusterExtractor, Visualizer). None of those
    classes inherit from each other; they're separate tools coordinated by a
    single orchestrator.
    """

    def __init__(self, config: Config | None = None) -> None:
        if config is None:
            config = Config()
        self._config = config

        # Dependency injection: each component receives Config at construction.
        # Changing a parameter in Config propagates automatically to every stage
        # without touching any of the component classes.
        self._loader = Loader()
        self._preprocessor = Preprocessor(config)
        self._normal_estimator = NormalEstimator(config)
        self._cluster_extractor = ClusterExtractor(config)
        self._visualizer = Visualizer(config.output_dir)
        self._subsample = config.render_subsample

    def run(self) -> dict[str, Any]:
        """Execute all pipeline stages and return intermediate results.

        Returning results as a dict keeps every intermediate artifact accessible
        for debugging, testing, and extension — callers can inspect the raw cloud,
        the cropped cloud, normals, or individual clusters without rerunning the
        full pipeline.
        """
        results: dict[str, Any] = {}

        # ── Stage 1: Load ─────────────────────────────────────────────────────
        logger.info("=" * 55)
        logger.info("STAGE 1 — Load Eagle dataset")
        raw = self._loader.load_eagle()
        self._visualizer.save_render(
            raw,
            "01_raw.png",
            "Stage 1: Raw Eagle point cloud",
            self._subsample,
        )
        results["raw"] = raw

        # ── Stage 2: Downsample ───────────────────────────────────────────────
        logger.info("=" * 55)
        logger.info("STAGE 2 — Voxel downsample")
        downsampled = self._preprocessor.downsample(raw)
        self._visualizer.save_render(
            downsampled,
            "02_downsampled.png",
            f"Stage 2: Voxel downsample ({self._config.voxel_size} m)",
            self._subsample,
        )
        results["downsampled"] = downsampled

        # ── Stage 3: Filter + crop ────────────────────────────────────────────
        # SOR removes floating ghost points first. Then crop_to_roi creates the
        # explicit cropped point cloud requested by the assignment. Normals and
        # clustering only see this cropped cloud.
        logger.info("=" * 55)
        logger.info("STAGE 3 — Statistical filtering + cropped ROI")
        filtered = self._preprocessor.remove_outliers(downsampled)
        cropped = self._preprocessor.crop_to_roi(filtered)
        self._visualizer.save_render(
            cropped,
            "03_cropped.png",
            (
                "Stage 3: SOR + cropped ROI "
                f"(padding = {self._config.crop_padding} m)"
            ),
            self._subsample,
        )
        results["filtered"] = filtered
        results["cropped"] = cropped

        # ── Stage 4: Normal estimation ────────────────────────────────────────
        # Normals are estimated on the cropped + cleaned cloud — this is the
        # "cropped point cloud" referenced in the assignment spec.
        logger.info("=" * 55)
        logger.info("STAGE 4 — Surface normal estimation on cropped + cleaned cloud")
        with_normals = self._normal_estimator.estimate(cropped)
        self._visualizer.save_normals_render(
            with_normals,
            "04_normals.png",
            self._subsample,
        )
        results["with_normals"] = with_normals

        # ── Stage 5: Euclidean clustering ─────────────────────────────────────
        logger.info("=" * 55)
        logger.info("STAGE 5 — Euclidean clustering with KD-tree radius expansion")
        cluster_indices = self._cluster_extractor.extract_euclidean_clusters(with_normals)
        labels = self._cluster_extractor.labels_from_clusters(
            len(with_normals.points),
            cluster_indices,
        )
        clusters = self._cluster_extractor.get_cluster_point_clouds(
            with_normals,
            cluster_indices,
        )
        colored = self._cluster_extractor.get_colored_cloud(with_normals, labels=labels)

        self._visualizer.save_render(
            colored,
            "05_clusters_colored.png",
            (
                f"Stage 5: {len(clusters)} Euclidean clusters "
                f"(tolerance={self._config.clustering_eps} m, "
                f"min_size={self._config.clustering_min_points})"
            ),
            self._subsample,
        )

        # Multi-angle overview of the colored cluster scene.
        # A single isometric view can hide clusters stacked in depth. Four angles
        # together make it impossible to miss any spatial separation between
        # segments — iso shows 3D shape, front/side confirm separation, top shows
        # the footprint layout.
        # Saves: 05_clusters_colored_iso.png, 05_clusters_colored_front.png,
        #        05_clusters_colored_side.png, 05_clusters_colored_top.png
        self._visualizer.save_multi_angle_render(
            colored,
            "05_clusters_colored.png",
            (
                f"Stage 5: {len(clusters)} Euclidean clusters"
            ),
            self._subsample,
        )
        summary = self._cluster_extractor.build_cluster_summary(
            clusters,
            len(with_normals.points),
        )
        summary_path = self._cluster_extractor.save_cluster_summary(
            summary,
            os.path.join(self._config.output_dir, "cluster_summary.json"),
        )

        results["cluster_indices"] = cluster_indices
        results["cluster_labels"] = labels
        results["clusters"] = clusters
        results["colored"] = colored
        results["cluster_summary"] = summary
        results["cluster_summary_path"] = summary_path

        # ── Per-cluster individual renders ────────────────────────────────────
        # Each cluster is painted with its dedicated vibrant color (same palette
        # used in the overview) and saved as its own PNG. This is important
        # because small clusters are invisible slivers in the combined view — a
        # solo render at the right point size lets you actually inspect them.
        max_save = min(len(clusters), self._config.max_clusters_to_save)
        logger.info(
            "Saving %d individual cluster renders...",
            max_save,
        )
        for i, cluster in enumerate(clusters[:max_save]):
            color = self._cluster_extractor.get_cluster_color(i)
            n_pts = len(cluster.points)

            # Paint the extracted cluster with its assigned vibrant color.
            # np.tile repeats the (3,) color vector n_pts times to build the
            # (N, 3) colors matrix that Open3D expects.
            cluster_colored = o3d.geometry.PointCloud(cluster)
            cluster_colored.colors = o3d.utility.Vector3dVector(
                np.tile(color, (n_pts, 1))
            )
            self._visualizer.save_render(
                cluster_colored,
                f"clusters/cluster_{i:02d}.png",
                f"Cluster {i:02d} — {n_pts:,} pts",
                self._subsample,
            )

        # ── Summary ───────────────────────────────────────────────────────────
        logger.info("=" * 55)
        logger.info("PIPELINE COMPLETE.")
        logger.info(
            "  %s raw -> %s downsampled -> %s cropped -> %d clusters",
            f"{len(raw.points):,}",
            f"{len(downsampled.points):,}",
            f"{len(with_normals.points):,}",
            len(clusters),
        )
        logger.info("=" * 55)

        return results


if __name__ == "__main__":
    Pipeline().run()
