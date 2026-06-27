"""Point cloud processing pipeline entry point.

Run from the project root:

python -m src.pipeline
"""

from __future__ import annotations

import logging
from typing import Any

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

    The pipeline uses composition rather than inheritance. It owns one instance
    of each processing component and coordinates their handoff order, while each
    component keeps responsibility for its own implementation details.

    This keeps the stages independently testable and makes the pipeline easier to
    extend: changing clustering, rendering, or normal estimation should not
    require rewriting the orchestration layer.
    """

    def __init__(self, config: Config | None = None) -> None:
        if config is None:
            config = Config()
        self._config = config

        # Each tool owns its own logic. Pipeline only coordinates the stage order.
        self._loader = Loader()
        self._preprocessor = Preprocessor(config)
        self._normal_estimator = NormalEstimator(config)
        self._cluster_extractor = ClusterExtractor(config)
        self._visualizer = Visualizer(config.output_dir)
        self._subsample = config.render_subsample

    def run(self) -> dict[str, Any]:
        """Run every stage and return the intermediate outputs.

        Returning the dict keeps intermediate results available for debugging and
        extension. Downstream code can inspect clusters, normals, or renders
        without rerunning the full point cloud pipeline.
        """
        results: dict[str, Any] = {}

        # ── Stage 1: Load ─────────────────────────────────────────────────────
        logger.info("=" * 55)
        logger.info("STAGE 1 - Pulling the Eagle dataset")
        raw = self._loader.load_eagle()
        self._visualizer.save_render(
            raw,
            "01_raw.png",
            "Stage 1: Raw Eagle point cloud",
            self._subsample,
        )
        results["raw"] = raw

        # ── Stage 2: Preprocess ───────────────────────────────────────────────
        logger.info("=" * 55)
        logger.info("STAGE 2 - Preprocessing (voxel downsample + SOR)")
        preprocessed = self._preprocessor.preprocess(raw)
        self._visualizer.save_render(
            preprocessed,
            "02_preprocessed.png",
            f"Stage 2: Voxel downsample ({self._config.voxel_size}m) + SOR",
            self._subsample,
        )
        results["preprocessed"] = preprocessed

        # ── Stage 3: Normal estimation ────────────────────────────────────────
        logger.info("=" * 55)
        logger.info("STAGE 3 - Surface normal estimation")
        with_normals = self._normal_estimator.estimate(preprocessed)
        self._visualizer.save_normals_render(
            with_normals,
            "03_normals.png",
            self._subsample,
        )
        results["with_normals"] = with_normals

        # ── Stage 4: Euclidean clustering ─────────────────────────────────────
        logger.info("=" * 55)
        logger.info("STAGE 4 - Euclidean clustering via DBSCAN")
        clusters = self._cluster_extractor.extract(with_normals)
        colored = self._cluster_extractor.get_colored_cloud(with_normals)

        self._visualizer.save_render(
            colored,
            "04_clusters.png",
            (
                f"Stage 4: {len(clusters)} clusters "
                f"(eps={self._config.clustering_eps}, "
                f"min_pts={self._config.clustering_min_points})"
            ),
            self._subsample,
        )
        results["clusters"] = clusters
        results["colored"] = colored

        # Save the top 5 biggest clusters separately.
        # Not every tiny floating fragment deserves its own PNG, but the main
        # bodies do. This makes the README visuals easier to inspect later.
        for i, cluster in enumerate(clusters[:5]):
            self._visualizer.save_render(
                cluster,
                f"05_cluster_{i:02d}.png",
                f"Cluster {i} - {len(cluster.points):,} points",
                self._subsample,
            )

        # ── Summary ───────────────────────────────────────────────────────────
        logger.info("=" * 55)
        logger.info("PIPELINE COMPLETE.")
        logger.info(
            "  %s raw pts -> %s clean pts -> %d clusters extracted",
            f"{len(raw.points):,}",
            f"{len(with_normals.points):,}",
            len(clusters),
        )
        logger.info("=" * 55)

        return results


if __name__ == "__main__":
    Pipeline().run()
