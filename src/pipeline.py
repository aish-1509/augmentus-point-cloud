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
    """
    Orchestrates the entire point cloud processing lifecycle.

    OOP brain dump, bc this is where architecture either gets clean or turns into
    cursed spaghetti:

    I'm using COMPOSITION over inheritance here.
    Pipeline should not inherit from Loader, Preprocessor, NormalEstimator, etc.
    If it did, it would become a God Object. Tight coupling = negative aura fr.
    One tiny DBSCAN change should not make the whole pipeline class feel cooked.

    By using composition, Pipeline just *has* the tools. It acts like the
    coordinator. It does not pretend to know the PCA math inside NormalEstimator;
    it just knows the next correct handoff.

    Why this design is a W:
    1. Independent testability: I can test one component without the others crying.
    2. Hot-swapping: if DBSCAN changes later, Pipeline barely flinches.
    3. SRP: Pipeline does one thing - dictates the flow and keeps the receipts.

    Tiny self-check:
    If a class cannot be explained in one clean sentence, it's probably doing too
    much. Pipeline's sentence is: run the stages in order and return the outputs.
    """

    def __init__(self, config: Config | None = None) -> None:
        if config is None:
            config = Config()
        self._config = config

        # Instantiating the decoupled squad.
        # Each tool owns its own logic. Pipeline just lines them up. W design.
        self._loader = Loader()
        self._preprocessor = Preprocessor(config)
        self._normal_estimator = NormalEstimator(config)
        self._cluster_extractor = ClusterExtractor(config)
        self._visualizer = Visualizer(config.output_dir)
        self._subsample = config.render_subsample

    def run(self) -> dict[str, Any]:
        """Run every stage and return the intermediate outputs.

        Returning the dict is a sweaty but useful move for downstream debugging.
        If I want to inspect clusters later, I do NOT want to rerun a huge point
        cloud through O(N log N) neighbour searches just to get the same objects.
        This keeps the receipts fr.
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
        logger.info("STAGE 2 - Preprocessing (voxel downsample + SOR cooking)")
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
        logger.info("STAGE 3 - Surface normal estimation (PCA time)")
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
        logger.info("PIPELINE COMPLETE. WE COOKED.")
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
