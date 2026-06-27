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
    Orchestrates the whole point cloud processing flow.

    OOP brain dump, bc this is the part where spaghetti can start cooking fast:

    I'm using COMPOSITION over inheritance here.
    Pipeline should not *be* a Loader, Preprocessor, NormalEstimator, etc.
    That would make it a giant God Object, and then every tiny change would touch
    the same monster class. Huge L.

    Instead, Pipeline *has* those tools. It is basically the coordinator / CEO:
    it knows the order of the work, but it does not pretend to know every detail
    of how each specialist does its job.

    Why this is a W:
    1. Testing is cleaner. I can test Loader without caring about DBSCAN.
    2. Swapping is easier. If I change DBSCAN later, Pipeline barely cares.
    3. The data flow is obvious: load -> preprocess -> normals -> clusters -> renders.

    Tiny self-check:
    If I cannot explain a class in one sentence, that class is probably doing too
    much. Pipeline's one sentence is simple: "run the stages in order and keep
    the outputs."
    """

    def __init__(self, config: Config | None = None) -> None:
        if config is None:
            config = Config()
        self._config = config

        # Instantiating the decoupled squad.
        # Each class owns one skill. Pipeline just lines them up.
        self._loader = Loader()
        self._preprocessor = Preprocessor(config)
        self._normal_estimator = NormalEstimator(config)
        self._cluster_extractor = ClusterExtractor(config)
        self._visualizer = Visualizer(config.output_dir)
        self._subsample = config.render_subsample

    def run(self) -> dict[str, Any]:
        """Run every stage and return the important intermediate outputs.

        Returning a dict is intentional. It keeps receipts.
        If I want to debug later, I do not want to rerun the whole Eagle scan just
        to inspect the cleaned cloud or colored clusters. The pipeline hands those
        objects back so future code/tests can reuse them.
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
        logger.info("STAGE 2 - Preprocessing: voxel downsample + SOR")
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
        logger.info("STAGE 3 - Surface normal estimation: PCA time")
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
        # Not every tiny fragment deserves its own PNG, but the big parts do.
        for i, cluster in enumerate(clusters[:5]):
            self._visualizer.save_render(
                cluster,
                f"05_cluster_{i:02d}.png",
                f"Cluster {i} - {len(cluster.points):,} points",
                self._subsample,
            )

        # ── Summary ───────────────────────────────────────────────────────────
        logger.info("=" * 55)
        logger.info("PIPELINE COMPLETE. We cooked.")
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
