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

logger = logging.getLogger(__name__)


class Pipeline:
    """
    Bro wait. I was about to just inherit all the tool classes into Pipeline
    like `class Pipeline(Loader, Preprocessor...)` but that is literally how
    you create a God Object. Massive L. The technical debt would be insane.

    If I do that, the whole codebase becomes spaghetti. I gotta use Composition
    instead. Pipeline shouldn't BE a loader, it should just HAVE a loader.
    Has-a relationship > Is-a relationship tbh.
    """

    def __init__(self, config: Config | None = None) -> None:
        if config is None:
            config = Config()
        self._config = config

        # setting up the squad here.
        self._loader = Loader()
        self._preprocessor = Preprocessor(config)
        self._normal_estimator = NormalEstimator(config)
        self._cluster_extractor = ClusterExtractor(config)
        self._visualizer = Visualizer(config.output_dir)
        self._subsample = config.render_subsample

    def run(self) -> dict[str, Any]:
        # I should return a dict of all the outputs so I don't have to rerun this
        # heavy ahh 1.5M point cloud every time I want to test a downstream function.
        results: dict[str, Any] = {}

        # Stage 1
        logger.info("pulling eagle...")
        raw = self._loader.load_eagle()
        self._visualizer.save_render(raw, "01_raw.png", "Raw Eagle", self._subsample)
        results["raw"] = raw

        # Stage 2
        logger.info("cleaning...")
        preprocessed = self._preprocessor.preprocess(raw)
        self._visualizer.save_render(
            preprocessed,
            "02_preprocessed.png",
            "Cleaned",
            self._subsample,
        )
        results["preprocessed"] = preprocessed

        # gonna finish the rest in the next commit, pushing this arch setup first.
        return results


if __name__ == "__main__":
    Pipeline().run()
