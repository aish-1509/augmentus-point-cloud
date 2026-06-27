import logging

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
    Ok wait. Before I just write a 500-line god-script that calls everything in order,
    I need to think about OOP cuz Augmentus literally said OOP is important.

    If I make a massive class that INHERITS from Loader, Preprocessor, etc...
    that's called a God Object. It's a massive L. If one thing breaks, the whole
    class is cooked.

    Instead, I need to use COMPOSITION. Pipeline shouldn't BE a loader, it should
    HAVE a loader.
    Basically, Pipeline is just the CEO. It doesn't know HOW to downsample, it
    just tells the Preprocessor to go do it and return the result. CEO mindset fr.
    """

    def __init__(self, config: Config = None) -> None:
        if config is None:
            config = Config()
        self._config = config

        # Composition check: Instantiating the squad here.
        # They are fully decoupled. W architecture.
        self._loader = Loader()
        self._preprocessor = Preprocessor(config)
        self._normal_estimator = NormalEstimator(config)
        self._cluster_extractor = ClusterExtractor(config)
        self._visualizer = Visualizer(config.output_dir)
        self._subsample = config.render_subsample

    def run(self):
        # tbh I should probably save the intermediate steps in a dict so if I want to
        # extend this pipeline later, I have access to the raw vars without re-running.
        results = {}

        # Step 1
        logger.info("loading eagle...")
        raw = self._loader.load_eagle()
        self._visualizer.save_render(raw, "01_raw.png", "Raw Eagle", self._subsample)
        results["raw"] = raw

        # Step 2
        logger.info("preprocessing...")
        preprocessed = self._preprocessor.preprocess(raw)
        self._visualizer.save_render(preprocessed, "02_clean.png", "Cleaned", self._subsample)

        # ah wait I need to finish the rest. Gonna push this setup first.
        return results


if __name__ == "__main__":
    Pipeline().run()
