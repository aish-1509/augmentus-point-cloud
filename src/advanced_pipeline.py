from __future__ import annotations

import logging
from collections import defaultdict

import numpy as np
import open3d as o3d

from src.config import Config
from src.pipeline import Pipeline

logger = logging.getLogger(__name__)


class AdvancedPipeline(Pipeline):
    """
    Bro wait. I need to show them I actually understand their Scan-to-Path product.
    Base pipeline is cool, but Augmentus actually does global registration, Poisson
    surface meshing, and edge detection for robot welding. I'm gonna build that here.

    OOP Check: I'm INHERITING from Pipeline.
    Usually inheritance is mid, but here it's actually the Open/Closed Principle.
    I am extending the functionality without touching the original Pipeline class. W.
    """

    def __init__(self, config: Config | None = None) -> None:
        super().__init__(config)
        self._poisson_depth = 8
        self._edge_threshold_deg = 20.0

    def run_advanced(self):
        # run the base stuff first
        results = self.run()
        with_normals = results["with_normals"]

        # -- REGISTRATION --
        # I need to simulate a second scan to show off registration.
        # If I just use ICP (Iterative Closest Point) from a cold start, it's gonna
        # completely fail if the clouds are rotated too much. My MacBook Pro is gonna
        # take off like a jet engine trying to solve that.
        # I need to do FPFH + RANSAC first to get a rough global match, THEN refine with ICP.

        # -- POISSON --
        # Need to turn the cloud into a watertight mesh. Poisson needs normals.
        # Glad I did the PCA normals in the base pipeline.

        # -- EDGES --
        # AutoEdge robot paths... how do I find a weld seam?
        # It's literally just the dihedral angle between two adjacent triangles.
        # If the angle is steep (>20 deg), it's a corner or a seam.
        # I can just use dot products of the face normals for this. Easy.

        # Gonna push this architecture thought process and actually code it next.
        pass


if __name__ == "__main__":
    AdvancedPipeline().run_advanced()
