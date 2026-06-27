"""Advanced pipeline: registration, Poisson meshing, and feature edges.

Run from the project root:

python -m src.advanced_pipeline
"""

from __future__ import annotations

import logging
from collections import defaultdict

import numpy as np
import open3d as o3d

from src.config import Config
from src.pipeline import Pipeline

logger = logging.getLogger(__name__)


class AdvancedPipeline(Pipeline):
    """Extends Pipeline with registration, reconstruction, and edge detection.

    This class uses inheritance as an explicit extension point: the base pipeline
    remains responsible for the required assignment stages, while the advanced
    pipeline adds optional geometry-processing stages afterward. That follows the
    Open/Closed Principle without complicating the simpler pipeline path.
    """

    def __init__(self, config: Config | None = None) -> None:
        super().__init__(config)
        # depth=9 gives roughly 4x more triangles than depth=8. The mesh actually
        # starts to show feather groove detail at this level, which is exactly what
        # you want if you're planning a spray-paint path — the nozzle needs to know
        # every surface fold so coverage stays even.
        self._poisson_depth = 9
        self._edge_threshold_deg = 20.0

    # -- REGISTRATION ---------------------------------------------------------

    def _prepare_for_fpfh(
        self,
        pcd: o3d.geometry.PointCloud,
        voxel_size: float,
    ) -> tuple[o3d.geometry.PointCloud, o3d.pipelines.registration.Feature]:
        """Downsample and compute FPFH features for global registration.

        FPFH acts like a local geometry fingerprint. Absolute xyz coordinates are
        fragile because the object might move, but local neighbourhood shape is
        more stable.
        A corner should still "feel" like a corner after rotation/translation.

        So the move is:
        1. Downsample so matching is not dominated by raw point count.
        2. Estimate normals because FPFH needs local surface direction.
        3. Build the feature descriptors that RANSAC can match.
        """
        down = pcd.voxel_down_sample(voxel_size * 2)
        down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(
                radius=voxel_size * 4,
                max_nn=30,
            )
        )
        down.orient_normals_consistent_tangent_plane(50)

        fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            down,
            o3d.geometry.KDTreeSearchParamHybrid(
                radius=voxel_size * 10,
                max_nn=100,
            ),
        )
        return down, fpfh

    def global_registration(
        self,
        source: o3d.geometry.PointCloud,
        target: o3d.geometry.PointCloud,
        voxel_size: float = 0.05,
    ) -> o3d.pipelines.registration.RegistrationResult:
        """Find a rough source->target alignment with FPFH + RANSAC.

        ICP from a cold start can fail when the scans start too far apart, because
        it may converge to a local minimum.

        RANSAC is the rough-match phase: it tries feature correspondences and
        rejects nonsense transforms until it finds a decent initial alignment.
        Then local ICP can refine from a reasonable starting transform.
        """
        logger.info("Computing FPFH features for global registration...")
        src_down, src_feat = self._prepare_for_fpfh(source, voxel_size)
        tgt_down, tgt_feat = self._prepare_for_fpfh(target, voxel_size)

        return o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            src_down,
            tgt_down,
            src_feat,
            tgt_feat,
            True,
            voxel_size * 1.5,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
            3,
            [
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                    voxel_size * 1.5
                ),
            ],
            o3d.pipelines.registration.RANSACConvergenceCriteria(100_000, 0.999),
        )

    def local_registration(
        self,
        source: o3d.geometry.PointCloud,
        target: o3d.geometry.PointCloud,
        initial_transform: np.ndarray | None = None,
        max_dist: float = 0.02,
    ) -> o3d.pipelines.registration.RegistrationResult:
        """Refine alignment using point-to-plane ICP.

        Point-to-point ICP only minimizes nearest xyz point distances.
        Point-to-plane ICP uses target normals, so it minimizes distance along the
        surface direction. For smooth scanned geometry, that is usually more
        stable.
        """
        init = initial_transform if initial_transform is not None else np.eye(4)

        target_for_icp = target
        if not target_for_icp.has_normals():
            target_for_icp = self._normal_estimator.estimate(
                o3d.geometry.PointCloud(target_for_icp)
            )

        return o3d.pipelines.registration.registration_icp(
            source,
            target_for_icp,
            max_dist,
            init,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100),
        )

    # -- POISSON RECONSTRUCTION -----------------------------------------------

    def poisson_reconstruction(
        self,
        pcd: o3d.geometry.PointCloud,
    ) -> tuple[o3d.geometry.TriangleMesh, np.ndarray]:
        """Turn oriented points into a watertight mesh with Poisson reconstruction.

        Poisson tries to recover an inside/outside function from the normal vector
        field. The formal shape is:

            laplacian(chi) = divergence(V)

        V is the normal field. chi is the indicator field. Once chi exists, the
        surface is basically the boundary where inside flips to outside.

        `depth=8` controls octree detail. Lower values lose shape detail; higher
        values increase memory and runtime. 8 is a practical middle for this scan.
        """
        if not pcd.has_normals():
            raise ValueError("Poisson needs oriented normals. Run NormalEstimator first.")

        logger.info(
            "Running Poisson reconstruction (depth=%d).",
            self._poisson_depth,
        )
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            pcd,
            depth=self._poisson_depth,
        )

        density_array = np.asarray(densities)
        if len(density_array) > 0:
            # Poisson forces watertightness, so it can hallucinate thin ghost areas.
            # Low-density vertices are the suspicious ones. Drop the weakest 5%.
            mesh.remove_vertices_by_mask(density_array < np.quantile(density_array, 0.05))

        return mesh, density_array

    # -- EDGE DETECTION --------------------------------------------------------

    def detect_feature_edges(
        self,
        mesh: o3d.geometry.TriangleMesh,
    ) -> list[tuple[np.ndarray, np.ndarray, float]]:
        """Find sharp feature edges using dihedral angles between mesh faces.

        The whole idea:
        A smooth surface has neighbouring triangle normals pointing almost the
        same way. A seam/corner has neighbouring normals that suddenly disagree.

        So for every mesh edge shared by two triangles:

            theta = arccos(n0 dot n1)

        If theta is above the threshold, that edge is interesting. For a
        spray-painting workflow, these are the places I would inspect first:
        corners, feather ridges, plinth edges, and wing-body transitions where
        the nozzle may need slower motion, different standoff distance, or extra
        overlap.

        Mesh = graph, faces = nodes, shared edges = adjacency. Once I saw it like
        that, the implementation became pretty direct.
        """
        mesh.compute_triangle_normals()
        verts = np.asarray(mesh.vertices)
        tris = np.asarray(mesh.triangles)
        face_normals = np.asarray(mesh.triangle_normals)

        edge_to_faces: dict[tuple[int, int], list[int]] = defaultdict(list)
        for face_id, tri in enumerate(tris):
            for i in range(3):
                edge = tuple(sorted((int(tri[i]), int(tri[(i + 1) % 3]))))
                edge_to_faces[edge].append(face_id)

        feature_edges: list[tuple[np.ndarray, np.ndarray, float]] = []
        for edge, face_ids in edge_to_faces.items():
            if len(face_ids) != 2:
                continue

            n0 = face_normals[face_ids[0]]
            n1 = face_normals[face_ids[1]]

            # Clip saves us from tiny floating point errors pushing dot outside [-1, 1].
            dot = float(np.clip(np.dot(n0, n1), -1.0, 1.0))
            angle_deg = float(np.degrees(np.arccos(dot)))

            if angle_deg > self._edge_threshold_deg:
                # These are the places where a spray nozzle needs to slow down
                # or adjust its standoff distance — wing-body joints, feather
                # ridges, the plinth corners. Flat surface sections in between
                # can be covered at a constant speed.
                feature_edges.append((verts[edge[0]], verts[edge[1]], angle_deg))

        logger.info(
            "Detected %d feature edges above %.1f degrees",
            len(feature_edges),
            self._edge_threshold_deg,
        )
        return feature_edges

    # -- ADVANCED RUN ----------------------------------------------------------

    def run_advanced(self) -> dict[str, object]:
        """Run base pipeline, then advanced geometry stages.

        The advanced stages report concrete metrics and artifacts: RANSAC/ICP
        fitness, RMSE, a Poisson mesh render, and feature-edge counts. Those
        outputs make the extended pipeline easier to evaluate and debug.
        """
        results = self.run()
        with_normals = results["with_normals"]
        if not isinstance(with_normals, o3d.geometry.PointCloud):
            raise TypeError("Expected results['with_normals'] to be an Open3D PointCloud.")

        # Stage 5: registration
        logger.info("=" * 55)
        logger.info("STAGE 5 - Global + local registration")
        angle = np.radians(45)
        simulated_transform = np.array(
            [
                [np.cos(angle), -np.sin(angle), 0.0, 0.10],
                [np.sin(angle), np.cos(angle), 0.0, 0.05],
                [0.0, 0.0, 1.0, 0.02],
                [0.0, 0.0, 0.0, 1.00],
            ]
        )

        source = o3d.geometry.PointCloud(with_normals)
        source.transform(simulated_transform)
        target = o3d.geometry.PointCloud(with_normals)

        ransac_result = self.global_registration(source, target)
        icp_result = self.local_registration(
            source,
            target,
            initial_transform=ransac_result.transformation,
        )

        aligned = o3d.geometry.PointCloud(source)
        aligned.transform(icp_result.transformation)
        self._visualizer.save_render(
            aligned,
            "06_registered.png",
            f"Stage 5: ICP fitness={icp_result.fitness:.4f}",
            self._subsample,
        )
        logger.info(
            "Registration complete: fitness=%.4f, RMSE=%.6f",
            icp_result.fitness,
            icp_result.inlier_rmse,
        )

        results["registration_ransac"] = ransac_result
        results["registration_icp"] = icp_result
        results["registered"] = aligned

        # Stage 6: Poisson reconstruction
        logger.info("=" * 55)
        logger.info("STAGE 6 - Poisson surface reconstruction")
        mesh, densities = self.poisson_reconstruction(with_normals)
        tri_count = len(np.asarray(mesh.triangles))
        logger.info("Poisson mesh: %s triangles", f"{tri_count:,}")
        # Sample densely so the render actually shows surface detail, not just a
        # rough silhouette. 300K points ≈ enough to show feather groove texture.
        mesh_sample = mesh.sample_points_uniformly(number_of_points=300_000)
        self._visualizer.save_render(
            mesh_sample,
            "07_poisson_mesh.png",
            f"Stage 6: Watertight Poisson mesh — {tri_count:,} triangles",
            self._subsample,
        )
        results["mesh"] = mesh
        results["mesh_densities"] = densities

        # Stage 7: feature edges
        logger.info("=" * 55)
        logger.info("STAGE 7 - Feature edge detection")
        feature_edges = self.detect_feature_edges(mesh)

        if feature_edges:
            edge_points = np.vstack([[p0, p1] for p0, p1, _ in feature_edges])
            edge_pcd = o3d.geometry.PointCloud()
            edge_pcd.points = o3d.utility.Vector3dVector(edge_points)
            self._visualizer.save_render(
                edge_pcd,
                "08_feature_edges.png",
                f"Stage 7: {len(feature_edges)} candidate edge segments",
                self._subsample,
            )
            results["feature_edge_cloud"] = edge_pcd

        results["feature_edges"] = feature_edges

        logger.info("=" * 55)
        logger.info("ADVANCED PIPELINE COMPLETE.")
        logger.info("=" * 55)

        return results


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(name)s - %(message)s")
    AdvancedPipeline().run_advanced()
