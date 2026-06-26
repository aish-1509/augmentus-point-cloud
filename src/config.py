from dataclasses import dataclass


@dataclass
class Config:
    """Central configuration for the point cloud processing pipeline.

    Every numeric parameter lives here. No magic numbers anywhere else.
    Change a value once and every stage picks it up automatically.
    """

    # ── Preprocessing ─────────────────────────────────────────────────────────
    voxel_size: float = 0.02
    # Each 3D cube of this side-length (in metres) collapses to one centroid point.
    # Eagle is a large outdoor scan (~1m scale object).
    # 0.02m = 2cm cubes keeps fine detail while cutting point count ~30x.
    # Too many points surviving → increase. Shape gets blocky → decrease.

    sor_nb_neighbors: int = 30
    # For each point, find this many nearest neighbours and compute mean distance.
    # More neighbours = more stable estimate, slightly slower.
    # 30 is a safe default after downsampling to ~50k points.

    sor_std_ratio: float = 2.0
    # Remove points whose mean neighbour distance exceeds
    # (global_mean + std_ratio × std_deviation).
    # 2.0 = 2-sigma rule: removes ~5% of points (only genuine outliers).
    # Lower = more aggressive removal. Higher = more conservative.

    # ── Normal estimation ─────────────────────────────────────────────────────
    normal_radius: float = 0.1
    # Search radius for PCA neighbourhood in metres.
    # Must be large enough to capture enough neighbours to define a stable plane.
    # Rule of thumb: 3–5× voxel_size. Here: 0.1 / 0.02 = 5× voxel_size.

    normal_max_nn: int = 30
    # Hard cap on neighbours used per point even if more fall within normal_radius.
    # PCA quality saturates at ~20–30 points. Capping prevents dense regions
    # from dominating computation time.

    # ── Clustering (DBSCAN) ───────────────────────────────────────────────────
    clustering_eps: float = 0.05
    # Maximum distance between two points to be considered neighbours in DBSCAN.
    # Must be larger than voxel_size (so adjacent voxel centres can reach each other)
    # and smaller than the gap between separate objects.
    # Too small → everything labelled noise. Too large → all clusters merge into one.
    # Tune this first if clustering looks wrong.

    clustering_min_points: int = 50
    # Minimum neighbours within eps for a point to be a DBSCAN core point.
    # After downsampling, 50 points ≈ a 20cm² surface patch — a meaningful minimum.
    # Too small → noise fragments become clusters. Too large → real parts labelled noise.

    # ── Output ────────────────────────────────────────────────────────────────
    output_dir: str = "docs/renders"
    # Directory where all PNG renders are written. Created automatically if absent.

    render_subsample: int = 8000
    # Matplotlib 3D scatter slows badly past ~10k points.
    # Randomly subsample to this count for rendering only.
    # Actual processing always uses the full cloud — this only affects image output.
