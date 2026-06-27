"""Configuration values and the reasoning behind them.

Why a dataclass?
I only need a small object that carries named settings around the pipeline.
`@dataclass` writes the boring parts for me: `__init__`, `__repr__`, and `__eq__`.
So instead of hand-writing `def __init__(self, voxel_size=0.02, ...)`, I declare
the fields once and Python builds the constructor. The `float` and `int` pieces
after the colons are type hints; they are there for the next reader and for
tools like mypy.

The section dividers are not meant to be fancy. They make the file scannable:
preprocessing, normals, clustering, output. The order also follows the pipeline
order, so the config can be read top-to-bottom like the data flow.
"""

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
    # This is a starting guess, not a law. The Eagle scan is roughly a 1m-scale
    # sculpture, and the smallest useful details are probably wing edges and the
    # beak curve, around 3-5cm wide.
    # Rule of thumb: choose a voxel size around 1/3 to 1/10 of the smallest
    # feature I care about.
    # 0.005m: barely reduces points, still slow.
    # 0.01m: good detail, but still dense.
    # 0.02m: good balance for this scale, so I am starting here.
    # 0.05m: still possible, but edges get blockier.
    # 0.10m: likely too coarse; wing tips start losing meaningful shape.
    # Too many points surviving → increase. Shape gets blocky → decrease.

    sor_nb_neighbors: int = 30
    # For each point, find this many nearest neighbours and compute mean distance.
    # If that mean distance is much larger than the global average, the point is
    # treated as an outlier.
    # 10: fast, but the estimate is noisy because the sample is tiny.
    # 20: common default and stable for many clouds.
    # 30: slightly more stable, with tiny extra cost after downsampling.
    # 100: very stable, but noticeably slower.
    # I am using 30 because the downsampled Eagle cloud should be sparse enough
    # that this gives a cleaner mean-distance estimate without a real runtime hit.

    sor_std_ratio: float = 2.0
    # Remove points whose mean neighbour distance exceeds
    # (global_mean + std_ratio × std_deviation).
    # This is basically the 2-sigma rule from statistics.
    # 1.0: removes anything over 1σ from the mean, which is too aggressive.
    # 2.0: removes anything over 2σ, roughly the outer 5%.
    # 3.0: removes only extreme outliers, so it is very conservative.
    # Scanner ghost points are usually much farther from the surface than real
    # points, so 2.0 should remove the obvious junk without shaving off the eagle.

    # ── Normal estimation ─────────────────────────────────────────────────────
    normal_radius: float = 0.1
    # Search radius for PCA neighbourhood in metres.
    # To estimate a normal, Open3D needs enough nearby points to fit a local plane.
    # If this were 0.02m, the same as voxel_size, many points would only see one
    # or two neighbours. That is not enough for PCA.
    # Rule of thumb: normal_radius ≈ 3-5× voxel_size.
    # Here: 0.1 / 0.02 = 5× voxel_size.
    # For a 1m object, a 10cm patch is large enough to be stable but still small
    # enough to follow the surface. If this became 0.5m, it could mix the wing,
    # body, and edges into one neighbourhood and the boundary normals would be bad.

    normal_max_nn: int = 30
    # Hard cap on neighbours used per point even if more fall within normal_radius.
    # Even if 200 points are inside the 10cm radius, I only want the nearest 30.
    # PCA quality usually saturates around 20-30 points. Adding points 31-200
    # costs more time than it is worth.
    # The cap also keeps dense scan regions from dominating runtime.

    # ── Clustering (DBSCAN) ───────────────────────────────────────────────────
    clustering_eps: float = 0.10
    # Maximum distance between two points to be considered neighbours in DBSCAN.
    # Must be larger than voxel_size (so adjacent voxel centres can reach each other)
    # and smaller than the gap between separate objects.
    # After actually measuring the cleaned Eagle cloud, the median nearest-neighbour
    # spacing is about 1.4cm. My first 5cm guess sounded reasonable, but with
    # min_points=50 it was still too strict and labelled the real scan as all noise.
    # 10cm gives each surface patch enough local reach while still keeping separated
    # components from collapsing into one giant blob.
    # eps < voxel_size: points cannot reach each other, so everything becomes noise.
    # eps = 0.10: data-driven starting point for this Eagle scan.
    # eps > real object gaps: separate components merge into one cluster.
    # Tune this first if clustering looks wrong.

    clustering_min_points: int = 50
    # Minimum neighbours within eps for a point to be a DBSCAN core point.
    # Without enough neighbours, DBSCAN labels the point as noise (-1).
    # 5: tiny noise fragments can survive as fake clusters.
    # 50: requires a real local patch before calling something a cluster.
    # 500: can throw away smaller but real parts of the eagle.
    # After downsampling, 50 points is roughly a credit-card-sized surface patch,
    # which is a reasonable minimum for avoiding individual stray points.

    # ── Output ────────────────────────────────────────────────────────────────
    output_dir: str = "docs/renders"
    # Directory where all PNG renders are written. Created automatically if absent.

    render_subsample: int = 50000
    # Matplotlib 3D scatter is still CPU-rendered, so I do not want to throw the
    # full 316k cleaned points into every PNG.
    # But the first 8k render pass looked too thin, like the eagle was turning
    # into dust. 50k is the better presentation tradeoff: denser, more solid,
    # still quick enough on my laptop.
    # Actual processing always uses the full cloud; this only affects image output.
