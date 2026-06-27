# Augmentus Point Cloud Processing

[![Tests](https://github.com/aish-1509/augmentus-point-cloud/actions/workflows/ci.yml/badge.svg)](https://github.com/aish-1509/augmentus-point-cloud/actions/workflows/ci.yml)

Runnable Open3D pipeline for the Augmentus perception assignment.

The pipeline uses Open3D's built-in `o3d.data.EaglePointCloud()` dataset, so no
large `.pcd` or `.ply` files are committed. The important outputs are saved as
PNG renders under `docs/renders/`.

## Quick Start

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

Run the required pipeline:

```bash
python -m src.pipeline
```

Run the extended pipeline with registration, Poisson reconstruction, and feature
edge detection:

```bash
python -m src.advanced_pipeline
```

Run the tests:

```bash
python -m pytest tests/ -v
```

## Pipeline Stages

1. Load the Eagle point cloud with `o3d.data.EaglePointCloud()`.
2. Voxel downsample the raw scan.
3. Remove statistical outliers.
4. Estimate and consistently orient surface normals.
5. Segment the cloud using DBSCAN as the Euclidean-distance clustering step.
6. Save PNG renders for the raw, processed, normal-map, clustered, and per-cluster
   outputs.

The advanced pipeline then adds:

1. FPFH + RANSAC global registration.
2. Point-to-plane ICP refinement.
3. Poisson surface reconstruction.
4. Dihedral-angle feature edge detection.

## Latest Verified Run

Verified locally on 28 June 2026:

```text
Base pipeline:
796,825 raw points -> 316,519 clean points -> 13 clusters extracted

Advanced pipeline:
ICP fitness       = 1.0000
ICP RMSE          = 0.000000
Poisson triangles = 1,022,704  (depth=9)
Feature edges     = 95,768 segments above 20 degrees

Tests:
11 passed
```

## Render Outputs

| Stage | Render | Notes |
| --- | --- | --- |
| 1 | [Raw Eagle](docs/renders/01_raw.png) | Original Open3D Eagle scan |
| 2 | [Preprocessed](docs/renders/02_preprocessed.png) | Voxel downsample + statistical outlier removal |
| 3 | [Normals](docs/renders/03_normals.png) | RGB normal-map view using `abs(nx, ny, nz)` |
| 4 | [Clusters](docs/renders/04_clusters.png) | DBSCAN labels painted with `tab20` colors |
| 5 | [Cluster 0](docs/renders/05_cluster_00.png) | Largest extracted cluster |
| 5 | [Cluster 1](docs/renders/05_cluster_01.png) | Next largest extracted cluster |
| 5 | [Cluster 2](docs/renders/05_cluster_02.png) | Next largest extracted cluster |
| 5 | [Cluster 3](docs/renders/05_cluster_03.png) | Next largest extracted cluster |
| 5 | [Cluster 4](docs/renders/05_cluster_04.png) | Next largest extracted cluster |
| 6 | [Registered](docs/renders/06_registered.png) | Simulated second scan aligned back to target |
| 7 | [Poisson Mesh](docs/renders/07_poisson_mesh.png) | Mesh sampled back to points for a lightweight render |
| 8 | [Feature Edges](docs/renders/08_feature_edges.png) | Sharp mesh transitions useful for spray-paint coverage checks |

The clustered render is mostly one dominant color because the Eagle sculpture is
one physically connected object. With `eps=0.10` and `min_points=50`, DBSCAN finds
one main body cluster with `314,018` points, plus 12 small fragments and `814`
noise points. That is expected for this scan: the clustering stage separates small
disconnected regions without forcing the main sculpture body apart.

## Render Reflection

The first render pass was technically correct, but visually it was not doing the
data justice. Matplotlib's default 3D plot style made the Eagle scan look too
sparse: white background, grey gridlines, tiny points, and an average camera
angle. The geometry was working under the hood, but the images did not communicate
that clearly.

The next issue was more subtle: Matplotlib can make correct 3D data look wrong.
If X, Y, and Z limits are auto-scaled independently, a wide point cloud can look
stretched or blobby in the final PNG. Also, large semi-transparent scatter points
turn dense 3D geometry into a muddy cloud because Matplotlib is not a true
hardware point-cloud renderer.

The final orientation issue was a coordinate-frame mismatch, not a camera problem.
The Eagle dataset is stored Y-up (measured: the Y axis spans ~8.6 units from plinth
base to wing tip, the largest range of the three axes). Matplotlib is Z-up. Passing
Y-up data into a Z-up renderer lays the eagle on its side regardless of camera angle.

The visualizer applies a render-only `Rx(-90°)` rotation to the NumPy points before
plotting. `Rx(-90°)` maps `new_z = -y`, which correctly inverts the scanned Y axis
onto matplotlib's Z-up axis so the plinth sits at the bottom and the wing tip is at
the top — matching the physical orientation of the sculpture.

The final render pass treats the PNGs more like visual inspection artifacts than
default math plots:

- Removed axes and gridlines so the point cloud is the only subject in the frame.
- Switched to a dark background so cluster colors and normal-map colors have real
  contrast.
- Increased render subsampling from `8,000` to `400,000` points. The cleaned
  Eagle cloud has `316,519` points, so the processed renders now show every
  cleaned point instead of dropping most of the scan.
- Used tiny opaque points instead of large transparent points to reduce visual
  sludge from depth sorting.
- Applied a render-only `Rx(-90°)` posture correction (`new_z = -y`) so the Eagle
  is upright — plinth at the bottom, wing tip at the top.
- Locked X/Y/Z to one shared physical range so the rendered object is not warped.
- Switched to an orthographic, low 3/4 camera angle after the object itself was
  stood upright.
- Used deterministic subsampling so rerunning the pipeline does not create random
  visual diffs in Git.

The important detail: this only changes rendering. The actual processing pipeline
still uses the full point cloud at every stage.

## Architecture

UML class diagram: [docs/uml/class_diagram.drawio](docs/uml/class_diagram.drawio)

Main modules:

- `src/config.py` keeps the tunable pipeline values in one dataclass.
- `src/loader.py` loads the Eagle dataset and validates that it is non-empty.
- `src/preprocessor.py` handles voxel downsampling and statistical outlier removal.
- `src/normal_estimator.py` estimates PCA-based normals and orients them consistently.
- `src/cluster_extractor.py` runs DBSCAN clustering and returns per-cluster clouds.
- `src/visualizer.py` saves headless Matplotlib PNG renders.
- `src/pipeline.py` orchestrates the required assignment pipeline.
- `src/advanced_pipeline.py` extends the base pipeline with registration,
  reconstruction, and feature edge extraction.

## Why The Advanced Pipeline Matters

The extra stages are framed around spray-paint coverage on the Eagle sculpture:

- Registration simulates aligning two scans of the same object before planning.
- Poisson reconstruction turns the point cloud into a dense surface, which is
  easier to reason about than disconnected points when thinking about coverage.
- Dihedral feature edges highlight ridges, corners, plinth boundaries, and
  wing-body transitions where a spray nozzle may need slower motion, adjusted
  standoff distance, or extra overlap.

The base assignment is still fully contained in `src/pipeline.py`; the advanced
pipeline is a separate extension so the required path stays simple.

## Tests

The tests use synthetic in-memory point clouds instead of downloading the Eagle
dataset. That keeps the test suite fast, deterministic, and safe for CI.

- `tests/test_preprocessing.py` checks that downsampling reduces point count,
  preserves a non-empty result, behaves monotonically with voxel size, keeps
  centroids inside bounds, and removes statistical outliers without deleting the
  core surface.
- `tests/test_cluster_extractor.py` builds two separated Gaussian blobs so DBSCAN
  must produce more than one segment. It also checks exact cluster count, sorting,
  point-count preservation, and RGB validity.

## Notes On Parameter Tuning

The first clustering guess was `eps=0.05`, but the real Eagle run showed that
`eps=0.05` with `min_points=50` labelled the whole cleaned cloud as noise. After
measuring the cleaned scan, the median nearest-neighbour distance was about
1.4 cm, so `eps=0.10` is the data-driven default used here. It keeps
`min_points=50` strict enough to avoid tiny noise islands while still producing
real segments on the Eagle cloud.
