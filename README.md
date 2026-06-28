# Augmentus Point Cloud Processing

[![Tests](https://github.com/aish-1509/augmentus-point-cloud/actions/workflows/ci.yml/badge.svg)](https://github.com/aish-1509/augmentus-point-cloud/actions/workflows/ci.yml)

Open3D point-cloud processing pipeline for the Augmentus perception coding
assignment.

The project uses Open3D's built-in `o3d.data.EaglePointCloud()` dataset. Large
`.pcd` / `.ply` files are intentionally not committed because the assignment
notes that point-cloud files are too large and render images are sufficient for
review. The generated PNG outputs live in `docs/renders/`.

## Assignment Checklist

| Requirement | Implementation |
| --- | --- |
| Python project | `src/` package with runnable modules |
| Mandatory Open3D usage | `open3d` used for loading, filtering, normals, KD-tree search, registration, meshing |
| Load `o3d.data.EaglePointCloud()` | `Loader.load_eagle()` in `src/loader.py` |
| Downsampling | `Preprocessor.downsample()` using `voxel_down_sample()` |
| Filtering | `Preprocessor.remove_outliers()` using Statistical Outlier Removal |
| Cropped point cloud before normals | `Preprocessor.crop_to_roi()` called before `NormalEstimator.estimate()` |
| Surface normal estimation | `NormalEstimator.estimate()` with `KDTreeSearchParamHybrid` |
| Euclidean clustering | `ClusterExtractor.extract_euclidean_clusters()` using KD-tree radius BFS |
| Cluster visualization | `Visualizer.save_render()` and `Visualizer.save_multi_angle_render()` |
| Save each cluster separately | `docs/renders/clusters/cluster_*.png` |
| Intermediate renders | raw, downsampled, cropped, normals, clusters in `docs/renders/` |
| Renders shared in README | Images embedded below, not just linked |
| UML class diagram | `docs/uml/class_diagram.drawio` and `docs/uml/class_diagram.png` |
| OOP architecture | `Loader`, `Preprocessor`, `NormalEstimator`, `ClusterExtractor`, `Visualizer`, `Pipeline` |
| Unit tests | `tests/`, currently 19 tests |
| CI | GitHub Actions workflow in `.github/workflows/ci.yml` |

## Quick Start

```bash
git clone https://github.com/aish-1509/augmentus-point-cloud.git
cd augmentus-point-cloud

python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

Run the required assignment pipeline:

```bash
python -m src.pipeline
```

Run the optional advanced pipeline:

```bash
python -m src.advanced_pipeline
```

Run tests:

```bash
python -m pytest tests/ -v
```

## Pipeline Order

The main pipeline follows the assignment wording directly:

1. Load Eagle point cloud with `o3d.data.EaglePointCloud()`.
2. Voxel downsample the raw cloud.
3. Filter ghost points with Statistical Outlier Removal.
4. Crop to a region of interest with an Open3D `AxisAlignedBoundingBox`.
5. Estimate surface normals on the cropped point cloud.
6. Perform Euclidean clustering with KD-tree radius expansion.
7. Save a full colored cluster scene and individual cluster renders.
8. Write cluster statistics to `docs/renders/cluster_summary.json`.

## Latest Verified Run

Verified locally on 28 June 2026:

```text
Base pipeline:
796,825 raw points
329,988 downsampled points
316,519 after Statistical Outlier Removal
314,626 cropped ROI points
5 valid Euclidean clusters
11 tiny connected components dropped as noise

Advanced pipeline:
ICP fitness       = 1.0000
ICP RMSE          = 0.000000
Poisson triangles = 1,042,152
Feature edges     = 97,977 segments above 20 degrees

Tests:
19 passed
```

## Render Gallery

### Raw Eagle Cloud

<img src="docs/renders/01_raw.png" width="700" alt="Raw Eagle point cloud render">

### Downsampled Cloud

<img src="docs/renders/02_downsampled.png" width="700" alt="Downsampled Eagle point cloud render">

### Cropped Point Cloud Used For Normals

This is the explicit cropped ROI stage required before normal estimation.

<img src="docs/renders/03_cropped.png" width="700" alt="Cropped Eagle point cloud render">

### Surface Normals

Normals are visualized as an RGB normal map using `abs(nx, ny, nz)`.

<img src="docs/renders/04_normals.png" width="700" alt="Surface normal RGB render">

### Euclidean Clusters

The Eagle sculpture is mostly one physically connected object, so the main body
dominates the colored render. The Euclidean cluster extractor still finds four
additional valid disconnected components above the 50-point cutoff and drops
11 tiny connected components as noise.

<img src="docs/renders/05_clusters_colored.png" width="700" alt="Colored Euclidean clusters">

### Cluster Views

Four views are saved because one camera angle can hide spatial separation.

| Isometric | Front |
| --- | --- |
| <img src="docs/renders/05_clusters_colored_iso.png" width="420" alt="Cluster isometric view"> | <img src="docs/renders/05_clusters_colored_front.png" width="420" alt="Cluster front view"> |

| Side | Top |
| --- | --- |
| <img src="docs/renders/05_clusters_colored_side.png" width="420" alt="Cluster side view"> | <img src="docs/renders/05_clusters_colored_top.png" width="420" alt="Cluster top view"> |

### Individual Cluster Renders

| Cluster 00 | Cluster 01 | Cluster 02 |
| --- | --- | --- |
| <img src="docs/renders/clusters/cluster_00.png" width="280" alt="Cluster 00"> | <img src="docs/renders/clusters/cluster_01.png" width="280" alt="Cluster 01"> | <img src="docs/renders/clusters/cluster_02.png" width="280" alt="Cluster 02"> |

| Cluster 03 | Cluster 04 |
| --- | --- |
| <img src="docs/renders/clusters/cluster_03.png" width="280" alt="Cluster 03"> | <img src="docs/renders/clusters/cluster_04.png" width="280" alt="Cluster 04"> |

Cluster statistics are saved in
[`docs/renders/cluster_summary.json`](docs/renders/cluster_summary.json).

## Optional Advanced Outputs

These are not required by the assignment, but they show the pipeline can extend
toward inspection or spray-paint coverage workflows.

| Registered Scan | Poisson Mesh | Feature Edges |
| --- | --- | --- |
| <img src="docs/renders/06_registered.png" width="300" alt="Registered scan"> | <img src="docs/renders/07_poisson_mesh.png" width="300" alt="Poisson mesh render"> | <img src="docs/renders/08_feature_edges.png" width="300" alt="Feature edge render"> |

## Architecture

UML source: [`docs/uml/class_diagram.drawio`](docs/uml/class_diagram.drawio)

<img src="docs/uml/class_diagram.png" width="900" alt="UML Class Diagram">

| Module | Responsibility |
| --- | --- |
| `src/config.py` | Central dataclass for all tunable parameters |
| `src/loader.py` | Loads the Open3D Eagle dataset and validates it |
| `src/preprocessor.py` | Voxel downsampling, Statistical Outlier Removal, cropped ROI |
| `src/normal_estimator.py` | PCA-based surface normal estimation |
| `src/cluster_extractor.py` | Euclidean cluster extraction, coloring, cluster summaries |
| `src/visualizer.py` | Headless Matplotlib PNG renders and normal-map visualization |
| `src/pipeline.py` | Required assignment pipeline orchestration |
| `src/advanced_pipeline.py` | Optional registration, Poisson reconstruction, feature-edge extraction |

## OOP Design Decisions

**Encapsulation**: each stage owns its configuration and behavior. For example,
`Preprocessor` manages voxel size, outlier filtering, and crop behavior without
leaking Open3D implementation details into `Pipeline`.

**Abstraction**: `Pipeline` calls simple stage methods such as `load_eagle()`,
`downsample()`, `crop_to_roi()`, `estimate()`, and
`extract_euclidean_clusters()`. The Open3D-specific work stays inside the class
that owns it.

**Composition**: `Pipeline` is composed of `Loader`, `Preprocessor`,
`NormalEstimator`, `ClusterExtractor`, and `Visualizer`. This keeps each class
independently testable.

**Single Responsibility Principle**: `Loader` only loads data. `Preprocessor`
only prepares the cloud. `NormalEstimator` only estimates normals.
`ClusterExtractor` only extracts clusters. `Visualizer` only renders outputs.

**Dependency Injection**: a shared `Config` object is passed into processing
classes so parameters are centralized and easy to tune.

**Inheritance for extension**: `AdvancedPipeline` extends `Pipeline` to add
registration, Poisson reconstruction, and feature-edge detection without
modifying the base assignment path.

**Testability**: tests use synthetic point clouds so they do not need to
download the Eagle dataset or depend on network state.

## Euclidean Clustering Notes

The default clustering path is explicit Euclidean Cluster Extraction:

- build `o3d.geometry.KDTreeFlann`
- visit each point once
- grow connected components with radius search and a queue
- keep clusters with at least `clustering_min_points`
- label smaller components as noise

This matches the usual PCL-style Euclidean clustering concept more directly than
using DBSCAN alone. DBSCAN remains available as an optional comparison helper,
but the main pipeline uses `extract_euclidean_clusters()`.

## Render Design Notes

Open3D's interactive viewer is not ideal for automated review because it needs a
display manager. The renderer uses Matplotlib with the `Agg` backend, which works
in headless CI/Docker environments and saves PNGs directly.

The visualizer also fixes three practical rendering issues:

- The Eagle scan is stored Y-up while Matplotlib is Z-up, so a render-only
  `Rx(-90°)` rotation is applied to the copied NumPy points.
- X/Y/Z limits are locked to one shared physical range so the object is not
  stretched.
- Point size is adaptive, so the 314k-point main body and tiny 50-point clusters
  are both readable.

## Tests

Current suite: `19 passed`.

Test coverage includes:

- voxel downsampling reduces point count
- downsampled output is not empty
- larger voxel size produces fewer points
- downsampled centroids stay inside original bounds
- Statistical Outlier Removal removes distant noise
- Statistical Outlier Removal preserves most inliers
- crop never increases point count
- crop removes boundary points with large padding
- cropped points stay inside padded AABB
- zero crop padding is a no-op
- normal estimation adds one finite normal per point
- empty cloud normal estimation fails clearly
- Euclidean clustering produces more than one segment
- Euclidean clustering finds exactly two synthetic blobs
- clusters are sorted largest-first
- colored cluster cloud preserves point count
- RGB colors stay within `[0, 1]`
- tiny components below min size become noise
- cluster summaries include geometry statistics

## Repository Structure

```text
augmentus-point-cloud/
├── README.md
├── requirements.txt
├── .gitignore
├── .github/workflows/ci.yml
├── src/
│   ├── config.py
│   ├── loader.py
│   ├── preprocessor.py
│   ├── normal_estimator.py
│   ├── cluster_extractor.py
│   ├── visualizer.py
│   ├── pipeline.py
│   └── advanced_pipeline.py
├── tests/
│   ├── test_preprocessing.py
│   ├── test_normal_estimator.py
│   └── test_cluster_extractor.py
└── docs/
    ├── renders/
    │   ├── 01_raw.png
    │   ├── 02_downsampled.png
    │   ├── 03_cropped.png
    │   ├── 04_normals.png
    │   ├── 05_clusters_colored.png
    │   ├── cluster_summary.json
    │   └── clusters/
    └── uml/
        ├── class_diagram.drawio
        └── class_diagram.png
```

## Dependency Notes

`requirements.txt` keeps the runtime small:

- `open3d`
- `numpy`
- `matplotlib`
- `pytest`

`.gitignore` excludes generated caches, virtual environments, `.pcd`, `.ply`,
and OS junk files. The Open3D Eagle dataset is downloaded/cached by Open3D
locally instead of being committed.
