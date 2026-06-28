# Augmentus Point Cloud Processing

[![Tests](https://github.com/aish-1509/augmentus-point-cloud/actions/workflows/ci.yml/badge.svg)](https://github.com/aish-1509/augmentus-point-cloud/actions/workflows/ci.yml)

Open3D pipeline for the Augmentus perception coding assignment. Loads the Eagle sculpture scan, downsamples it, crops to a clean ROI, estimates surface normals on that cropped cloud, runs Euclidean cluster extraction, and saves everything as PNG renders.

No `.pcd` or `.ply` files are committed — the Eagle dataset downloads automatically from Open3D on first run and caches locally. The render images below are the actual pipeline outputs.

---

## Quick Start

```bash
git clone https://github.com/aish-1509/augmentus-point-cloud.git
cd augmentus-point-cloud

python -m venv .venv
source .venv/bin/activate        # macOS / Linux
# .venv\Scripts\activate         # Windows

pip install -r requirements.txt
```

Run the tests:

```bash
pytest
```

Run the core assignment pipeline:

```bash
python -m src.pipeline
```

Outputs land in `docs/renders/`. Run the extended pipeline for registration, Poisson meshing, and feature edge detection:

```bash
python -m src.advanced_pipeline
```

---

## Pipeline Overview

The pipeline runs five stages in order. Each stage is a separate class with its own job — no stage knows what the others are doing.

| Stage | What happens | Render saved |
|---|---|---|
| 1 — Load | `Loader.load_eagle()` fetches the Eagle PCD via `o3d.data.EaglePointCloud()` | `01_raw.png` |
| 2 — Downsample | `Preprocessor.downsample()` runs voxel grid reduction at `voxel_size=0.02m` | `02_downsampled.png` |
| 3 — Filter + Crop | `Preprocessor.remove_outliers()` runs SOR, then `crop_to_roi()` carves an AABB | `03_cropped.png` |
| 4 — Normals | `NormalEstimator.estimate()` runs PCA on the **cropped** cloud | `04_normals.png` |
| 5 — Cluster | `ClusterExtractor.extract_euclidean_clusters()` uses KD-tree BFS radius expansion | `05_clusters_colored.png` + multi-angle + `clusters/cluster_XX.png` |

The assignment specifically says to estimate normals for the "cropped point cloud." Stage 3 makes that explicit — the crop is a named, documented stage, not a hidden side-effect. Stage 4 only ever sees the cropped cloud.

---

## Assignment Requirement Checklist

| Requirement (from PDF) | Where it lives |
|---|---|
| `o3d.data.EaglePointCloud()` | `Loader.load_eagle()` in `src/loader.py` |
| Voxel downsampling | `Preprocessor.downsample()` in `src/preprocessor.py` |
| Cropped point cloud | `Preprocessor.crop_to_roi()` in `src/preprocessor.py` |
| Normals on cropped cloud | `NormalEstimator.estimate()` in `src/normal_estimator.py` — called after crop |
| Euclidean clustering | `ClusterExtractor.extract_euclidean_clusters()` — KD-tree BFS, not just DBSCAN |
| Highlight / save clusters | `05_clusters_colored.png` (scene overview) + `clusters/cluster_XX.png` per cluster |
| Intermediate renders | `docs/renders/` — raw, downsampled, cropped, normals, clusters |
| Renders shared on README | Every required stage is embedded below using `<img>` tags |
| UML class diagram | `docs/uml/class_diagram.drawio` and `docs/uml/class_diagram.png` |
| OOP demonstrated | Encapsulation, composition, inheritance, DI — all documented in this README |
| Unit test: downsampling reduces count | `TestVoxelDownsampling::test_reduces_point_count` |
| Unit test: clustering > 1 segment | `TestClusterExtractor::test_produces_more_than_one_cluster` |
| At least 2 unit tests | 19 total across preprocessing, clustering, and normals |
| No PCD/PLY committed | `.gitignore` excludes `*.pcd` and `*.ply` |
| README with setup + run instructions | This file |
| README with architecture description | "Architecture" section below |

For a compact final cross-check see [`docs/final_submission_checklist.md`](docs/final_submission_checklist.md).
For a row-by-row mapping of every assignment sentence to code or docs, see [`docs/assignment_traceability_matrix.md`](docs/assignment_traceability_matrix.md).

---

## Render Gallery

All images are the actual outputs from running `python -m src.pipeline`. The Eagle dataset downloads automatically on the first run.

### Stage 1 — Raw Eagle

<img src="docs/renders/01_raw.png" width="600">

### Stage 2 — Voxel Downsampled

<img src="docs/renders/02_downsampled.png" width="600">

### Stage 3 — Cropped ROI (used for all downstream processing)

The crop step is what makes the "estimate normals for the cropped point cloud" requirement explicit. After SOR filtering, an AABB is computed from the data and shrunk inward by `crop_padding=0.05m` per face. Everything after this sees the cropped cloud only.

<img src="docs/renders/03_cropped.png" width="600">

### Stage 4 — Surface Normals

Normal vectors are mapped to RGB: `|Nx| → Red`, `|Ny| → Green`, `|Nz| → Blue`. Using absolute values avoids the clipping issue where negative normal components would show as black. The color pattern reveals surface orientation at a glance — flat horizontal surfaces read blue/green, vertical faces read red.

<img src="docs/renders/04_normals.png" width="600">

### Stage 5 — Euclidean Clusters (four viewpoints)

The main colored scene render shows all valid Euclidean clusters together. The largest connected body is the Eagle sculpture itself; the smaller valid clusters are separated surface fragments above the 50-point cutoff.

<img src="docs/renders/05_clusters_colored.png" width="600">

A single angle can hide clusters that overlap in depth. Four viewpoints together make spatial separation visually unambiguous.

| Isometric | Front | Side | Top |
|---|---|---|---|
| <img src="docs/renders/05_clusters_colored_iso.png" width="230"> | <img src="docs/renders/05_clusters_colored_front.png" width="230"> | <img src="docs/renders/05_clusters_colored_side.png" width="230"> | <img src="docs/renders/05_clusters_colored_top.png" width="230"> |

### Stage 5 — Individual Cluster Renders

Each cluster is saved separately with its assigned vibrant color. Small clusters that are invisible slivers in the combined view are clearly visible here.

| cluster_00 | cluster_01 | cluster_02 |
|---|---|---|
| <img src="docs/renders/clusters/cluster_00.png" width="280"> | <img src="docs/renders/clusters/cluster_01.png" width="280"> | <img src="docs/renders/clusters/cluster_02.png" width="280"> |

| cluster_03 | cluster_04 | |
|---|---|---|
| <img src="docs/renders/clusters/cluster_03.png" width="280"> | <img src="docs/renders/clusters/cluster_04.png" width="280"> | |

A `cluster_summary.json` is also written to `docs/renders/` with point count, bounding box, and centroid data for each cluster.

**Note on cluster count:** The Euclidean pass finds five valid components above the `min_cluster_size=50` threshold. A sixth connected component exists in the data with 37 points, but falls below the cutoff. Lowering the threshold to 30 would include it, but at 37 points (roughly 2 cm × 2 cm surface patch at 2 cm voxel spacing) it is more likely a scanner edge artifact than meaningful geometry. Keeping the threshold at 50 avoids presenting noise as a cluster.

### Advanced Pipeline Renders (optional)

| Registered | Poisson Mesh | Feature Edges |
|---|---|---|
| <img src="docs/renders/06_registered.png" width="280"> | <img src="docs/renders/07_poisson_mesh.png" width="280"> | <img src="docs/renders/08_feature_edges.png" width="280"> |

---

## Architecture

**UML class diagram:** [docs/uml/class_diagram.drawio](docs/uml/class_diagram.drawio)
*(Open in [diagrams.net](https://app.diagrams.net) or the draw.io VS Code extension)*

**Architecture design notes** (OOP decisions, design evolution, crop rationale): [`docs/architecture_design_notes.md`](docs/architecture_design_notes.md)

<img src="docs/uml/class_diagram.png" width="900">

The diagram is not there as decoration. I used it as the architecture map to check responsibilities and relationships before finalizing the implementation: where `Config` gets injected, which classes are composed into `Pipeline`, and where `AdvancedPipeline` extends the base pipeline.

The pipeline uses composition: `Pipeline` holds one instance of each processing component and coordinates the call order. No component knows about the others.

| Class | File | What it owns |
|---|---|---|
| `Config` | `src/config.py` | Every tunable parameter in one `@dataclass`. No magic numbers in other files. |
| `Loader` | `src/loader.py` | Fetches the Eagle scan via `o3d.data.EaglePointCloud()`. Validates it's non-empty. |
| `Preprocessor` | `src/preprocessor.py` | Voxel downsampling, SOR filtering, AABB crop. Each step is its own public method. |
| `NormalEstimator` | `src/normal_estimator.py` | PCA normal estimation with KDTreeHybrid. Orients normals consistently. |
| `ClusterExtractor` | `src/cluster_extractor.py` | KD-tree BFS Euclidean cluster extraction. Coloring, per-cluster clouds, JSON summary. |
| `Visualizer` | `src/visualizer.py` | Headless Matplotlib renders. Dark background, coordinate-frame correction, multi-angle. |
| `Pipeline` | `src/pipeline.py` | Orchestrates the 5-stage assignment pipeline using the above components. |
| `AdvancedPipeline` | `src/advanced_pipeline.py` | Extends `Pipeline` with FPFH+RANSAC, ICP, Poisson reconstruction, feature edges. |

---

## OOP Refresher and Design Thinking

Honestly — OOP was something I studied back in Year 1, and while the concepts were there somewhere in my head, I hadn't applied them to a project of this structure in a while. Since the assignment PDF specifically called out OOP and UML as evaluation criteria, I knew I couldn't just write some functions and call it a day.

So before touching any code, I went back and revised the actual concepts. Encapsulation, abstraction, composition vs inheritance, dependency injection, single responsibility. Not just as definitions, but as things I could point to in the codebase and explain why they're there.

Here's where each concept actually shows up in this project:

**Encapsulation**
Every class stores its configuration as private attributes (`self._voxel_size`, `self._tolerance`, `self._crop_padding`). External code can't accidentally change what a stage is doing mid-run. The `@property` decorator on `Visualizer.output_dir` gives read access without exposing the backing attribute.

**Abstraction**
`Pipeline` calls `preprocessor.downsample()`, `preprocessor.crop_to_roi()`, `normal_estimator.estimate()`. It doesn't know anything about how voxel grids work, how AABB bounds are computed, or what KDTreeSearchParamHybrid means. Each class hides its Open3D internals behind a simple interface.

**Composition over inheritance (base pipeline)**
`Pipeline` owns instances of `Loader`, `Preprocessor`, `NormalEstimator`, `ClusterExtractor`, `Visualizer`. None of them inherit from each other. Each one can be swapped or tested independently without touching the others.

**Inheritance (advanced pipeline)**
`AdvancedPipeline` extends `Pipeline` — it calls `self.run()` to get the base stages, then adds registration, Poisson meshing, and edge detection afterward. The base pipeline is unchanged. This is the Open/Closed Principle applied: closed to modification, open to extension.

**Dependency Injection**
Every component receives a `Config` dataclass at construction. Change `voxel_size` in one place, it propagates automatically. No global state, no hardcoded magic numbers scattered across files.

**Single Responsibility**
`Loader` only loads. `Preprocessor` only cleans and crops. `NormalEstimator` only estimates normals. `ClusterExtractor` only clusters. `Visualizer` only renders. Each class is testable in isolation because it has exactly one job.

**`@dataclass` for Config**
`@dataclass` generates `__init__`, `__repr__`, and `__eq__` automatically. Every field has an inline comment explaining what it does and why the default value was chosen — not just what it is. That comment doubles as engineering documentation.

---

## Development Notes: What I Had to Figure Out

This was not just "call a few Open3D functions in order." The core work was
understanding what each stage was supposed to prove, then making that proof
visible in the code, tests, UML, and renders.

### 1. The assignment wording had to drive the pipeline order

The line that mattered most was: *"Estimate surface normals for the cropped
point cloud."*

At first glance, it is easy to read "preprocessing" as just downsample + remove
outliers. But if normals are estimated immediately after that, the cropped-cloud
requirement is only loosely satisfied at best. So I made crop a real stage:

```text
raw -> downsample -> SOR filter -> crop ROI -> normals -> Euclidean clusters
```

`crop_to_roi()` is its own public method, `03_cropped.png` is saved as its own
render, and `NormalEstimator.estimate()` only receives the cropped cloud. That
way the requirement is not hidden inside a vague helper.

### 2. Downsample before SOR, otherwise the search cost is wasted

Statistical Outlier Removal needs nearest-neighbour distances. Running that on
the full raw Eagle scan would do expensive neighbour searches on ~800k points.
Voxel downsampling first cuts the cloud to ~330k points, then SOR runs on that
smaller cloud. The shape is still preserved, but the expensive step has less
work to do.

That ordering is why `Preprocessor` exposes the steps separately and why the
pipeline logs each point count:

```text
796,825 raw -> 329,988 downsampled -> 316,519 cleaned -> 314,626 cropped
```

### 3. The numbers in `Config` needed actual reasoning, not magic defaults

I did not want `config.py` to look like a random pile of constants. Every number
has a tradeoff:

- `voxel_size=0.02` keeps 2cm detail, which is reasonable for a sculpture-scale
  scan where feather/beak details are a few centimetres wide.
- `sor_nb_neighbors=30` gives a stable mean-distance estimate after downsampling.
- `normal_radius=0.1` is 5x the voxel size, large enough for PCA to fit a local
  plane but not so large that it blends different surfaces.
- `clustering_eps=0.10` was chosen after the initial 5cm idea was too strict for
  the real cleaned Eagle cloud with `min_points=50`.

The important part is not pretending those values are universal. They are sane
starting points for this dataset, and the comments explain how to tune them.

### 4. Normal estimation made me slow down and think about local geometry

A point cloud is just coordinates until normals are attached. For each point,
Open3D looks at nearby points, fits a local plane using PCA, and takes the least
spread direction as the surface normal. That only works if the neighbourhood is
large enough to contain a surface patch, which is why `normal_radius` must be
larger than `voxel_size`.

There is also the sign issue: PCA gives an axis, not a guaranteed outward
direction. `orient_normals_consistent_tangent_plane(100)` is there so adjacent
patches mostly agree instead of randomly flipping direction.

### 5. "Euclidean clustering" deserved a real implementation

Open3D has `cluster_dbscan()`, and DBSCAN is distance-based, so it is tempting to
use that and move on. But the assignment says Euclidean clustering, which in
robotics usually means the PCL-style connected-component algorithm:

1. Build a KD-tree.
2. Start from an unvisited seed point.
3. Radius-search neighbours within the tolerance.
4. Grow the component using BFS.
5. Keep components above the minimum cluster size.

That is what `ClusterExtractor.extract_euclidean_clusters()` implements. DBSCAN
is still available as an optional comparison helper, but the required path uses
KD-tree radius expansion explicitly.

### 6. Cluster output needed interpretation, not just colors

The Eagle sculpture is mostly one physically connected object, so the biggest
cluster is expected to dominate the colored render. That does not mean clustering
failed. It means the main body is connected, while smaller valid components
above the 50-point threshold are separated and saved individually.

That is why the pipeline writes both:

- a combined colored scene render
- four viewpoints of the same colored scene
- individual `cluster_00.png` to `cluster_04.png`
- `cluster_summary.json` with point counts, bounding boxes, extents, and centroids

The JSON summary matters because it makes the visual result auditable.

### 7. Renders were part of the assignment, not decoration

The PDF says the PCD files are too large and render images are sufficient. That
means the renders are not optional screenshots; they are the review artifact.
Getting them readable took real iteration:

- GitHub/CI is headless, so `matplotlib.use("Agg")` has to happen before
  importing `pyplot`.
- The Eagle scan is effectively Y-up, while matplotlib is Z-up, so the render
  path applies a render-only `Rx(-90°)` rotation to make the eagle upright.
- Matplotlib auto-scales axes independently, which can stretch the object. The
  visualizer locks X/Y/Z to one shared physical range.
- Dense clouds need tiny points; tiny clusters need larger points. Point size is
  adaptive so both the 314k-point main body and small clusters remain visible.
- The default `tab20` palette looked muted on dark backgrounds, so cluster colors
  are manually chosen to stay bright and readable.
- README images are embedded inline instead of linked, because the reviewer
  should see the outputs immediately.

### 8. Tests made the claims concrete

The required tests were downsampling and clustering, but I added more because
each extra test locked down a specific claim:

- downsampling actually reduces point count
- larger voxels produce fewer points
- SOR removes distant outliers but preserves most inliers
- crop never adds points
- cropped points stay inside the padded AABB
- `crop_padding=0.0` is a no-op
- normals exist, match point count, and are finite
- clustering produces more than one segment
- cluster colors preserve point count and stay inside `[0, 1]`
- tiny components below min size become noise

Using synthetic clouds was intentional. Unit tests should not depend on
downloading a 50MB Eagle file or on network state.

### 9. CI and repo hygiene had their own little traps

A local script passing is not the same as a reviewer being able to run it. I had
to make the repo boring in the best way:

- `.gitignore` excludes virtualenvs, caches, `.pcd`, `.ply`, `.npy`, and `.npz`.
- `requirements.txt` is one dependency per line.
- GitHub Actions installs the system libraries Open3D needs on Ubuntu.
- The UML exists as both `.drawio` and `.png`, because not everyone will open a
  draw.io file manually.
- Old render filenames were removed so `docs/renders/` only shows the final
  canonical outputs used by the README.

### 10. The advanced pipeline is extra, but it is separated cleanly

The core assignment is satisfied by `src.pipeline`. `AdvancedPipeline` is kept in
its own file so it does not complicate the required path. It adds registration,
Poisson reconstruction, and feature-edge detection as a product-adjacent
extension: useful for thinking about scan alignment and spray-paint coverage,
but not necessary to run the base assignment.

### What I would improve next

| Improvement | Difficulty | Why I did not add it yet |
|---|---:|---|
| CLI config overrides like `--voxel-size` and `--cluster-tolerance` | Easy | Useful, but the assignment is clearer when the default `Config` is the single source of truth. |
| Stage timing / `pipeline_metrics.json` | Easy to medium | Helpful for profiling, but it adds another generated artifact to keep in sync. |
| Systematic voxel/tolerance benchmark grid | Medium | Needs multiple long Eagle runs and a fair scoring method, not just a quick table. |
| BFS clustering vs DBSCAN runtime comparison | Medium | Valuable, but it should be measured carefully on the same cleaned/cropped cloud. |
| `pytest-cov` badge | Easy locally, medium in CI | Coverage is useful, but adding a badge means maintaining one more CI/reporting path. |
| CLI + benchmark scripts together | Medium | Best done as a follow-up once the submission pipeline stays stable. |

---

## Latest Verified Results

```text
Base pipeline:
796,825 raw pts → 329,988 downsampled → 316,519 cleaned → 314,626 cropped → 5 clusters extracted

Advanced pipeline:
ICP fitness       = 1.0000
ICP RMSE          = 0.000000
Poisson triangles = ~1.04M  (depth=9)
Feature edges     = ~98k segments above 20 degrees

Tests:
19 passed
```

---

## Tests

```bash
pytest
pytest -v          # verbose, shows individual test names
```

| Test file | What it covers |
|---|---|
| `tests/test_preprocessing.py` | Downsampling (4 tests), SOR (2 tests), Crop (4 tests) |
| `tests/test_cluster_extractor.py` | Multi-segment output, exact count, sort order, color validity, noise labels, JSON summary |
| `tests/test_normal_estimator.py` | Normals exist and are finite after estimation, empty-cloud guard |

Tests use synthetic in-memory point clouds — no Eagle download needed, no network calls, deterministic across runs.

### Why the Unit Tests Use Synthetic Point Clouds

The Eagle scan is the real assignment dataset, but it is not ideal for unit
tests. Unit tests should be small, deterministic, and focused on one behaviour
at a time. If every test downloaded or processed the full Eagle cloud, the suite
would be slower, network-dependent, and harder to debug.

So the tests generate tiny in-memory point clouds with NumPy:

- random cubes for downsampling and cropping
- Gaussian blobs for clustering
- a near-flat plane for normal estimation
- dense inlier cores plus faraway points for outlier removal

That lets each test ask one clean question: "Did this method keep its promise?"
The full Eagle dataset is still verified separately by running `python -m src.pipeline`.

### Every Test, In Plain English

| Test | File | Synthetic data used | What it proves | If it failed, it would mean... |
|---|---|---|---|---|
| `test_reduces_point_count` | `test_preprocessing.py` | 10,000 random points in a cube | Voxel downsampling actually reduces point count, which is a direct assignment requirement. | `voxel_down_sample()` is not being called correctly, or the voxel size is too small to merge points. |
| `test_output_is_not_empty` | `test_preprocessing.py` | 5,000 random cube points | Downsampling reduces density without deleting the entire cloud. | The voxel settings or preprocessing call destroyed all geometry. |
| `test_larger_voxel_gives_fewer_points` | `test_preprocessing.py` | Same cloud downsampled with `0.05m` and `0.20m` voxels | Larger voxels merge more space into fewer occupied cells. | The downsampling behaviour is backwards or the comparison setup is wrong. |
| `test_points_stay_within_original_bounds` | `test_preprocessing.py` | 10,000 random cube points | Downsampled centroids stay inside the original bounding box. | Voxel centroids are being generated outside the source geometry, which would corrupt spatial structure. |
| `test_removes_some_points_from_noisy_cloud` | `test_preprocessing.py` | Dense Gaussian core plus faraway outliers | Statistical Outlier Removal removes obvious floating noise. | SOR is not filtering distant ghost points. |
| `test_preserves_most_inlier_points` | `test_preprocessing.py` | 1,000 inliers plus 50 outliers | SOR is not too aggressive; it keeps at least 900 valid inlier points. | The filter settings are shaving away real surface data, not just noise. |
| `test_crop_never_increases_point_count` | `test_preprocessing.py` | 5,000 random cube points | Cropping can only remove points, never create new ones. | The crop method is returning invalid output or mixing data incorrectly. |
| `test_crop_removes_boundary_points_with_large_padding` | `test_preprocessing.py` | 10,000 random cube points with large padding | A large crop padding visibly shrinks the cloud. | `crop_padding` is not actually affecting the ROI. |
| `test_cropped_points_stay_inside_padded_aabb` | `test_preprocessing.py` | Random cube points with known padded bounds | Every surviving point is inside the expected Axis-Aligned Bounding Box. | The crop bounds math has a boundary or sign error. |
| `test_zero_padding_preserves_all_points` | `test_preprocessing.py` | 3,000 random cube points | `crop_padding=0.0` behaves as a no-op. | The crop stage removes points even when explicitly told not to. |
| `test_normal_estimation_adds_one_finite_normal_per_point` | `test_normal_estimator.py` | A near-flat synthetic plane | Normal estimation attaches exactly one finite normal vector per point. | Open3D normal estimation failed, produced invalid values, or returned the wrong normal count. |
| `test_normal_estimation_rejects_empty_cloud` | `test_normal_estimator.py` | Empty `PointCloud` | Empty inputs fail clearly instead of silently continuing. | The pipeline could produce meaningless outputs from an empty cloud. |
| `test_produces_more_than_one_cluster` | `test_cluster_extractor.py` | Two separated Gaussian blobs | Clustering produces more than one segment, which is the exact assignment test requirement. | The clustering tolerance is too large, or cluster extraction is not separating disconnected components. |
| `test_finds_exactly_two_blobs` | `test_cluster_extractor.py` | Two clean blobs far apart | The Euclidean cluster extractor returns exactly the two expected components. | The algorithm is either merging separate blobs or splitting one blob into fragments. |
| `test_clusters_sorted_largest_first` | `test_cluster_extractor.py` | One large blob and one smaller blob | Clusters are sorted so the main object appears first. | Downstream code might inspect a tiny fragment before the main object. |
| `test_colored_cloud_preserves_point_count` | `test_cluster_extractor.py` | Two separated blobs | Coloring clusters only adds RGB values; it does not add/remove geometry. | The visualization step is mutating the point cloud incorrectly. |
| `test_colored_cloud_has_valid_rgb` | `test_cluster_extractor.py` | Two separated blobs | Every point has an RGB color in Open3D's valid `[0, 1]` range. | Renders may appear blank, clipped, or invalid because color values are wrong. |
| `test_labels_mark_unkept_noise_as_minus_one` | `test_cluster_extractor.py` | One valid blob plus one tiny isolated speck | Tiny components below `clustering_min_points` become noise label `-1`. | Scanner speckles could be reported as real clusters. |
| `test_cluster_summary_contains_geometry_stats` | `test_cluster_extractor.py` | Two separated blobs | Cluster summaries include point count, bounding box min/max, extent, and centroid. | The JSON summary would not be useful for debugging or reviewer inspection. |

### What GitHub Actions / CI Tests

GitHub Actions runs the workflow in `.github/workflows/ci.yml` on every push to
`main` and on pull requests targeting `main`.

| CI step | What it does | Why it matters |
|---|---|---|
| `actions/checkout@v7` | Pulls the repository into a fresh GitHub-hosted Ubuntu machine. | Proves the repo can be tested outside my laptop. |
| `actions/setup-python@v6` | Installs Python 3.11. | Matches the local development runtime closely. |
| Install system dependencies | Installs packages such as `libgl1`, `libegl1`, `libgomp1`, `libglib2.0-0`, `libsm6`, `libxext6`, `libxrender1`. | Open3D needs these native libraries on Linux, even when tests are headless. |
| `pip install -r requirements.txt` | Installs Open3D, NumPy, Matplotlib, and Pytest. | Proves the dependency file is enough to set up the project. |
| `MPLBACKEND=Agg python -m pytest tests/ -v` | Runs all 19 unit tests without opening a GUI window. | Proves preprocessing, normal estimation, clustering, coloring, and summaries still work after each push. |

The CI intentionally does **not** run the full Eagle pipeline. That would make
every push download/process/render a large point cloud and would slow CI down
unnecessarily. The full real-data verification is done locally with:

```bash
python -m src.pipeline
python -m src.advanced_pipeline
```

So the testing strategy is split cleanly:

| Layer | Command | Purpose |
|---|---|---|
| Unit tests | `pytest -v` | Fast synthetic checks for specific behaviours. |
| Core real-data pipeline | `python -m src.pipeline` | Verifies the required assignment pipeline on the actual Eagle dataset. |
| Advanced real-data pipeline | `python -m src.advanced_pipeline` | Verifies optional registration, Poisson reconstruction, and feature-edge stages. |
| GitHub CI | GitHub Actions `Tests` workflow | Confirms the unit test suite passes in a clean Linux environment after every push. |

---

## Repository Structure

```text
augmentus-point-cloud/
├── src/
│   ├── config.py              # All tunable parameters in one dataclass
│   ├── loader.py              # Eagle dataset loading
│   ├── preprocessor.py        # Downsample + SOR + AABB crop
│   ├── normal_estimator.py    # PCA normal estimation
│   ├── cluster_extractor.py   # Euclidean BFS clustering
│   ├── visualizer.py          # Headless PNG rendering
│   ├── pipeline.py            # Core assignment pipeline
│   └── advanced_pipeline.py   # Registration + Poisson + edge detection
├── tests/
│   ├── test_preprocessing.py
│   ├── test_cluster_extractor.py
│   └── test_normal_estimator.py
├── docs/
│   ├── renders/               # All PNG outputs (generated at runtime)
│   │   └── clusters/          # Per-cluster individual renders
│   ├── uml/
│   │   ├── class_diagram.drawio
│   │   └── class_diagram.png
│   ├── assignment_traceability_matrix.md  # Maps every requirement to code/docs
│   ├── architecture_design_notes.md       # OOP decisions and design rationale
│   └── final_submission_checklist.md
├── .github/workflows/ci.yml   # GitHub Actions: runs pytest on every push
├── requirements.txt
└── .gitignore                 # Excludes *.pcd, *.ply, venv, __pycache__
```

---

## Why No PCD/PLY Files

The assignment notes that PCD files are too large to upload. The Eagle scan is ~50MB. The pipeline uses `o3d.data.EaglePointCloud()` which downloads and caches the file automatically on first run — no local copy needs to be committed. `.gitignore` excludes `*.pcd`, `*.ply`, and the Open3D data cache directory.

---

## Why Open3D

Open3D's Python API is the practical choice for this assignment:

- `o3d.data.EaglePointCloud()` is the required dataset source — no other library provides this.
- `PointCloud` objects store points, normals, and colors in a consistent structure that the whole pipeline can pass around without format conversion.
- `KDTreeFlann` gives fast radius search, which is the core primitive for both normal estimation and the Euclidean cluster BFS.
- The Agg matplotlib backend (set before any pyplot import) makes rendering headless-safe on CI without a display manager.

---

## Advanced Pipeline

`AdvancedPipeline` inherits from `Pipeline` and adds:

1. **FPFH + RANSAC global registration** — rough alignment from a cold start, before ICP.
2. **Point-to-plane ICP** — local refinement using target surface normals for stable convergence on smooth geometry.
3. **Poisson surface reconstruction** (depth=9) — converts the oriented point cloud to a watertight mesh.
4. **Dihedral-angle feature edge detection** — finds sharp mesh transitions (>20°) relevant to spray-paint nozzle path planning.

These stages are framed around coverage planning for the Eagle sculpture: registration simulates aligning two scans, Poisson gives a continuous surface to reason about, and feature edges flag corners, feather ridges, and plinth boundaries where a spray nozzle needs slower motion or extra overlap.

```bash
python -m src.advanced_pipeline
```

The core assignment is fully satisfied by `src/pipeline.py`. The advanced pipeline is a separate file so the required path stays simple.
