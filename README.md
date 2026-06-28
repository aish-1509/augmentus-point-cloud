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

### Advanced Pipeline Renders (optional)

| Registered | Poisson Mesh | Feature Edges |
|---|---|---|
| <img src="docs/renders/06_registered.png" width="280"> | <img src="docs/renders/07_poisson_mesh.png" width="280"> | <img src="docs/renders/08_feature_edges.png" width="280"> |

---

## Architecture

**UML class diagram:** [docs/uml/class_diagram.drawio](docs/uml/class_diagram.drawio)
*(Open in [diagrams.net](https://app.diagrams.net) or the draw.io VS Code extension)*

<img src="docs/uml/class_diagram.png" width="900">

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

### The crop wording looked small but actually mattered

The assignment says: *"Estimate surface normals for the cropped point cloud."*

It would have been easy to just downsample, run SOR, and call that preprocessing. A lot of pipelines do exactly that. But the PDF used the word "cropped" specifically, and I didn't want the implementation to be ambiguous about it.

So `crop_to_roi()` is its own named method in `Preprocessor`. The pipeline calls it explicitly, saves a separate render for it (`03_cropped.png`), and Stage 4 (normals) only ever receives the cropped cloud as input. The comment in `pipeline.py` says exactly why: the assignment requested this ordering, so the code makes it visible rather than hiding it inside a generic `preprocess()` call.

### "Euclidean clustering" vs just using DBSCAN

Open3D ships `cluster_dbscan()` out of the box, and it would have been easy to use it and call it Euclidean clustering because the eps parameter is a Euclidean distance. But the assignment wording is "Euclidean clustering," which in robotics typically means the PCL-style algorithm:

1. Build a KD-tree.
2. Pick an unvisited seed point.
3. Find all neighbours within the tolerance radius.
4. Grow the connected component using a queue (BFS).
5. Keep components larger than `min_cluster_size`.

That's what `ClusterExtractor.extract_euclidean_clusters()` does — explicit BFS over a `KDTreeFlann`, not DBSCAN. DBSCAN is still available as `run_dbscan_optional()` for comparison, but it's not the pipeline's default.

### Rendering was actually harder than the processing

The PCD files can't be committed (the assignment says so, and they'd be ~50MB anyway). So the PNG renders are what the reviewer actually sees. Getting those to look clear required more iteration than I expected:

- Matplotlib is Z-up. The Eagle dataset is Y-up. Passing the data straight to matplotlib lays the eagle on its side regardless of camera angle. The fix is a render-only `Rx(-90°)` rotation applied to a copy of the numpy array — the pipeline cloud never gets touched.
- Matplotlib auto-scales X/Y/Z independently. A wide point cloud gets stretched vertically, making it look wrong. The fix: compute the max physical range across all axes and force all three limits to use it.
- `tab20` looks fine on white backgrounds but the muted mid-tones disappear on dark canvas. The cluster palette is hand-picked for dark backgrounds — fully saturated, maximum brightness.
- Four viewpoints (iso, front, side, top) instead of just one, because a single angle can hide clusters stacked in depth.

### Tests forced the pipeline to be less vague

The assignment asks for two specific tests: downsampling reduces point count, and clustering produces more than one segment. Fine, those were easy. But writing the crop tests forced me to be explicit about what `crop_to_roi()` is actually supposed to do:

- It can't add points (obvious, but testable).
- Survivors must be inside the padded AABB (this caught a bug during development where `crop_padding=0.0` triggered a different code path).
- Zero padding is a no-op, not an error.

The normal estimation tests required creating a synthetic point cloud with enough local geometry for PCA to produce stable normals. A flat grid works. A random uniform cloud doesn't — the normals come out random.

### What I would improve next

- Benchmark different voxel sizes and cluster tolerances systematically, not just by eyeballing the renders.
- Add CLI config overrides so `python -m src.pipeline --voxel-size 0.01 --tolerance 0.08` works without editing `config.py`.
- Profile the BFS clustering on the full Eagle cloud and compare runtime vs DBSCAN.
- Add a `pytest-cov` badge showing actual line coverage.

---

## Latest Verified Results

```text
Base pipeline:
796,825 raw pts → 329,988 downsampled → 316,519 cleaned → 314,626 cropped → 5 clusters extracted

Advanced pipeline:
ICP fitness       = 1.0000
ICP RMSE          = 0.000000
Poisson triangles = 1,042,152  (depth=9)
Feature edges     = 97,977 segments above 20 degrees

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
│   └── uml/
│       ├── class_diagram.drawio
│       └── class_diagram.png
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
