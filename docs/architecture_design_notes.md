# Architecture Design Notes

This document explains the OOP architecture decisions made during this project
and how the design maps to the assignment's stated evaluation criteria.

---

## Design Intent

The assignment explicitly lists OOP as an evaluation criterion and recommends
drawing the UML *before* writing code. In practice, the design evolved — the
initial class split came from reading the assignment wording carefully, and the
final UML was confirmed and tidied once the implementation was stable.

That process is noted here honestly so the reasoning is visible, not hidden.

---

## Initial Architecture Sketch

Reading the assignment PDF produced a natural component split:

```
EaglePointCloud (Open3D dataset)
        │
     Loader
        │
   Preprocessor
        ├── downsample()      — reduce density
        ├── remove_outliers() — strip scanner noise
        └── crop_to_roi()     — carve spatial ROI
        │
 NormalEstimator
        │
 ClusterExtractor
        │
    Visualizer
        │
  docs/renders/ (PNG outputs)
```

`Config` sits alongside as the shared parameter holder, injected into each
class at construction rather than scattered as global variables or defaults
baked into every call site.

`Pipeline` is not in the component list above because it is not a *processor* —
it is the *coordinator* that sequences the calls above. Keeping the coordinator
separate means each processor can be tested without touching the others.

---

## OOP Concepts in Practice

### Encapsulation

Each class stores its tunable parameters as private attributes:

```python
class Preprocessor:
    def __init__(self, config: Config) -> None:
        self._voxel_size    = config.voxel_size      # private
        self._nb_neighbors  = config.sor_nb_neighbors
        self._std_ratio     = config.sor_std_ratio
        self._crop_padding  = config.crop_padding
```

External code calls `preprocessor.downsample(pcd)` without knowing or caring
that voxel size is stored as a private float. The `@property` on
`Visualizer.output_dir` gives read access without exposing the backing
`_output_dir` attribute.

### Abstraction

`Pipeline.run()` calls:

```python
raw        = self._loader.load_eagle()
downsampled = self._preprocessor.downsample(raw)
filtered   = self._preprocessor.remove_outliers(downsampled)
cropped    = self._preprocessor.crop_to_roi(filtered)
with_norms  = self._normal_estimator.estimate(cropped)
clusters   = self._cluster_extractor.extract_euclidean_clusters(with_norms)
```

`Pipeline` knows *what* happens at each stage but nothing about *how*.
It does not know about KDTreeSearchParamHybrid, deque-based BFS,
or matplotlib Agg backends.

### Composition (base pipeline)

`Pipeline` *has* a Loader, *has* a Preprocessor, *has* a NormalEstimator, etc.
None of those classes inherit from each other. Each is independently testable:

- `tests/test_preprocessing.py` tests `Preprocessor` in isolation with
  synthetic clouds — no Eagle download, no Pipeline, no Visualizer needed.
- `tests/test_cluster_extractor.py` does the same for `ClusterExtractor`.
- `tests/test_normal_estimator.py` does the same for `NormalEstimator`.

### Inheritance (advanced pipeline)

`AdvancedPipeline(Pipeline)` demonstrates inheritance as an extension mechanism,
not a workaround:

```python
class AdvancedPipeline(Pipeline):
    def run(self) -> dict:
        results = super().run()           # base pipeline unchanged
        results.update(self._register(results["raw"], results["cropped"]))
        results.update(self._reconstruct(results["with_normals"]))
        results.update(self._find_feature_edges(results["mesh"]))
        return results
```

The base pipeline is closed to modification but open to extension.
Adding registration or Poisson reconstruction does not require editing
`Pipeline` at all.

### Dependency Injection

A single `Config` dataclass is passed to every component at construction:

```python
def __init__(self, config: Config | None = None) -> None:
    config = config or Config()
    self._preprocessor     = Preprocessor(config)
    self._normal_estimator = NormalEstimator(config)
    self._cluster_extractor = ClusterExtractor(config)
```

Changing `Config(voxel_size=0.01)` propagates into every stage automatically.
No global state, no keyword arguments scattered across call sites, no default
values baked silently into methods.

### Single Responsibility

| Class | Owns | Does NOT own |
|---|---|---|
| `Config` | All numeric parameters | Any processing logic |
| `Loader` | Fetching the dataset | Any preprocessing |
| `Preprocessor` | Downsampling, SOR, crop | Normals, clustering, rendering |
| `NormalEstimator` | PCA normals + orientation | Preprocessing, rendering |
| `ClusterExtractor` | BFS Euclidean clusters, coloring, JSON summary | Normals, rendering |
| `Visualizer` | Headless PNG rendering | Any geometry processing |
| `Pipeline` | Call ordering and result collection | Any algorithm detail |

### Testability by Design

Because each class has exactly one job and receives all its configuration at
construction, any stage can be tested with a synthetic point cloud in memory:

```python
def test_reduces_point_count() -> None:
    pcd = _make_cloud(10_000)
    result = Preprocessor(Config(voxel_size=0.1)).downsample(pcd)
    assert len(result.points) < len(pcd.points)
```

No network calls. No Eagle download. No file I/O. No shared state between tests.

---

## Why Crop Is an Explicit Stage

The assignment says: *"Estimate surface normals for the **cropped** point cloud."*

That wording matters. It would be easy to treat preprocessing as just
`downsample + remove_outliers` and then immediately estimate normals. That would
technically touch a "cleaned" cloud, but the assignment explicitly names the
cropped cloud as the input to normal estimation.

Making crop a separate method (`crop_to_roi`) and a separate pipeline stage
(Stage 3 in `Pipeline.run()`) makes the requirement unambiguous. Stages 4 and 5
both receive the result of `crop_to_roi()`, never the full downsampled cloud.

This is also why `03_cropped.png` is saved as its own render — to make the
crop stage visible to a reviewer without requiring them to read code.

---

## Why Euclidean Clustering, Not Just DBSCAN

Open3D ships `cluster_dbscan()`, which is density-based and distance-aware.
Using that would technically produce clusters. But the assignment says Euclidean
clustering, which in robotics and point-cloud processing usually refers to the
PCL-style algorithm: KD-tree radius search + BFS/DFS connected-component growth.

`ClusterExtractor.extract_euclidean_clusters()` implements that explicitly:

1. Build `KDTreeFlann(pcd)`.
2. Pick an unvisited seed point.
3. Radius-search neighbours within `clustering_eps`.
4. Grow the component with a deque-based BFS.
5. Keep components above `clustering_min_points`.

DBSCAN is kept as `run_dbscan_optional()` for comparison, but the main pipeline
calls the explicit Euclidean path. The README explains this choice and why the
DBSCAN helper exists.

---

## UML and Implementation Alignment

The final UML (`docs/uml/class_diagram.png`) was drawn to match the implemented
code, not the other way around. The honesty here: the UML was the architecture
map used to verify that responsibilities were clearly separated, that Config
was properly threaded through the composition, and that AdvancedPipeline's
inheritance arrow was the right relationship. It was part of the design
verification process, not a pre-commit artifact.

If this project were started again from scratch, the sketch above (the text
diagram in "Initial Architecture Sketch") would be the first thing committed,
and the draw.io file would follow before any `src/*.py` was written. The design
would be the same — the order of artifacts would be different.

---

## What Would Change at Larger Scale

| Change | Impact on current design |
|---|---|
| Multiple input datasets | `Loader` gets an abstract base; `EagleLoader` becomes one subclass |
| Pluggable clustering algorithms | `ClusterExtractor` gets an interface; Euclidean and DBSCAN become strategy implementations |
| Real-time streaming | `Pipeline` becomes a stateless factory; each stage processes a frame |
| CLI parameter overrides | `Config` replaces `@dataclass` with `click`/`argparse`-backed construction |

None of these require touching the existing algorithm code. That is the point
of keeping each class focused: extension paths stay local.
