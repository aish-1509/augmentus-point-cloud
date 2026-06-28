# Assignment Traceability Matrix

Every requirement from the *Coding Test: Point Cloud Processing* PDF is mapped to
a specific file, method, or README section below.

| Assignment wording | Evidence in repository | Verification command | Status |
|---|---|---|---|
| "Implement a project that performs 3D point cloud processing using Open3D" | `requirements.txt`, all `import open3d` in `src/` | `grep -R "import open3d" src/` | ✅ |
| "including filtering, clustering, and surface normal estimation" | `Preprocessor.remove_outliers()`, `ClusterExtractor.extract_euclidean_clusters()`, `NormalEstimator.estimate()` | `grep -R "remove_statistical_outlier\|extract_euclidean\|estimate_normals" src/` | ✅ |
| "clean, testable, and well-documented code" | Type hints, docstrings, inline reasoning comments throughout `src/`; 19 unit tests in `tests/` | `pytest -q` | ✅ |
| "Create a public GitHub repository" | `https://github.com/aish-1509/augmentus-point-cloud` | Repository is public on GitHub | ✅ |
| "initialize a project in either C#, Python, or C++" | Python project with `src/` package layout | `python --version && ls src/` | ✅ |
| "Use Open3D as the point cloud processing library" | `requirements.txt`: `open3d>=0.18.0`; all stages use `o3d.*` | `grep "open3d" requirements.txt` | ✅ |
| "Input: Use `o3d.data.EaglePointCloud()`" | `src/loader.py` — `Loader.load_eagle()` calls exactly this | `grep "EaglePointCloud" src/loader.py` | ✅ |
| "source code, tests, and documentation" committed | `src/`, `tests/`, `docs/`, `README.md` all committed | `git ls-files src tests docs README.md` | ✅ |
| "Load the Eagle point cloud dataset" | `Loader.load_eagle()` — downloads on first run, caches in `~/open3d_data/` | `grep -n "load_eagle\|read_point_cloud" src/loader.py` | ✅ |
| "Apply a down-sampling filter" | `Preprocessor.downsample()` — voxel grid at `voxel_size=0.02m` | `grep -n "voxel_down_sample" src/preprocessor.py` | ✅ |
| "reduce the number of points while preserving geometric structure" | Voxel centroid averaging preserves spatial shape; test `test_reduces_point_count` verifies reduction | `pytest tests/test_preprocessing.py -v` | ✅ |
| "Estimate surface normals **for the cropped point cloud**" | `NormalEstimator.estimate()` is called only after `crop_to_roi()` in Stage 4 of `Pipeline.run()` | `grep -n "crop_to_roi\|estimate" src/pipeline.py` | ✅ |
| "Perform Euclidean clustering" | `ClusterExtractor.extract_euclidean_clusters()` — explicit KD-tree BFS radius expansion, not just DBSCAN | `grep -n "KDTreeFlann\|search_radius" src/cluster_extractor.py` | ✅ |
| "separate the scene into individual components" | Five distinct components above the 50-point threshold are extracted and saved | `cat docs/renders/cluster_summary.json` | ✅ |
| "Highlight and save each cluster separately or color them for visualization" | `05_clusters_colored.png` (full scene) + `clusters/cluster_00.png` … `cluster_04.png` | `ls docs/renders/clusters/` | ✅ |
| "Save renders of intermediate results" | `01_raw.png`, `02_downsampled.png`, `03_cropped.png`, `04_normals.png`, `05_clusters_colored.png` | `ls docs/renders/*.png` | ✅ |
| "downsampled cloud, normals, clusters" (renders) | `02_downsampled.png`, `04_normals.png`, `05_clusters_colored.png` all present | `ls docs/renders/02_* docs/renders/04_* docs/renders/05_*` | ✅ |
| "share on readme" | Every required render is embedded with `<img>` tags in README under "Render Gallery" | `grep "<img src=" README.md` | ✅ |
| "PCD files too large — render images are sufficient" | `*.pcd` and `*.ply` are excluded in `.gitignore`; note in README "Why No PCD/PLY Files" | `git ls-files | grep -E '\.(pcd|ply)$'` | ✅ |
| "Provide a UML class diagram" | `docs/uml/class_diagram.drawio` and `docs/uml/class_diagram.png` | `ls docs/uml/` | ✅ |
| "Loader, Preprocessor, NormalEstimator, ClusterExtractor" (named in UML) | All four classes present; UML PNG shows them with attributes and methods | `grep -n "class Loader\|class Preprocessor\|class NormalEstimator\|class ClusterExtractor" src/*.py` | ✅ |
| "Demonstrating OOP concepts are important" | README section "OOP Refresher and Design Thinking"; `AdvancedPipeline` inherits `Pipeline`; `Config` DI; private `_attrs` | `grep -n "class AdvancedPipeline\|super()" src/advanced_pipeline.py` | ✅ |
| "Include UML diagram in .drawio or .png" | Both formats present | `ls docs/uml/class_diagram.drawio docs/uml/class_diagram.png` | ✅ |
| "README with setup instructions" | "Quick Start" section — clone, venv, `pip install`, `pytest`, `python -m src.pipeline` | `grep -n "Quick Start\|git clone\|pip install" README.md` | ✅ |
| "How to run the program" | "Quick Start" + "Pipeline Overview" table in README | `grep -n "python -m src" README.md` | ✅ |
| "Description of code architecture and functionality" | "Architecture" section in README with class table; "OOP Refresher" section | `grep -n "## Architecture\|## OOP" README.md` | ✅ |
| "at least 2 unit tests" | 19 tests across three test files | `pytest --collect-only -q` | ✅ |
| "voxel down sampling reduces point count" | `TestVoxelDownsampling::test_reduces_point_count` | `pytest tests/test_preprocessing.py::TestVoxelDownsampling::test_reduces_point_count -v` | ✅ |
| "clustering produces more than one segment" | `TestClusterExtractor::test_produces_more_than_one_cluster` | `pytest tests/test_cluster_extractor.py::TestClusterExtractor::test_produces_more_than_one_cluster -v` | ✅ |
| "clear, incremental commits" | Git log shows component-by-component history: config → clustering → crop → renders → docs | `git log --oneline --max-count=20` | ✅ |
| "project setup then component-by-component" | Commit history: CI setup → config → visualizer → clustering → normals → renders → final polish | `git log --oneline` | ✅ |
| "source code" (deliverable) | `src/` package with 8 Python modules | `ls src/*.py` | ✅ |
| "unit tests" (deliverable) | `tests/` with 19 tests across 3 files | `pytest -q` | ✅ |
| "UML class diagram" (deliverable) | `docs/uml/class_diagram.drawio` + `docs/uml/class_diagram.png` | `ls docs/uml/` | ✅ |
| "README documentation and instructions" (deliverable) | `README.md` — full pipeline docs, renders, OOP, architecture, tests | `wc -l README.md` | ✅ |
| "Correctness of point cloud processing pipeline" | Pipeline runs end-to-end; produces 5 valid clusters from Eagle scan; all renders generated | `python -m src.pipeline` | ✅ |
| "Code quality and maintainability" | Type hints, docstrings, Config dataclass with inline rationale, no magic numbers | `grep -n "def \|: float\|: int\|: str" src/config.py` | ✅ |
| "Test quality and coverage" | Synthetic clouds for isolation; each test validates a specific geometric claim; 19 tests | `pytest -v` | ✅ |
| "Clarity and completeness of UML diagram" | UML shows Config, Loader, Preprocessor, NormalEstimator, ClusterExtractor, Visualizer, Pipeline, AdvancedPipeline with attributes and composition arrows | `docs/uml/class_diagram.png` | ✅ |
| "Overall project organization, documentation, and commit hygiene" | Clear `src/`, `tests/`, `docs/` layout; canonical render naming; `.gitignore` excludes build/cache/PCD artifacts | `git ls-files --others --ignored --exclude-standard` | ✅ |
| "easy to run" | Three commands: `pip install -r requirements.txt` → `pytest` → `python -m src.pipeline` | README "Quick Start" | ✅ |
| "well-structured" | One responsibility per class; Config injects into all; Pipeline composes without inheritance | `ls src/ tests/ docs/` | ✅ |
| "visually informative" | Dark-background renders, coordinate-corrected orientation, adaptive point size, multi-angle cluster views | `ls docs/renders/` | ✅ |
| "AI tools are allowed, but…reflect your thought process" | README "Development Notes: What I Had to Figure Out" section explains every non-obvious decision | `grep -n "Development Notes" README.md` | ✅ |

---

## Verified Pipeline Numbers

Run `python -m src.pipeline` to reproduce:

```text
796,825 raw pts
  → 329,988 after voxel downsample (0.02m)
  → 316,519 after SOR (nb=30, std=2.0)
  → 314,626 after AABB crop (padding=0.05m)
  → 5 valid Euclidean clusters (eps=0.10m, min_pts=50)
     cluster_0: 314,009 pts (99.8% — main Eagle body)
     cluster_1:     239 pts
     cluster_2:     105 pts
     cluster_3:      96 pts
     cluster_4:      54 pts
```

A 6th connected component with 37 points exists in the data but falls below the
`clustering_min_points=50` threshold. That boundary is a deliberate engineering decision:
50 points represents roughly a credit-card-sized surface patch at 2 cm voxel spacing.
Components smaller than that are more likely to be scanner edge artifacts than
meaningful geometry. The 5-cluster result reflects this choice clearly.

---

## Quick Cross-Check Commands

```bash
# 1. Tests pass
pytest -q

# 2. Required renders exist
ls docs/renders/01_raw.png docs/renders/02_downsampled.png \
   docs/renders/03_cropped.png docs/renders/04_normals.png \
   docs/renders/05_clusters_colored.png

# 3. Individual cluster renders
ls docs/renders/clusters/

# 4. UML exists in both formats
ls docs/uml/class_diagram.drawio docs/uml/class_diagram.png

# 5. No PCD/PLY committed
git ls-files | grep -E '\.(pcd|ply)$' || echo "clean — no PCD/PLY committed"

# 6. Pipeline runs
python -m src.pipeline
```
