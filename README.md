# Point Cloud Processing Assignment

Repository for the Augmentus perception coding test.

The input is Open3D's built-in `o3d.data.EaglePointCloud()` dataset, so I am not planning to commit any `.pcd` or `.ply` files. The assignment also asks for rendered images of the important stages, which should keep the repo light while still making the results easy to inspect.

## What I Need To Build

- Load the Eagle point cloud using Open3D.
- Downsample it with a voxel filter and check that the point count drops.
- Estimate surface normals after preprocessing.
- Split the cloud into more than one component using Euclidean-distance clustering.
- Save visual outputs for the downsampled cloud, normals, and clusters.
- Include a UML diagram before or alongside the implementation, since the brief calls out OOP design.

## Current State

This first pass is just the repo shape. I wanted the project split up before writing the actual Open3D calls, because the grading brief puts weight on clean structure, tests, documentation, and commit history.

The next commits should be small and easy to follow:

1. UML/design sketch for the main classes.
2. Dataset loader for `EaglePointCloud`.
3. Preprocessing with voxel downsampling.
4. Normal estimation.
5. Clustering and cluster coloring/export.
6. Rendered output images and final README run instructions.

## Code Layout

- `src/loader.py` - load the Eagle dataset through Open3D.
- `src/preprocessor.py` - downsample and crop/filter the cloud before analysis.
- `src/normal_estimator.py` - estimate and orient normals.
- `src/cluster_extractor.py` - group points using distance-based clustering.
- `src/visualizer.py` - save or display the intermediate views.
- `src/pipeline.py` - run the whole flow in order.

The tests are split by behavior rather than by file name:

- `tests/test_preprocessing.py` will check that voxel downsampling reduces point count.
- `tests/test_clustering.py` will check that clustering produces more than one segment.

## Setup

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

## Run

The pipeline entry point will be:

```bash
python -m src.pipeline
```

This command is listed early so the final implementation has a fixed target, but the actual runnable pipeline is still pending.
