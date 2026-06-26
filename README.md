# Augmentus Point Cloud Processing

This repository is the working skeleton for the Perception Coding Assignment on point cloud processing.

The goal is to keep the implementation readable and modular: each file owns one stage of the pipeline, and the final `pipeline.py` script should connect those stages in a way that is easy to test and explain.

## Planned Pipeline

1. Load a point cloud from disk.
2. Preprocess the raw cloud with downsampling and filtering.
3. Estimate surface normals for local geometry.
4. Extract clusters from the processed cloud.
5. Visualize intermediate and final results.

## Repository Layout

```text
augmentus-point-cloud/
├── README.md
├── requirements.txt
├── .gitignore
├── src/
│   ├── __init__.py
│   ├── loader.py
│   ├── preprocessor.py
│   ├── normal_estimator.py
│   ├── cluster_extractor.py
│   ├── visualizer.py
│   └── pipeline.py
├── tests/
│   ├── __init__.py
│   ├── test_preprocessing.py
│   └── test_clustering.py
└── docs/
    └── uml/
```

## Module Responsibilities

- `loader.py`: read point cloud files and validate input paths.
- `preprocessor.py`: clean the cloud before downstream processing.
- `normal_estimator.py`: estimate normals for geometry-aware operations.
- `cluster_extractor.py`: group points into object-like clusters.
- `visualizer.py`: inspect clouds, normals, and clusters.
- `pipeline.py`: coordinate the full workflow from input to output.

## Setup

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

## Notes

The first commit only defines the project shape. Implementation, tests, and diagrams will be added incrementally so each stage can be reasoned about and verified on its own.
