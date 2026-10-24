# Model Training and Deployment

This folder contains the lightweight image-classification workflow used by the car project. It classifies an image ROI into `supplies`, `vehicle`, or `weapon`, and includes dataset-preparation scripts, training and evaluation code, and exported deployment models.

## Contents

| Path | Purpose |
| --- | --- |
| `train_tiny_classifier.py` | Train the compact PyTorch classifier and export ONNX |
| `evaluate_local_accuracy.py` | Evaluate a checkpoint on the independent test set |
| `evaluate_random_subset_accuracy.py` | Evaluate a sampled subset of the test set |
| `prepare_board_testset.py` | Prepare images for board-side evaluation |
| `convert_to_ncnn.ps1` | Convert an ONNX model to NCNN with `pnnx` or `onnx2ncnn` |
| `data-pipeline/` | Extract video frames, label images, and arrange class folders |
| `artifacts/` | Checkpoint, labels, metrics, ONNX, and NCNN deployment files |

The dataset directories are intentionally not included. Follow [`data-pipeline/README.md`](./data-pipeline/README.md) to prepare images, then place the resulting classes in `dataset_train/` and `dataset_test/` beside the training scripts. Each dataset should use one subdirectory per class, with folder names matching the model labels.

## Train and evaluate

Install the pinned dependencies in `requirements.txt`, then run from this directory:

```powershell
python -m pip install -r .\requirements.txt
python .\train_tiny_classifier.py --data-root .\dataset_train --test-root .\dataset_test --num-workers 0 --out-dir .\artifacts
python .\evaluate_local_accuracy.py --data-root .\dataset_train --test-root .\dataset_test --checkpoint .\artifacts\best_model.pt --num-workers 0
```

The training script uses a validation split from the training set for model selection and reports the independent test-set result separately. The checked-in `metrics.json` records 95.27% best validation accuracy and 60.17% independent test accuracy for the included checkpoint. Treat the test result as the more realistic estimate; the gap indicates that generalization still needs improvement.

## Deployment files

- `artifacts/best_model.pt`: PyTorch checkpoint
- `artifacts/tiny_classifier_fp32.onnx`: ONNX export
- `artifacts/tiny_classifier_fp32.ncnn.param` and `.bin`: NCNN export
- `artifacts/labels.txt`: output class order
- `artifacts/metrics.json`: training and evaluation metadata

The original source project contains the Windows `pnnx.exe` converter, but it is not copied here. Install or provide `pnnx`/`onnx2ncnn` separately when regenerating NCNN files; the existing exports are included.

