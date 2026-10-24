# Dataset Workflow

This project now uses the following dataset workflow:

```text
data-pipeline/
  collection/
    train/
      raw_video/
      raw_picture/
    test/
      raw_video/
      raw_picture/
  labeling/
    classify_from_raw_picture.py
    rename_pic.py
    unnamed_picture_train/
    unnamed_picture_test/
  tools/
    frame_extract.py
```

Use only the new directories for future work:

- Training videos: `collection/train/raw_video`
- Training frames: `collection/train/raw_picture`
- Test videos: `collection/test/raw_video`
- Test frames: `collection/test/raw_picture`
- Training labeling workspace: `labeling/unnamed_picture_train`
- Test labeling workspace: `labeling/unnamed_picture_test`

The source project contains historical dataset folders, but they are not included here. Keep newly collected images in the `collection/` and `labeling/` paths above.

## 1. Extract Frames

Training set:

```powershell
cd .\data-pipeline\tools
python .\frame_extract.py --video-dir ..\collection\train\raw_video --picture-dir ..\collection\train\raw_picture --step 10
```

Test set:

```powershell
cd .\data-pipeline\tools
python .\frame_extract.py --video-dir ..\collection\test\raw_video --picture-dir ..\collection\test\raw_picture --step 20
```

## 2. Label Frames

Training set:

```powershell
cd .\data-pipeline\labeling
python .\classify_from_raw_picture.py --source-root ..\collection\train\raw_picture --target-root .\unnamed_picture_train
```

Test set:

```powershell
cd .\data-pipeline\labeling
python .\classify_from_raw_picture.py --source-root ..\collection\test\raw_picture --target-root .\unnamed_picture_test
```

## 3. Build Train/Test Datasets

Training set:

```powershell
cd .\data-pipeline\labeling
python .\rename_pic.py --input-dir .\unnamed_picture_train --output-dir ..\..\dataset_train
```

Test set:

```powershell
cd .\data-pipeline\labeling
python .\rename_pic.py --input-dir .\unnamed_picture_test --output-dir ..\..\dataset_test
```
