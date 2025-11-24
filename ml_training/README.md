# Training STGCN

> Warning! To follow these steps you need a labeled dataset with keypoints data.

## Installation

The model was designed, trained and exported using MMAction2 framework.
Here the process of it's installation. For more details see the [documentation](https://mmaction2.readthedocs.io/en/latest/get_started/installation.html).

```
docker build -f .ml_training/Dockerfile --rm -t mmaction2 .
```
```
docker run --gpus all -it -v $(pwd)/ml_training:/mmaction2/ml_training mmaction2
```

## Training

1. Download the checkpoint
```
curl -o stgcn_model.pth https://download.openmmlab.com/mmaction/v1.0/skeleton/stgcn/stgcn_8xb16-joint-u100-80e_ntu120-xsub-keypoint-2d/stgcn_8xb16-joint-u100-80e_ntu120-xsub-keypoint-2d_20221129-612416c6.pth
```

2. Prepare the dataset using `prepare_dataset.ipynb`

3. Train the model
```
python3 tools/train.py ml_training/stgcn_custom.py
```

## Exporting

Use `tools/deployment/export_onnx_gcn.py` and specify config and checkpoint to export in `.onnx` format.