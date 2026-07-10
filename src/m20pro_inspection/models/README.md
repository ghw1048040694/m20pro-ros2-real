# Inspection Models

The current short-term test model is:

```text
best.pt
```

`best.pt` is used by the `ultralytics` backend for quick validation on a
workstation or on 104 after Python dependencies are installed. On 104, those
dependencies are installed outside the workspace in:

```text
/home/user/m20pro_yolo_pydeps
```

The launch file injects that path only into the YOLO node. It also preloads
`/lib/aarch64-linux-gnu/libgomp.so.1` on 104 for the PyTorch runtime.

For field deployment on RK3588, prefer converting the model to RKNN and
override `model_path` to the RKNN artifact:

```text
playphone_bg_best_rk3588_int8.rknn
```

Optional class names are stored one per line:

```text
labels_zh.txt
```

The current `best.pt` class order is:

```text
0 未戴安全帽
1 未穿安全背心
2 跌倒
3 火灾
4 现场杂乱
5 配电箱打开
```

Model artifacts such as `*.pt`, `*.onnx`, and `*.rknn` are ignored by Git.

Dataset/training reference:

```text
https://github.com/liu-big/power_station_safety.git
```
