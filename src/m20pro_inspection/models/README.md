# Inspection Models

The source model is kept on the x86_64 conversion workstation:

```text
best.pt
```

The RK3588 production artifact generated from it is:

```text
best_rk3588_fp16.rknn
```

104 contains only the RKNNLite runtime in:

```text
/home/user/m20pro_rknn_pydeps
```

It does not contain the Torch conversion environment. Rebuild the artifact on
the workstation with `scripts/convert_yolo_to_rknn.py`; see
`best_rk3588_fp16.json` for input format, tool versions, classes and hashes.

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
