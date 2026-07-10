# 106 边缘点云/激光链路可行性方案报告

日期：2026-07-07

## 目标

最终目标不是单纯让某个 topic 能 echo，而是让整条导航感知链路减负：

```text
106 读取原始 /LIDAR/POINTS
  -> 在 106 上变成轻量 scan 或轻量点云
  -> 104 只订阅轻量结果
  -> Nav2 更直接拿到 /scan
```

理想生产结果：

- 104 不再跨主机订阅原始大点云 `/LIDAR/POINTS`;
- 104 不再跑 `lidar_relay`;
- 104 不再跑 `pointcloud_fusion`;
- 104 的 DDS/SHM 压力明显下降；
- Nav2 使用更直接的二维激光输入；
- 旧链路必须保留为一键回退。

当前正式链路仍是旧链路：

```text
104: /LIDAR/POINTS
  -> lidar_relay
  -> /m20pro/lidar_points_relay
  -> pointcloud_fusion
  -> /scan
  -> Nav2/Web
```

## 已测试路线

### 方案 A：104 strict UDP-only 直接订阅原始点云

结论：不适合作为主线。

现象是 topic/graph 层面可以看到一些信息，但 raw `/LIDAR/POINTS` 样本不稳定或收不到。它没有解决“104 仍然要吃原始大点云”的根本问题，即使某次能通，也只是把传输方式换了一下，104 的大点云压力仍在。

### 方案 B：按旧经验恢复 106 普通 ROS2 `/LIDAR/POINTS`

结论：旧经验必须保留为排障手段，但不能把普通 rclpy 作为新主链路依赖。

旧的 104 恢复经验是：

```bash
ssh user@10.21.31.106
source /opt/robot/scripts/setup_ros2.sh
su
systemctl restart multicast-relay.service
```

同时检查 106 `/opt/robot/fastdds.xml` 中必须保留 `10.21.31.106` 白名单，`multicast-relay.service` 必须 enabled/active。

这套方法对“104 能看到 publisher 但 echo 没样本”的问题有效；6 月 9 日和第二台机器狗都靠它恢复过。但本轮在 106 上复核后发现：

- 106 DrDDS/原厂 DDS 层有 `/LIDAR/POINTS`;
- 普通 ROS2/rclpy 在某些 profile 下能看到 publisher，但订阅仍可能是 `0` 样本；
- 重启 `multicast-relay.service` 和 ROS daemon 后，普通 rclpy 入口仍不可靠。

因此，优先恢复 106 `/LIDAR/POINTS` 这条路已经测试过：如果指“普通 ROS2 订阅”，可行度不够；如果指“原厂 DrDDS 订阅”，可行度很高。

### 方案 C：106 普通 Python relay

结论：能作为历史参考，不建议作为正式方向。

7 月 2 日按正确 root ROS 环境进入 106 后，普通 Python relay 曾经能收到 `/LIDAR/POINTS` 并发布实验轻量点云，104 对比结果接近旧 relay。但它有两个问题：

- 依赖普通 ROS2/rclpy 入口，而该入口后来再次表现出不稳定；
- 临时 Python relay CPU 约 55.9%，不适合作为常驻生产节点。

所以这条路线证明“106 边缘处理有价值”，但不应直接生产化。

### 方案 D：106 原生 DrDDS 读取 `/LIDAR/POINTS`

结论：高可行度。

已验证命令形态：

```bash
drdds_lidar_probe /LIDAR/POINTS 0 0 rt 8
```

关键条件：

- topic 必须是带前导斜杠的 `/LIDAR/POINTS`;
- domain 为 `0`;
- prefix 为 `rt`;
- `use_shm=0`;
- 使用原厂 DrDDS 类型 `ChannelLidarPointCloud`；
- 不要把 ROS2 `sensor_msgs/msg/point_cloud2.hpp` 和原厂 dridl 类型混着用。

实测结果：

- 能收到约 10Hz 原始点云；
- frame 为 `lidar_link`;
- 点数随环境变化；
- 探针自身不依赖 FastDDS SHM。

### 方案 E：106 原生 DrDDS 直接生成实验 LaserScan

结论：当前最高可行度路线，已保留 demo。

推荐 demo 命令：

```bash
drdds_edge_scan_demo /LIDAR/POINTS /m20pro/scan_edge_exp 90 0 0 rt \
  -0.05 0.55 4 12000 m20pro_base_link 0.0174533 10.0 0.2
```

对应脚本：

```bash
tools/edge_scan_feasibility/run_balanced_demo_on_106.sh
```

104 侧对比结果：

```text
/scan                  rate≈3.65Hz finite_mean≈190.00 age_mean≈0.145 frame=m20pro_base_link
/m20pro/scan_edge_exp  rate≈3.36Hz finite_mean≈182.49 age_mean≈0.155 frame=m20pro_base_link
```

这说明它已经接近当前正式 `/scan` 的形态，可以作为下一轮电池充足后的重点验证对象。

2026-07-07 22:45 左右又做了一次短时复核：

- 106 普通 ROS2 入口：
  - `ros2 topic info -v /LIDAR/POINTS` 能看到 `Publisher count: 2`;
  - `ros2 topic echo /LIDAR/POINTS --no-arr` 6 秒超时，仍然没有样本；
  - 说明普通 ROS2 入口仍是“图可见、样本不可靠”。
- 106 DrDDS probe：
  - `/tmp/m20pro_edge_live_bin/drdds_lidar_probe /LIDAR/POINTS 0 0 rt 8`;
  - `samples=62`;
  - `rate_hz=10.1844`;
  - `frame=lidar_link`;
  - 说明原厂 DrDDS 点云入口仍可直接拿到样本。
- 106 DrDDS edge scan demo：
  - 旁路发布 `/m20pro/scan_edge_exp`，不发布 `/scan`;
  - 45 秒结果：`clouds=435`, `scans=143`, `rate_hz=3.32899`;
  - 104 侧 25 秒只读对比：

```text
/scan                  rate_hz=3.680 finite_mean=202.46 age_mean=0.144 frame=m20pro_base_link
/m20pro/scan_edge_exp  rate_hz=3.375 finite_mean=190.18 age_mean=0.157 frame=m20pro_base_link
```

这次 finite mean 差距约 6%，在短时只读测试里已经很接近正式 `/scan`。测试后 106 临时文件已删除，106 `/dev/shm` 回到约 27%，104 只剩正式 `/scan`，Web/API 仍为 `perception_ready`。

## 为什么不能直接从 `drddsctl` 获取点云

`drddsctl list` 适合判断原厂 DDS 图里有没有 publisher/subscriber，例如：

```text
Publisher -> rt/LIDAR/POINTS
Publisher -> rt/LIDAR/POINTS2
Publisher -> rt/LOC_BODY_POINTS
```

但它本质上是发现/诊断工具，不是稳定的数据接口。它能告诉我们“原厂 DDS 层有点云端点”，不能替代一个真正订阅并解析 `PointCloud2` 样本的节点。

真正要拿数据，需要类似现在 demo 里的方式：

```text
ChannelLidarPointCloud 订阅原厂 DrDDS 点云
  -> 解析 x/y/z
  -> 发布 LaserScan 或轻量 PointCloud2
```

所以 `drddsctl` 用来判活，`drdds_lidar_probe` / `drdds_edge_scan_demo` 用来取样本和生成可用数据。

## 推荐路线

推荐路线是：106 DrDDS edge scan。

阶段 1：保持旧链路不动，只旁路发布：

```text
106: DrDDS /LIDAR/POINTS -> /m20pro/scan_edge_exp
104: 同时订阅 /scan 和 /m20pro/scan_edge_exp 做对比
```

阶段 2：电池充足后做 5 分钟对比和短 bag：

- `/m20pro/scan_edge_exp` frame 必须是 `m20pro_base_link`;
- 频率稳定在约 3Hz 以上；
- finite ranges 与正式 `/scan` 差距控制在约 15% 内；
- age 均值低于约 0.30s；
- 正式 `/scan` 和前端状态不能变差。

阶段 3：把 106 demo 做成手动 service，不 enable 开机自启：

```text
m20pro-edge-scan-106.service
```

只要服务试验通过，再考虑让 Nav2 在受控测试中使用 edge scan 输入。

阶段 4：真正生产切换：

```text
M20PRO_SCAN_SOURCE=local_fusion | edge_scan
```

默认仍是 `local_fusion`。只有 edge scan 短任务导航成功、回退测试成功、104 SHM 明显下降后，才允许禁用 104 raw 点云、`lidar_relay` 和 `pointcloud_fusion`。

## 当前状态

已经完成：

- 多条路线测试与排除；
- 106 DrDDS 原生点云入口验证；
- 106 DrDDS 生成实验 LaserScan demo；
- 104 侧确认能收到 `/m20pro/scan_edge_exp` 并与正式 `/scan` 短时对比；
- 平衡参数 demo 保留；
- 本地 contract 支持 `perception_mode=edge_scan`，但默认仍为 `local_fusion`;
- `scan_topic` 参数已准备，默认仍为 `/scan`;
- 审计脚本确认 demo 没有接入生产启动路径。

尚未完成：

- 5 分钟长一点的 `/scan` 对比；
- 短 rosbag 复盘；
- 106 service 级试验；
- Nav2 使用 edge scan 的短任务测试；
- 104 停掉 raw 点云、`lidar_relay`、`pointcloud_fusion` 后的 SHM 实测。

## 最终判断

当前最高可行方案是：

```text
106 DrDDS /LIDAR/POINTS
  -> 106 edge scan
  -> 104 Nav2/Web
```

可行度：高。

但它还不是生产完成状态。它已经有可以保留的 demo，下一步必须在换电池后用 `NEXT_BATTERY_TEST_PLAN.md` 做短时验证，再进入 `SERVICE_TRIAL.md`。在这些门槛没过之前，104 正式链路继续使用旧的 `local_fusion` 回退方案。
