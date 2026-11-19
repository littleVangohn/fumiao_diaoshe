# rm2027_roi

RM2027 基地图传的 ROI 检测 / 时序估计 / 裁剪 QPMAP 算法包。按 ROS2
`ament_cmake` 功能包组织，核心算法不依赖 ROS，节点只做消息适配。

## 模块

```
include/rm2027_roi/
├─ core/
│  ├─ types.hpp              Lamp / Bar / RoiDetection / RoiEstimate / Qpmap
│  ├─ parameters.hpp         KalmanParams / RgParams
│  └─ roi_pipeline.hpp       检测 + 估计 + 裁剪的一体化 C++ 接口
├─ detection/
│  └─ rg_detector.hpp        HSV 绿灯/红条 + PAIR + RG-SINGLE 回退
├─ estimation/
│  └─ kalman_estimator.hpp   8 状态恒速 Kalman（Joseph / 方法感知 R / 门控）
├─ coding/
│  ├─ crop_qpmap.hpp         自适应 4:3 裁剪、20×15 QPMAP、归一化 ROI
│  └─ packetizer.hpp         300 B stream-packed 打包/重组
└─ ros/
   ├─ roi_detector_node.hpp
   ├─ roi_estimator_node.hpp
   └─ roi_crop_node.hpp
```

`src/` 与 `include/` 目录一一对应；`src/main.cpp` 是不依赖 ROS 的离线
demo；`src/ros/` 是三个解耦节点。

## ROS2 接口

本包同时包含 C++ 节点与自定义消息，不再需要独立接口包。
`msg/RoiDetection.msg` 和 `msg/RoiEstimate.msg` 由本包的
`rosidl_generate_interfaces` 生成，三个节点链接生成的 C++ typesupport。
生成头文件为 `rm2027_roi/msg/roi_detection.hpp`、
`rm2027_roi/msg/roi_estimate.hpp`，C++ 类型位于 `rm2027_roi::msg` 命名空间。
核心算法的数据类型仍在 `core/types.hpp`，不依赖 ROS 生成的消息。

| 消息 | 字段 |
|---|---|
| `RoiDetection` | `header, valid, cx, cy, w, h, method, confidence` |
| `RoiEstimate` | `header, valid, cx, cy, w, h, track_state, source, prediction_horizon_s, uncertainty, update_cost_ms` |

`method` 表示检测方法（如 `PAIR / RG_SINGLE / RG_LAMP`）；
`track_state` 为 `TRACKING / DEGRADED / LOST`；
`source` 为 `MEASUREMENT / PREDICTION / HOLD / NONE`。

| 节点 | 订阅 | 发布 |
|---|---|---|
| `roi_detector` | `image_raw` (`sensor_msgs/Image`) | `roi/detection` (`rm2027_roi/msg/RoiDetection`) |
| `roi_estimator` | `roi/detection` | `roi/estimate` (`rm2027_roi/msg/RoiEstimate`) |
| `roi_crop` | `image_raw`, `roi/estimate` | `roi/canvas` (`sensor_msgs/Image` 320×240), `roi/qmap` (`Int8MultiArray` 20×15), `roi/normalized` (`UInt8MultiArray` 4 B) |

检测节点输出 `method ∈ {PAIR, RG_SINGLE, RG_LAMP}` 和 `confidence`；
估计节点按来源调整观测噪声：PAIR 1×、RG_SINGLE 2×/conf、RG_LAMP 3×、
旧 SINGLE/LAMP 12×；非 PAIR 连续拒绝 8 次才重初始化。

## 构建与测试

```bash
cd /mnt/d/Pacific_doorlock/RM2027_base_model
source /opt/ros/humble/setup.bash
colcon build --packages-select rm2027_roi \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --packages-select rm2027_roi
colcon test-result --verbose
```

检查生成的接口：

```bash
ros2 interface show rm2027_roi/msg/RoiDetection
ros2 interface show rm2027_roi/msg/RoiEstimate
```

旧版外部使用者须将依赖改为 `rm2027_roi`，并更新消息类型、头文件与命名空间。
建议使用全新工作空间重新构建，避免旧接口安装产物残留。

17 项 GTest：Kalman 恒速/漏检/方法感知/协方差、裁剪/QPMAP/分包往返、
合成红绿灯几何、pipeline 回退与跟踪。

## 离线 demo

```bash
cd /mnt/d/Pacific_doorlock/RM2027_base_model
source install/setup.bash
./install/rm2027_roi/lib/rm2027_roi/offline_demo \
  --source assets/media/input/base_orbit.mp4 \
  --out /tmp/roi_out --frames 600 \
  --encoder rm2027_roi/tools/qpmap_encoder
```

输出 `canvas_320x240.yuv`、`qmap_320x240.bin`、`estimates.csv`、
`stream.h264`、`packets.bin`、`encoder.csv`。

在 600 帧 `base_orbit.mp4` 上实测：
`PAIR 447 / RG_SINGLE 95 / RG_LAMP 56 / NONE 2`，
`190,338 B / 76.135 kbps / 668 包`，与 Python 参考实现的 H.264 码流
SHA256 **完全一致**（`77d8386a…b87032`）。

## Launch

```bash
ros2 launch rm2027_roi roi_pipeline.launch.py
```

默认订阅 `/camera/image_raw`。参数在 `config/detector.yaml`、
`config/kalman.yaml`、`config/coding.yaml`、`config/pipeline.yaml`。

## 冻结参数

```yaml
q_pos: 1313.07
q_size: 0.000510016
r_pos: 25.3156
r_size: 0.016075
gate_chi2: 16.0
gate_reinit: 3
p0_vel: 283.212
p0_size_vel: 0.707658
hold: 2
```

RG 模板：`max_pairs=60, min_pairs=5, max_template_age=120,
cov_reg=0.0025, mahal_max=4.0, dy_max=0.8`。

## 边界

- 真实视频无人工逐帧 GT；3–5 s 的参考是“最后一次 PAIR 中心”，
  不是真值。合成图 GT 只覆盖单帧回退，不能标定 RG 的序列模板。
- 慢速 30–100 px/s 和机动组仍是已知回退区间，详见根目录预研报告。
- 编码器 `tools/qpmap_encoder` 是 Linux ELF；Windows 下请用 WSL，
  或运行 `bash tools/build_encoder.sh` 重新编译。
