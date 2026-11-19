# RM2027 基地图传算法稳定基线

RM2027 基地颈部装甲板的低带宽图传算法：**HSV/几何 ROI 检测 →
RG-SINGLE 退化回退 → 恒速 Kalman → 自适应 4:3 裁剪 → 320×240 →
QPMAP → 低延迟编码/分包 → 解码 2× 显示**。

[`docs/`](docs/) 和 [`assets/media/output/`](assets/media/output/) 中。

## 结果速览

| 项 | 值 |
|---|---|
| 最终 Kalman P95 \|τ\|（mid+fast） | **20.50 ms**（EMA 基线 74.50 ms，−72.5%） |
| 中心 E95 | **8.38 px**（EMA 19.61 px） |
| 静态抖动 RMS | 2.789 px（EMA 2.694 px，1.035×，门槛内） |
| 漏检 1–2 帧 E95 | **2.03 px**（EMA 23.60 px） |
| IoU 均值 | **0.907**（EMA 0.739） |
| RG-SINGLE 修复（102–155 帧） | 旧 SINGLE 41.7 px / 28 次跳变 → **7.7 px / 0 次** |
| 最终视频 | **76.135 kbps / 668 包 / 0 LOST** |
| C++ 与 Python 最终码流 | SHA256 **完全一致** |

已知回退：慢速 30–100 px/s 的 P95 为 119.74 ms（EMA 113.55），
机动组 43.01 ms（EMA 72.76）；详见预研报告。

## 目录

```
RM2027_base_model/
├─ README.md
├─ .gitignore
├─ docs/
│  ├─ 预研报告.md                 算法 pipeline 预研报告（V2.0）
│  └─ results/                    消融 CSV、合成图 GT、诊断图
├─ rm2027_roi/                   算法功能包（ROS2 C++）
│  ├─ package.xml / CMakeLists.txt
│  ├─ msg/                       RoiDetection.msg / RoiEstimate.msg
│  ├─ include/rm2027_roi/{core,detection,estimation,coding,ros}
│  ├─ src/{core,detection,estimation,coding,ros,main.cpp}
│  ├─ config/                    detector/kalman/coding/pipeline YAML
│  ├─ launch/roi_pipeline.launch.py
│  ├─ test/                      4 个 GTest 文件（17 项）
│  └─ tools/                     qpmap_encoder 源码、脚本与 Linux 二进制
├─ assets/
│  ├─ model/                     Blender、STL 与基地参考图
│  └─ media/
│     ├─ input/base_orbit.mp4    输入视频（1440×1080、30 fps、600 帧）
│     └─ output/                 最终视频、码流、并排/放大对比、截图
├─ docs/                         预研报告与结果 CSV/图
```

## 快速开始

需要 ROS2 Humble、OpenCV、Eigen3、cv_bridge；WSL Ubuntu 22.04 已验证。

```bash
cd /mnt/d/Pacific_doorlock/RM2027_base_model
source /opt/ros/humble/setup.bash
colcon build --packages-select rm2027_roi \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --packages-select rm2027_roi
colcon test-result --verbose
```

离线跑完整视频（含编码与 300 B 分包）：

```bash
./install/rm2027_roi/lib/rm2027_roi/offline_demo \
  --source assets/media/input/base_orbit.mp4 \
  --out /tmp/roi_out --frames 600 \
  --encoder rm2027_roi/tools/qpmap_encoder
```

ROS2 在线节点：

```bash
ros2 launch rm2027_roi roi_pipeline.launch.py
```

默认订阅 `/camera/image_raw`；话题与参数见
[`rm2027_roi/README.md`](rm2027_roi/README.md)。
