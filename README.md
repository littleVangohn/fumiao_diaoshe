# RM2027 基地图传算法稳定基线

RM2027 基地颈部装甲板的低带宽图传算法：**HSV/几何 ROI 检测 →
RG-SINGLE 退化回退 → 恒速 Kalman → 自适应 4:3 裁剪 → 320×240 →
QPMAP → 低延迟编码/分包 → 解码 2× 显示**。

算法部分已整理为 ROS2 Humble C++ 功能包，核心库与 ROS 节点解耦；
离线 demo 和 ROS2 launch 均可用。冻结参数与最终视频在
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

## 单一 ROS2 C++ 功能包

本项目只包含一个 ROS2 包 `rm2027_roi`。项目根目录是源码集合与构建工作目录，
`rm2027_roi/` 才是包含 `package.xml` 和 `CMakeLists.txt` 的功能包目录。
消息、核心算法、三个在线节点和离线 demo 均归属该包。

- `msg/RoiDetection.msg`：检测节点向估计节点传递检测框、方法与置信度。
- `msg/RoiEstimate.msg`：估计节点向裁剪节点传递估计框、跟踪状态与预测信息。
- 消息由同包的 `rosidl_generate_interfaces` 生成；三个 C++ 节点链接生成的 typesupport。
- 核心库 `rm2027_roi_core` 不依赖 ROS；离线 demo 不使用 ROS 消息，但本包的
  CMake 构建仍需要 ROS2 环境。

合包不改变三个节点通过话题通信的方式。完整消息字段见
[`rm2027_roi/README.md`](rm2027_roi/README.md)。

从旧版升级时，原 `rm2027_roi_msgs/msg/*` 消息类型改为 `rm2027_roi/msg/*`；
外部节点需同步修改依赖、头文件和命名空间并重新编译。请使用全新构建工作空间，
或在确认旧构建产物无需保留后清理工作空间的 `build/`、`install/`、`log/`，
再在仅加载 ROS2 Humble 的新终端中构建，避免加载旧接口。

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

## 最终交付

- [预研报告](docs/预研报告.md)
- [最终视频（640×480）](assets/media/output/kalman_rg_640x480.mp4)
- [最终码流（320×240 H.264）](assets/media/output/kalman_rg_320x240.h264)
- [旧/新并排对比](assets/media/output/compare_old_vs_rg_640x480.mp4)
- [3–5 s ROI 放大对比](assets/media/output/zoom_old_vs_rg_3to5s.mp4)
- [冻结参数](docs/results/kalman_params.json)
- [消融结果](docs/results/rg_ablation.csv)、
  [合成图 GT 对照](docs/results/rg_synthetic_eval.csv)

## 验证

- ROS2 C++ 功能包：`colcon build` 通过，17 项 GTest 全过。
- 2026-11-19 单包合并验证：WSL Ubuntu 22.04 / ROS2 Humble 构建通过；
  17 项测试无失败，两个 `rm2027_roi/msg/*` 接口均可通过 `ros2 interface show` 查看。
- C++ 离线 demo 在 600 帧输入上输出
  `PAIR 447 / RG_SINGLE 95 / RG_LAMP 56 / NONE 2`，
  `190,338 B / 76.135 kbps / 668 包`；
  与 Python 参考实现的 H.264 码流 SHA256 完全一致。
- 最终视频对应 `assets/media/output/kalman_rg_640x480.mp4`。

## 证据边界

- 正式时序指标来自合成解析真值；真实视频无人工逐帧 GT，
  3–5 s 的参考是“最后一次 PAIR 中心”，不是真值。
- 300 张合成图 GT 只覆盖单帧回退，不能标定 RG 的 PAIR 序列模板。
- 单段渲染绕轨视频、PC/WSL 离线环境；未验证真实相机、无线链路、
  目标硬件算力。
- 慢速段与机动组仍有回退，详见报告第 2、3 节。

## 上传 GitHub

在 Windows PowerShell 中首次上传（当前目录尚未初始化 Git）：

```powershell
cd D:\Pacific_doorlock\RM2027_base_model
ssh -T git@github.com
git init
git add .
git commit -m "Merge ROI messages and nodes into one ROS2 package"
git branch -M main
git remote add origin git@github.com:littleVangohn/fumiao_diaoshe.git
git push -u origin main
```

`ssh -T git@github.com` 只测试 SSH 身份认证，GitHub 不提供交互式 SSH shell；
`git@github.com:littleVangohn/fumiao_diaoshe.git` 是 Git 远程地址。
若已配置 `origin`，使用 `git remote set-url origin git@github.com:littleVangohn/fumiao_diaoshe.git`。
若远端已有提交，需先获取并整合远端历史，再推送；不要直接强制覆盖。
`assets/` 中的 STL/Blend 较大，
单文件均 <100 MB；若 GitHub 仍限制，请改用 Git LFS。
