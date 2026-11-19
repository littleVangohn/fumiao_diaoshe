# 最终交付

| 文件 | 内容 |
|---|---|
| `kalman_rg_640x480.mp4` | 最终解码视频，600 帧、30 fps、640×480 |
| `kalman_rg_320x240.h264` | 最终编码码流，320×240、76.135 kbps、668×300 B |
| `compare_old_vs_rg_640x480.mp4` | 左旧 SINGLE、右 RG-SINGLE 并排 |
| `zoom_old_vs_rg_3to5s.mp4` | 3–5 s ROI 放大对比（蓝旧、红 RG） |
| `previews/` | 3.0/4.0/5.0 s 放大截图 |
| `kalman_rg_stats.json` | 码率、包数、方法计数、冻结参数 |

编码链路：自适应 4:3 裁剪 → 320×240 → QPMAP BG+10/FG−5/NEUT0 →
libx264 ABR 88k（B=0、lookahead=0、AQ=2、mbtree=0、单线程）→
300 B stream-packed → 重组 → 解码 → 2× 640×480。
