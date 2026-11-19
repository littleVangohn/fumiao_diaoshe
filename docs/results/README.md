# 结果文件

| 文件 | 内容 |
|---|---|
| `kalman_test_summary.csv` | 第二轮 Kalman 测试集汇总（含 EMA 参考行） |
| `kalman_test_decision.json` | 冻结参数、门槛、改善量 |
| `kalman_params.json` | 冻结 Kalman 参数 |
| `rg_ablation.csv` | frame 102–155 旧/LAMP/RG-SINGLE 消融 |
| `rg_synthetic_eval.csv` | 300 张合成图 GT 对照 |
| `real_rg_descriptive.csv/json` | 真实视频无 GT 滞后代理 |
| `rg_ablation.png` | 102–155 帧 ROI 轨迹对比 |
| `roi_drift_3to5s.png` | 旧 SINGLE 左右横跳诊断 |

所有数据口径见根目录预研报告附录 B。真实视频无人工逐帧 GT，
`real_rg_descriptive` 只是“原始检测/最后一次 PAIR”参考下的描述性结果。
