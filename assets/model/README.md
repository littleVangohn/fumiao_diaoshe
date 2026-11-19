# RM2027 基地（吊射位）模型

从整场 STL `C:\Users\ty\Downloads\RMUC2027_V1.0.0.STL` 中按坐标裁出的红方基地。

## 坐标与尺寸（场地坐标，mm）

- 基地主体包围盒：x 1651–3267，y 7053–8950，z 497–1682
- 中心：x≈2459，y≈8001（场地的另一侧基地关于 x=14000 对称）
- 模型尺寸：1.09 × 1.19 × 1.185 m
- 高度 1185mm 与规则手册图 4-11 的 1181mm 对齐
- 三角面：597,350

## 裁切方法

1. 30mm 体素连通域标记，种子取在基地主体表面（label 28）
2. 取该连通域并向外膨胀 2 格，保留三角面
3. 按包围盒剔除跨界的整场地面
4. Blender 内按 loose parts 拆分 261 个部件，删除启动区地板（z < 0.52m）

## 材质（Blender 5.0 程序化，无贴图）

| 材质 | 用途 | 主要参数 |
|---|---|---|
| ArmorLight | 塔身三面主装甲 | 浅灰，metallic 0.74，rough 0.38 |
| Gunmetal | 顶部横梁、飞镖检测模块、颈部装甲板两端突出块 | 近黑，metallic 0.92，rough 0.29 |
| MidMetal | 下部侧舱、散热件 | 中灰，metallic 0.86 |
| BaseGraphite | 基地底座裙板 | 石墨灰，metallic 0.83 |
| GuideGreen | 顶部绿色引导灯 | 绿色自发光 |
| LEDRedNeck | 绿灯上下两块装甲板两侧梯形凹槽内的灯带件 | 红色自发光，强度 6 |

颈部灯带按规则手册《基地脖子_正视特写》布置：绿灯上方、下方两块装甲板左右两侧的梯形凹槽内各嵌一条红色竖灯带。STL 中对应的灯带件为上装甲板 `base_left_clean.041/.042`、下装甲板 `base_left_clean.040`；下装甲板右侧在 STL 中缺件，按镜像补出 `base_left_clean.040R`。装甲板两端的突出块 `.003/.004` 已改回 Gunmetal，颈部红色只保留在灯带件上。

## 文件

- `RM2027_base.stl` — 裁出的基地模型（mm，Z 向上）
- `RM2027_base_material.blend` — Blender 工程（模型 + 材质 + 灯光 + 相机）
- `RM2027_base_neck_front.png` — 颈部正视图（对照规则手册基地脖子特写）
- `RM2027_base_reference.png` — 基地整体参考图

> 早期的主图/顶部特写渲染已清理，需要时从 Blender 工程重新渲染。

渲染：Blender 5.0 Cycles GPU（OptiX），160 spp + 降噪，AgX。
