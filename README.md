# 元萝卜 AI 下棋机器人「四合一启蒙版」研究

针对元萝卜 AI 下棋机器人四合一启蒙版的研究项目，包含两条工作线：

1. **产品研究**：拆解实物 + 竞品对标，产出升级设计方案（研究报告视角）
2. **毕业设计工程**：智能制造·智能设计方向——复刻 SCARA 机械臂的 Fusion 360 数字样机，设计「末端工具 + 被动扶正附件」协同的自主取子机构（CAD/CAE 视角）

## 目录结构

```
docs/        报告与毕设文档
matlab/      SCARA 运动学仿真脚本 (.m) 与输出图 (.png)
fusion/      Fusion 360 三维模型 (.f3d) 与 scara_addin 建模脚本
shell_scan/  外观扫描备件（照片、网格），外观复刻用
pics/        实物照片
```

## 关键文档

- `docs/元萝卜四合一启蒙版_竞品分析与升级设计方案.docx` —— 产品研究报告（正式交付物）
- `docs/params.md` —— 机械手参数唯一来源（实测 L1/L2、基座位置、轴高、行程、棋子与棋盒规格）
- `docs/scara_fusion_spec.md` —— Fusion 建模底料：参数表 + 装配步骤 + 对标测点角度表
- `docs/fusion_build_checklist.md` —— Fusion 建模可勾选步骤清单

## MATLAB 仿真

`matlab/` 下包含 SCARA 运动学全套脚本：正/逆运动学（`scara_fk.m`、`scara_ik.m`）、轨迹规划（`scara_traj.m`）、工作空间与盲区分析（`scara_workspace.m`、`scara_blind_zone.m`）、扶正附件包络校核（`scara_corrector_envelope.m`）、取放全流程演示（`scara_pick_place.m`）。

在 `matlab/` 目录下运行：

```bash
matlab -batch "verify"
```

## 运动学约定

- 棋盘中心为原点，Z 轴向上
- 基座位于 (0, +230)
- θ1 = 0 指向棋盘中心（-Y 方向），θ2 单向折叠 0~180°

任何几何参数以 `docs/params.md` 为准；改动参数须同步 `matlab/*.m` 常量与 `docs/scara_fusion_spec.md` 的参数表。
