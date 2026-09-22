# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目定位

**元萝卜 AI 下棋机器人「四合一启蒙版」**的研究项目，非传统软件开发。并列两条工作线：

1. **产品研究**：拆解实物 + 竞品对标，产出**升级设计方案**（研究报告视角）
2. **毕业设计工程**：智能制造·智能设计方向——复刻该 SCARA 机械臂的数字样机（Fusion），并设计「末端工具 + 被动扶正附件」协同的自主取子机构（CAD/CAE 视角）

## 目录结构

```
docs/       报告与毕设文档（见下）
matlab/     SCARA 运动学仿真脚本(.m) + 输出图(.png)
fusion/     Fusion 360 三维模型（.f3d，当前为空，待搭）
shell_scan/ 外观扫描备件（photos/ 照片、mesh/ 网格），外观复刻用
pics/       实物照片
```

## 关键文档

- `docs/元萝卜四合一启蒙版_竞品分析与升级设计方案.docx` —— 产品研究报告（正式交付物，默认 Word）
- `docs/params.md` —— **机械手参数唯一来源**（实测数据：L1/L2、基座 (0,+230)、轴高 110、行程、棋子与棋盒规格）
- `docs/scara_fusion_spec.md` —— Fusion 建模底料：参数表 + 装配步骤 + **对标测点角度表**
- `docs/fusion_build_checklist.md` —— Fusion 建模可勾选步骤清单

## 工作环境

- **MATLAB R2025b** 已装且入 PATH，可无界面运行：在 `matlab/` 下 `matlab -batch "脚本名"`（中文输出在控制台可能乱码，数字正常）
- **Fusion 360** 已装（用户操作 GUI；AI 侧产出尺寸/步骤/角度，不代点）
- Ansys 模块可用性待确认（意图用于后续接触/物理仿真）

## 一致性约定

- 任何几何参数以 `docs/params.md` 为准；**改动参数必须同步**：`matlab/*.m` 里的常量、`docs/scara_fusion_spec.md` 的参数表与标定角度表
- 运动学约定：棋盘中心为原点、Z 向上、基座在 (0,+230)、θ1=0 指向棋盘中心(−Y)、θ2 单向折叠 0~180°

## 当前进度

- **a 阶段（进行中）**：Fusion 复刻「动得对」的 SCARA 数字样机，用标定角度自检 + 真机姿态对标做验收
- **b 阶段（挂起）**：接触/物理仿真，验证「扫子入槽→立正→取走」的扶正动作（纯运动学证不了）

## 工作视角

默认「研究分析 + 产品设计/工程设计」视角，而非「软件工程」；优先关注文档、图纸、参数表。正式交付默认 Word (.docx)。交流用中文。
