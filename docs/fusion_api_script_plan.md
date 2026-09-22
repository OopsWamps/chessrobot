# 用 Fusion API 脚本参数化搭 SCARA + 导出 Mirabuf（技术路线）

> 定位：a 阶段的**自动化替代方案**——不手点，用 Python 脚本从零生成参数化的 SCARA 骨架（含 Joint），再用 Synthesis 导出器导 Mirabuf，供 b 阶段物理仿真。
> 依据：官方 `FusionAPIReference`（API 文档/签名）＋ `Synthesis` 的 Fusion Exporter（导出器）。**API 细节以 FusionAPIReference 为准，本文件只给结构与关键调用。**

## 0. 为什么走 API 脚本，而不是 MCP
官方 `FusionMCPSample/tips.md` 自己写明：**MCP 擅长「读设计信息、改现有组件属性、辅助写 add-in」，但「从零画草图建组件」通常超出其能力范围**。而我们的任务正是**从零建** SCARA → 所以用 **API 脚本**；MCP 留作辅助调试（见第 6 节）。

## 1. 目标产物
- 一个 Fusion Python **Add-In/Script**，运行后自动生成：`base / link1 / link2 / slider / tool` 五个组件 + 三个 Joint（两个旋转副 + 一个滑动副，带限位）
- 改脚本顶部参数（如 L2=185）→ 模型随之更新
- 模型可直接被 **Synthesis Fusion Exporter** 导出成 Mirabuf

## 2. 前置
- Fusion 360 已装；API 环境（`adsk.core`、`adsk.fusion`）随 Fusion 提供
- Add-In 最小结构：文件夹 + `xxx.manifest` + `xxx.py`（含 `run(context)` / `stop(context)` 入口）
- 参考：`FusionAPIReference`（含 Python stub，查签名）、官方 `AddInSample`

## 3. 脚本结构（骨架）
```
PARAMS = { "L1":195, "L2":185, "base_y":230, "axis_h":110,
           "z_top":30, "d3_travel":35,
           "th1_lim":(-150,15), "th2_lim":(0,180) }

run(context):
  1. 取 rootComp = design.rootComponent
  2. 建基座 base：草图(圆, 圆心 (0,base_y)) → 拉伸高 axis_h
  3. 建 link1：长方体长 L1，起点接轴心
  4. 建 link2：长方体长 L2，起点接 link1 远端
  5. 建 slider + tool：竖直杆，行程 d3_travel，吸嘴口按 z_top 定位
  6. 建三个 Joint（关键，见第 4 节）
  7. 打印末端吸嘴口坐标（FK 自检用）
```
> 组件/Occurrence **起稳定名字**（base/link1/…），导出器按结构遍历，命名清晰便于后续认。

## 4. 关键：用 API 建 Joint（已核对 `FusionAPIReference` 的 Python stub）

### 4.1 建关节的调用链
```python
# 取关节集合
joints = rootComp.joints

# 用两处几何/原点创建 JointInput
# ⚠ 第一个 = 要动的组件，第二个 = 固定参考件（已实测：反了会报 grounded 错误）
inp = joints.createInput(geometryToMove, geometryReference)   # -> JointInput

# 指定运动类型（二选一）
inp.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)   # 旋转副
inp.setAsSliderJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)     # 滑动副
# 需要自定义轴时：传 customRotationAxisEntity / customSliderDirectionEntity

joint = joints.add(inp)          # -> Joint
# Joint 上也同名方法（建好后改类型）：joint.setAsRevoluteJointMotion(...) 等
```

### 4.2 关节几何怎么给（JointGeometry 静态构造器，选一个）
| 构造器 | 用途（本毕设对应） |
|---|---|
| `JointGeometry.createByPoint(point)` | 用一个构造点/草图点/顶点定位关节原点 |
| `JointGeometry.createByProfile(profile, sketchCurve, keyPointType)` | 用草图轮廓+曲线定原点（**推荐**：轴心/肘点用草图点最稳） |
| `JointGeometry.createByPlanarFace(face, edge, keyPointType)` | 平面+边定原点 |
| `createByCurve / createByNonPlanarFace / createByBetweenTwoPlanes / createByTwoEdgeIntersection` | 其他场景 |

对象也可直接传 **JointOrigin**（见 4.4）。

### 4.3 设限位（先拿到 motion，再拿 limits）
```python
motion = joint.jointMotion                              # -> JointMotion
rev = motion  # 旋转副应可转为 RevoluteJointMotion / 滑动副转 SliderJointMotion
lim = rev.rotationLimits                                 # 旋转副: rotationLimits
# 滑动副: lim = motion.slideLimits
lim.isMinimumValueEnabled = True;  lim.minimumValue = th1_min
lim.isMaximumValueEnabled = True;  lim.maximumValue = th1_max
# get/set 也可：rev.rotationValue / motion.slideValue（当前值）
```
- `JointLimits` 成员：`minimumValue / maximumValue / restValue` + `isMinimumValueEnabled / isMaximumValueEnabled / isRestValueEnabled`
- ⚠ **单位**：Fusion API 角度类属性通常用**弧度**，实测确认后再填（`rotationValue`/limits 都按同一单位）

### 4.4 零点方向怎么定（决定标定表能不能直接照填）
- 关节的「0°」= **创建时两几何的对齐姿态**。要满足 **θ1=0 → −Y（棋盘中心）**、**θ2=0 → 大小臂伸直**，靠以下手段控制：
  - **`JointOrigin`**（推荐）：用 `JointOrigins` 建自定义原点，可设 `offsetX/offsetY/offsetZ`（`ModelParameter`）、`angle`、`isFlipped`，以及 `xAxisEntity` / `zAxisEntity` 指定轴向；其 `primaryAxisVector / secondaryAxisVector / thirdAxisVector` 给出实际轴向
  - 或用几何构造器选中方向正确的面/边/草图曲线
- **策略**：轴心/肘点都在同高度、同 Z 轴，两旋转副都取 `ZAxisJointDirection`；把 link 的草图方向画成「0° 时指向 −Y」，零点即天然成立

### 4.5 关于「接地件」与关节几何（✅ 2026-09-10 在真机 Fusion 实测确定）
- `createInput` 的**第一个几何必须是「要动的组件」**，第二个才是固定的参考件。顺序反了会报：
  `RuntimeError: 3 : First component to move is grounded, it can not move.`
- 正确链路：`createInput(link1, base)` → `createInput(link2, link1)` → `createInput(slider, link2)`
- **参考件必须是「独立且已固定的组件」**：把基座实体直接建在根组件里 → `occurrenceTwo=None`
  → Fusion 报「部件关系存在冲突」，参数写进去也不动（实测踩过）。
- **关节几何必须用面，不能用两个重合的草图点**：点-点关节的旋转轴退化为零长，
  Fusion 报「距离冲突」，关节永久转不动。
- **✅ 已验证可用的建法**（末端与 MATLAB FK 误差 **0.02mm**）：
  1. base 独立组件 + `isGrounded=True`；link1 画「轮毂圆 + 臂」，同在 z=axis_h 平面、向上拉伸
  2. 取 link1 轮毂**底面**圆面 ↔ base **顶面**圆面，两端都用
     `createByPlanarFace(face, None, JointKeyPointTypes.CenterKeyPoint)`
  3. `setAsRevoluteJointMotion(ZAxisJointDirection)` ＋ **`JointInput.isFlipped = True`**
  4. `isFlipped` 只能设在 **JointInput** 上；建好后再改 `Joint.isFlipped` 会报
     `Cannot be edited before rolling back`
- ⚠ 驱动关节后**不要**调 `design.computeAll()`，它会把驱动值重置回 0。
- 驱动/读数：`RevoluteJointMotion.rotationValue` 可读可写（弧度）；`Joint.angle` 是
  ModelParameter，`.value` 同为弧度。关节是否健康看 `Joint.healthState`（0=正常）
  与 `errorOrWarningMessage`。
- 组件位置读取：`Occurrence.transform`（关节驱动后**会**跟着变，可用于校验末端坐标）。
- 另有 `addInferredJoint(InferredJointInput)`（自动推断配对），本路线用不上

> 上述签名均来自 `FusionAPIReference/Fusion_API_Python_Reference/defs/adsk/fusion.py`（2026 May 版）；实现以该 stub 为准。

## 5. 导出 Mirabuf（不靠 API 直接写）
Mirabuf 由 **Synthesis Fusion Exporter**（另一个 Fusion 插件）生成，不是本脚本直接输出：
1. 装 `Synthesis/exporter/SynthesisFusionAddin`（Utilities→Add-Ins→+）
2. 在本脚本建好的设计里，用它导出 `.mirabuf`（GZip）
3. 导出器只接受 **Revolute / Slider / Ball** 三类 Joint——我们的 SCARA 正好两类全中
> 脚本与导出器的边界：脚本负责「把模型和 Joint 建对」，导出器负责「读 Joint 转 Mirabuf」。

## 6. MCP 在其中的角色（辅助，不主导）
- 用 `get_screenshot` 让 AI 看视口、`get_api_documentation` 查 API、`execute_api_script` 快速试一段
- 适合「调试/探索/改属性」，不适合「从零建几何」

## 7. 验证
1. 改 `PARAMS["L2"]=185` 重跑 → 模型尺寸随之更新
2. 脚本直接计算并打印若干目标格点的 FK 结果，与 `matlab/` 的 `scara_ik` 对标（同一套公式）
3. 导出 Mirabuf 成功、丢进 Synthesis 能被识别（b 阶段再验）

## 8. 里程碑
1. 跑通最小 Add-In（弹个消息框）→ 确认 API 环境
2. 脚本建出 base+link1，Joint 能转
3. 补齐 link2+slider+tool，限位/零点正确
4. 参数化跑通、FK 对标一致
5. 导出 Mirabuf 成功

## 9. 风险
- API 建 Joint 的原点/轴向/零点较绕，**这一步最花时间**，建议先做最小双连杆验证
- Fusion 版本间 API 有差异（仓库文档更新到 2026 May），以 FusionAPIReference 为准
- 导出器是 sample 级，非产品支持
