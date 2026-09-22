# scara_script.py
# 元萝卜 SCARA 数字样机 —— 参数化生成器（三种模式）
#
#   TASK = "single"  单文件模式：在一个设计里建 base/link1/link2/slider 四个组件 + 三个关节。
#                    用于快速迭代与 15 格点运动学自检（回归基线）。
#   TASK = "parts"   零件模式  ：把样机拆成 6 个【独立零件文件】，各自保存到云端项目下的 SCARA/ 文件夹。
#   TASK = "assy"    装配模式  ：新建装配体，把 6 个零件以【外部参照】插入，建 3 个关节 + 2 个刚性组。
#   TASK = "scene"   场景模式  ：在当前设计里建棋盘(330x330+网格刻线)与围棋子阵列(13 路透镜子)，
#                    给扶正附件设计 / b 阶段接触仿真提供几何环境。幂等可重复运行。
#
# 运行：Fusion → 实用工具/UTILITIES → 脚本和附加模块 → 脚本(Scripts) → 选本文件夹 → 运行
# 顺序：先跑 "parts"（产出 6 个零件文件）→ 再跑 "assy"（产出装配体）。
#
# ── 为什么必须由睿君手动跑（而不是走 MCP）──────────────────────────────
# Fusion 规定：**保存不能在事务里执行**。API 文档原文：
#   "Saving a document is not supported within any of the Command related events.
#    When a command is running, a transaction is open, and saving a document cannot
#    be transacted and, as a result, cannot be contained within a command transaction."
# MCP 执行器每次跑脚本都开一个 PTransaction → 所以 save/saveAs 必崩插件。
# 睿君在 Fusion 里手动运行本脚本不在事务里，saveAs 正常。
#
# 单位：Fusion API 长度=cm、角度=弧度；本脚本参数用 mm/deg，内部换算。
# 坐标系：棋盘中心为原点、Z 向上、基座轴心 (0, base_y)；θ1=0 指向 −Y（棋盘中心）、θ2=0 大小臂伸直。
#
# ── 关节构造经验（2026-09-10 实测确定，全部与 MATLAB FK 对拍过，别随意改）────────
#  1) 每个被参考的零件必须是【独立组件】并固定（base）；实体直接建在根组件里
#     → 关节第二端 occurrenceTwo=None → Fusion 报"部件关系存在冲突"，关节转不动。
#  2) 关节几何必须用【面】，不能用两个重合的草图点：点-点关节旋转轴退化 → 报"距离冲突"。
#  3) 配对方式统一为「活动件的轮毂底面 ↔ 参考件的圆/顶面」+ **JointInput.isFlipped = True**：
#     - th1: link1 转台轮毂底面 ↔ base 顶面
#     - th2: link2 肘部轮毂底面 ↔ link1 肘部轮毂顶面
#     - d3 : 第二段顶面        ↔ 套筒顶环面（滑动副）
#  4) isFlipped 只能设在 JointInput 上；建好后再改 Joint.isFlipped 会报
#     `Cannot be edited before rolling back`。
#  5) 驱动关节后【不要】调 design.computeAll()，它会把驱动值重置回 0。
#  6) 滑动副正方向 = +Z（向上）；下压请用负值（-35 = 压到底，吸嘴口到棋盘面 −5mm）。
#
# ── 收尾两件事（每次运行都会做，别删）────────────────────────────────────
#  A) 物理碰撞：把全部零件放进一个接触集（浏览器里叫「碰撞集」）；
#     总开关（UI 的「启用接触」）保持【关】——常开会干扰 API 驱动关节的自检并崩插件。
#     想在 UI 里手动拖拽查干涉时，自己点一下「启用接触」即可。
#  B) 隐藏辅助几何：脚本过程中用到的构造平面（浏览器里的「平面1…」）与草图，收尾统一熄灭。
#
# ── 多文件装配体的约定（TASK="parts"/"assy"）────────────────────────────
#  * 零件按【装配位置绝对坐标】建模 → 插入装配体时用单位矩阵即可原样落位，零变换风险。
#  * 文件名/组件名一律 ASCII（沿用既有约定：中文名经 GBK 通道会损坏）。
#  * 倒角与上色在【零件文件】里做（外部参照的零件在装配体里是只读的，改不动）。
#  * 套筒靠【刚性组】固定在小臂上、吸嘴靠【刚性组】随伸缩轴 —— 这样 6 个真·单件才连得起来。
# ──────────────────────────────────────────────────────────────

import math
import traceback

import adsk.core
import adsk.fusion

# ==================== 模式选择 ====================
TASK = "scene"         # "single" | "parts" | "assy" | "scene"

# ==================== 多文件装配体配置 ====================
PROJECT_NAME = "chess robot"   # 云端项目名（本样机的文件都存这里）；留空则取活动项目
FOLDER_NAME = "SCARA"    # 该项目根目录下存放本样机所有文件的文件夹
ASSY_NAME = "SCARA-assembly"
# (组件键, 文件名)；列表顺序 = 插入顺序
PART_FILES = [
    ("base", "SCARA-base"),
    ("link1", "SCARA-link1"),
    ("link2", "SCARA-link2"),
    ("sleeve", "SCARA-sleeve"),
    ("shaft", "SCARA-shaft"),
    ("nozzle", "SCARA-nozzle"),
]

PARAMS = {
    # 运动学（mm）
    "L1": 195.0, "L2": 185.0,
    "base_y": 230.0, "axis_h": 110.0,
    "z_top": 30.0,          # 吸嘴口最高位到棋盘面
    "d3_travel": 35.0,      # 下压行程
    # 外形（mm，可随意改，不影响运动学）
    "base_disc_r": 75.0, "base_disc_h": 10.0,        # 底部黑盘
    "base_sphere_r": 50.0, "base_sphere_z": 55.0,    # 球体（给球心高度）
    "base_collar_r": 50.0, "base_collar_z1": 80.0,   # 环带（从赤道到 z1）
    "base_top_r": 45.0,                              # 肩部 housing 半径（顶面 = θ1 基准面）
    "hub_r": 20.0,                           # 转台圆柱（大臂的一部分，=关节基准面 Ø40）
    "elbow_hub_r": 17.0, "wrist_hub_r": 12.0,   # 大臂肘端轴颈 Ø34
    "elbow_lobe_r": 22.0,                    # 小臂肘端轴承台 Ø44（大臂轴颈坐在它上面，露出承压环）
    "pin_r": 7.0,                            # 肘部销 Ø14（穿到臂顶露出销帽）
    "arm_w": 40.0, "arm_w2": 35.0,           # 实测：大臂宽 40、小臂宽 35
    "fillet_r": 2.0,                         # 全机倒圆角半径
    "arm_t": 35.0,                           # 实测：大臂/小臂厚度 35mm
    "plate_t": 28.0,                         # 臂板厚（比层厚小 7 并抬高 2 → 与关节面都错开，面不会被合并掉）
    # 末端（实测 2026-09-11）：三段；伸缩发生在第二段，第一段固定在臂上，吸嘴仅随第二段平移
    "ee_mount_z": 95.0,                      # 工具挂点高度（实测：气泵顶 ~95，即腕面之下 ~35）
    "ee_s1_d": 37.0, "ee_s1_h": 38.0,        # ① 第一段：套筒（从挂点往下 38），固定在臂上
    "ee_s1_bore_d": 30.6,                    #    套筒内孔：比第二段大 0.6 做间隙配合（导向，不约束）
    "ee_bracket_d": 37.0,                    #    支架：与套筒同径 → 腕部到末端是一根连续的圆柱（不再鼓出来）
    "ee_s2_d": 30.0,                         # ② 第二段：伸缩轴（长度见下方 EE_SHAFT 推导）
    "ee_tip_d": 17.0, "ee_tip_h": 25.0,      # ③ 吸嘴：随第二段动，25
    # 关节限位（deg）
    "th1_min": -150.0, "th1_max": 15.0,
    "th2_min": 0.0, "th2_max": 180.0,
}

MM = 0.1                 # mm -> cm
RAD = math.pi / 180.0    # deg -> rad
Z_LINK2 = PARAMS["ee_mount_z"]             # 小臂层底面 = 工具挂点（95），向上 arm_t（35）→ 95..130
Z_LINK1 = Z_LINK2 + PARAMS["arm_t"]        # 大臂层在小臂上方：130..165
EE_MOUNT = PARAMS["ee_mount_z"]            # 工具挂点（=关节中心）高度
WRIST_Y = PARAMS["base_y"] - PARAMS["L1"] - PARAMS["L2"]   # 腕端 y（=-150）
# 第二段轴长：由「收缩态吸嘴口 = 实测 z_top」锁定：挂点 − z_top = 轴长 + 吸嘴长。
# ⚠ 套筒缩到 38 后，「全伸只露出 25」与「收缩态 z_top=30」不可兼得（2026-09-11 与睿君确认）：
#   取 z_top → 轴长 40，代价是全伸露出 37mm（轴在孔内仍留 3mm 导向）。
EE_SHAFT = PARAMS["ee_mount_z"] - PARAMS["z_top"] - PARAMS["ee_tip_h"]
EE_EXPOSED = EE_SHAFT - PARAMS["ee_s1_h"] + PARAMS["d3_travel"]   # 全伸时露在套筒外的长度
EE_TIP_REST = EE_MOUNT - (EE_SHAFT + PARAMS["ee_tip_h"])   # 收缩态吸嘴口 → 应等于 z_top
EE_TIP_LOW = EE_TIP_REST - PARAMS["d3_travel"]             # 完全伸长时吸嘴口 → 应等于 z_top−行程
KPT = adsk.fusion.JointKeyPointTypes
TIP_LOCAL = (0.0, 0.0, -(EE_SHAFT + PARAMS["ee_tip_h"]) * MM)   # 吸嘴口（相对挂点, cm）


def _leaves(occ):
    """下潜到真正"有实体"的那一层实例，返回实例列表。
    ⚠ 外部参照进来的零件，顶层实例的 bRepBodies 是【空的】（2026-09-11 实测：count==0），
    因为零件的几何建在【零件文件根组件下的子组件】里（childOccurrences.count==1，
    子实例 bRepBodies.count==1）。所以取面/建关节前必须先下潜。
    """
    try:
        if occ.bRepBodies.count:
            return [occ]
    except Exception:
        pass
    out = []
    try:
        kids = occ.childOccurrences
    except Exception:
        return out
    for i in range(kids.count):
        out.extend(_leaves(kids.item(i)))
    return out


def _zrange(occ):
    """实例实体在装配上下文里的 z 范围（mm）；读不到返回 None。装配后自检用。"""
    try:
        lv = _leaves(occ)
        b = lv[0].bRepBodies.item(0).boundingBox
        return b.minPoint.z * 10, b.maxPoint.z * 10
    except Exception:
        return None


def _fmt_range(r):
    return "读取失败" if r is None else "%.1f~%.1f" % r


def _faces(target):
    """遍历 target 的面。Component 与实例都吃：
      - Component：直接用原生面（单文件模式构建期）。
      - 实例：下潜到有实体的那一层，用它的 bRepBodies —— 那已经是装配上下文的代理实体的面，
        可以直接喂给 JointGeometry（本地实例与外部参照实例都走这条路）。
    """
    comp = getattr(target, "component", None)
    if comp is None:                                  # Component
        for b in target.bRepBodies:
            for f in b.faces:
                yield f
        return
    for leaf in _leaves(target):
        bodies = leaf.bRepBodies
        if bodies.count:
            for b in bodies:
                for f in b.faces:
                    yield f
        else:                                         # 兜底：拿源组件原生面转代理
            for b in leaf.component.bRepBodies:
                for f in b.faces:
                    yield f.createForAssemblyContext(leaf)


def _planar_at(target, z_mm, want_area_mm2, tol=0.06):
    """在 z=z_mm 高度上找朝上/朝下的平面，面积最接近 want_area_mm2 的那个。
    ⚠ 不能只用死容差：零件是【先倒角再保存】的，圆角会把基准面切小
    （例如 Ø100 倒 R2 后只剩 Ø96，面积差 8%，超 6% 容差），所以要留"取最接近"的兜底。
    平面 + z 高度 + 法线朝 Z 三重过滤后，同一处不会再有第二个候选面。
    """
    cands = []
    for f in _faces(target):
        if "Plane" not in f.geometry.objectType:
            continue
        if abs(f.boundingBox.minPoint.z - z_mm * MM) > 1e-3:
            continue
        try:
            if abs(f.geometry.normal.z) < 0.9:
                continue
        except Exception:
            pass
        cands.append(f)
    if not cands:
        return None
    for f in cands:                                   # 先严格匹配
        if abs(f.area * 100.0 - want_area_mm2) < want_area_mm2 * tol:
            return f
    return min(cands, key=lambda f: abs(f.area * 100.0 - want_area_mm2))


def _circ_face(target, z_mm, dia_mm):
    """在 target 里找 z=z_mm 高度上、面积≈直径 dia_mm 圆面的平面"""
    return _planar_at(target, z_mm, math.pi * (dia_mm * 0.5) ** 2)


def _plane(comp, z_mm):
    pi = comp.constructionPlanes.createInput()
    pi.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(z_mm * MM))
    return comp.constructionPlanes.add(pi)


def _ext(comp, sk, dist_mm, join=False):
    fe = comp.features.extrudeFeatures
    op = (adsk.fusion.FeatureOperations.JoinFeatureOperation if join
          else adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    pi = fe.createInput(sk.profiles.item(0), op)
    pi.setDistanceExtent(False, adsk.core.ValueInput.createByReal(dist_mm * MM))
    return fe.add(pi)


def _cut(comp, sk, dist_mm):
    """用挤出做挖料（挖内孔/凹槽）"""
    fe = comp.features.extrudeFeatures
    pi = fe.createInput(sk.profiles.item(0),
                        adsk.fusion.FeatureOperations.CutFeatureOperation)
    pi.setDistanceExtent(False, adsk.core.ValueInput.createByReal(dist_mm * MM))
    return fe.add(pi)


def _annulus_face(target, z_mm, outer_d_mm, inner_d_mm):
    """找 z=z_mm 高度上的环形平面（例如套筒的顶环面）"""
    want = math.pi * ((outer_d_mm * 0.5) ** 2 - (inner_d_mm * 0.5) ** 2)
    return _planar_at(target, z_mm, want)


def _circle(comp, plane, x_mm, y_mm, r_mm):
    sk = comp.sketches.add(plane)
    sk.sketchCurves.sketchCircles.addByCenterRadius(
        adsk.core.Point3D.create(x_mm * MM, y_mm * MM, 0.0), r_mm * MM)
    return sk


def _rect(comp, plane, y0_mm, y1_mm, w_mm):
    sk = comp.sketches.add(plane)
    sk.sketchCurves.sketchLines.addTwoPointRectangle(
        adsk.core.Point3D.create(-w_mm * 0.5 * MM, y0_mm * MM, 0.0),
        adsk.core.Point3D.create(w_mm * 0.5 * MM, y1_mm * MM, 0.0))
    return sk


def _taper(comp, plane, y0_mm, w0_mm, y1_mm, w1_mm):
    """梯形臂：y0 端宽 w0、y1 端宽 w1（按实物照片，臂略有收窄）"""
    sk = comp.sketches.add(plane)
    ln = sk.sketchCurves.sketchLines

    def p(x, y):
        return adsk.core.Point3D.create(x * MM, y * MM, 0.0)

    a, b = p(-w0_mm * 0.5, y0_mm), p(w0_mm * 0.5, y0_mm)
    c, d = p(w1_mm * 0.5, y1_mm), p(-w1_mm * 0.5, y1_mm)
    for u, v in ((a, b), (b, c), (c, d), (d, a)):
        ln.addByTwoPoints(u, v)
    return sk


def _new_comp(root, name):
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name = name
    return occ


# ==================== 零件几何 ====================

def _revolve_base(comp):
    """基座外形：底盘 + 球体 + 环带 + 肩部 housing，半剖面草图旋转 360°
    草图在 YZ 平面。⚠ 该平面局部坐标为 u = −Z、v = Y（实测），
    所以用 p(Y, Z) 换算，直接按全局 (Y,Z) 下笔，避免转错平面。
    """
    P = PARAMS
    ay = P["base_y"]
    r_disc, h_disc = P["base_disc_r"], P["base_disc_h"]
    r_sph, cz = P["base_sphere_r"], P["base_sphere_z"]
    r_col, z1 = P["base_collar_r"], P["base_collar_z1"]
    r_top, z_ax = P["base_top_r"], P["axis_h"]

    dz = h_disc - cz
    r_at = math.sqrt(max(r_sph * r_sph - dz * dz, 0.0))   # 球面在底盘顶面处的半径

    sk = comp.sketches.add(comp.yZConstructionPlane)
    ln = sk.sketchCurves.sketchLines
    arcs = sk.sketchCurves.sketchArcs

    def p(Y, Z):
        return adsk.core.Point3D.create(-Z * MM, Y * MM, 0.0)

    ax0, ax1 = p(ay, 0.0), p(ay, z1)       # 轴线：到环带顶为止（上半部分已划给大臂）
    b = p(ay + r_disc, 0.0)
    c = p(ay + r_disc, h_disc)
    d = p(ay + r_at, h_disc)
    ctr = p(ay, cz)
    e = p(ay + r_sph, cz)
    f = p(ay + r_col, z1)
    g = p(ay + r_top, z1)
    h = p(ay + r_top, z_ax)

    axis = ln.addByTwoPoints(ax0, ax1)
    ln.addByTwoPoints(ax0, b)
    ln.addByTwoPoints(b, c)
    ln.addByTwoPoints(c, d)

    a0 = math.atan2(d.y - ctr.y, d.x - ctr.x)
    a1 = math.atan2(e.y - ctr.y, e.x - ctr.x)
    sweep = a1 - a0
    if sweep <= 0.0:
        sweep += 2.0 * math.pi
    arcs.addByCenterStartSweep(ctr, d, sweep)

    ln.addByTwoPoints(e, f)
    ln.addByTwoPoints(f, ax1)

    rev_in = comp.features.revolveFeatures.createInput(
        sk.profiles.item(0), axis,
        adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    rev_in.setAngleExtent(False, adsk.core.ValueInput.createByReal(2.0 * math.pi))
    comp.features.revolveFeatures.add(rev_in)


def _build_base(comp):
    _revolve_base(comp)


def _build_link1(c1):
    """大臂 = 基座上半部分（肩部 housing）+ 转台轮毂 + 臂板 + 肘端 + 销，合成一个整体。"""
    P = PARAMS
    p1 = _plane(c1, Z_LINK1)
    # 基座上半部分（肩部 housing）属于大臂 → 先建它，再与下面各件合并成一个整体
    _ext(c1, _circle(c1, _plane(c1, P["base_collar_z1"]), 0.0, P["base_y"],
                     P["base_collar_r"]), Z_LINK1 - P["base_collar_z1"])
    _ext(c1, _circle(c1, p1, 0.0, P["base_y"], P["hub_r"]), P["arm_t"], join=True)
    _ext(c1, _taper(c1, _plane(c1, Z_LINK1 + 2.0), P["base_y"], P["arm_w"],
                    P["base_y"] - P["L1"], P["arm_w"] * 0.85),
         P["plate_t"], join=True)   # 抬高 2：避开肘部关节面（否则被合并掉）
    _ext(c1, _circle(c1, p1, 0.0, P["base_y"] - P["L1"], P["elbow_hub_r"]),
         P["arm_t"], join=True)
    # 肘部销：从臂板里穿上去，在臂顶露出销帽（不碰关节面，避免把基准面掏成环）
    _ext(c1, _circle(c1, _plane(c1, Z_LINK1 + 2.0), 0.0,
                     P["base_y"] - P["L1"], P["pin_r"]),
         P["arm_t"] - 3.0, join=True)


def _build_link2(c2):
    """小臂（在【大臂下方】，不含套筒）= 肘端轴承台 + 臂板 + 腕端圆头，合成一个整体。"""
    P = PARAMS
    # 肘端轴承台（Ø44）：从层底做到关节面；大臂的轴颈坐在它上面 → 止口配合
    _ext(c2, _circle(c2, _plane(c2, Z_LINK2), 0.0, P["base_y"] - P["L1"],
                     P["elbow_lobe_r"]), Z_LINK1 - Z_LINK2)
    _ext(c2, _taper(c2, _plane(c2, Z_LINK2), P["base_y"] - P["L1"],
                    P["arm_w2"], WRIST_Y, P["arm_w2"] * 0.85), P["plate_t"], join=True)
    # 腕端圆头：与套筒同径（Ø37），底面正好落在工具挂点 → 臂与末端齐平相接
    _ext(c2, _circle(c2, _plane(c2, Z_LINK2), 0.0, WRIST_Y,
                     P["ee_bracket_d"] * 0.5), P["plate_t"], join=True)


def _build_sleeve(cw):
    """套筒（第一段）：固定在挂点下方，先实心再挖内孔当导向。"""
    P = PARAMS
    _ext(cw, _circle(cw, _plane(cw, EE_MOUNT), 0.0, WRIST_Y, P["ee_s1_d"] * 0.5),
         -P["ee_s1_h"])
    _cut(cw, _circle(cw, _plane(cw, EE_MOUNT), 0.0, WRIST_Y,
                     P["ee_s1_bore_d"] * 0.5), -P["ee_s1_h"])


def _build_shaft(cs):
    """伸缩轴（第二段）：建在挂点下方，靠 joint_d3 顶到套筒顶环面后就位。"""
    _ext(cs, _circle(cs, _plane(cs, -EE_SHAFT), 0.0, 0.0,
                     PARAMS["ee_s2_d"] * 0.5), EE_SHAFT)


def _build_nozzle(cn):
    """吸嘴（第三段）：靠刚性组跟随伸缩轴。"""
    P = PARAMS
    _ext(cn, _circle(cn, _plane(cn, -(EE_SHAFT + P["ee_tip_h"])), 0.0, 0.0,
                     P["ee_tip_d"] * 0.5), P["ee_tip_h"])


BUILDERS = {
    "base": _build_base,
    "link1": _build_link1,
    "link2": _build_link2,
    "sleeve": _build_sleeve,
    "shaft": _build_shaft,
    "nozzle": _build_nozzle,
}

# 配色：基座 + 末端执行器 = 黑；大臂 + 小臂 = 浅灰
PAINT = {"base": "black", "link1": "gray", "link2": "gray",
         "sleeve": "black", "shaft": "black", "nozzle": "black"}

# 倒角半径（按零件给）。⚠ 套筒必须用小半径：外 Ø37 / 内孔 Ø30.6 → 壁厚只有 3.2mm，
# R2 倒角两边各吃 2mm（2+2=4 > 3.2）会把顶/底环面【整圈啃掉】，joint_d3 的基准面就没了
# （2026-09-11 实测：套筒倒完角后一个平面都不剩）。R0.8 留 1.6mm 平面。
FILLET_R = {"base": 2.0, "link1": 2.0, "link2": 2.0,
            "sleeve": 0.8, "shaft": 2.0, "nozzle": 2.0}


def _find_appearance(app, kind):
    """在库里找外观。⚠ 中文关键字必须用码位构造：脚本经 GBK 临时文件传给 Fusion，
    直接写中文字面量会变乱码。库里的命名形如 '塑料 - 有光泽(灰色)'。
    """
    BLACK = chr(0x9ED1)                    # 黑
    WHITE = chr(0x767D)                    # 白
    GRAY = chr(0x7070)                     # 灰
    CLEAR = chr(0x900F) + chr(0x660E)      # 透明（排除半透明/透明，要实色）
    LIB_APPEARANCE = chr(0x5916) + chr(0x89C2)   # 外观
    libs = app.materialLibraries
    order = [li for li in range(libs.count)
             if LIB_APPEARANCE in libs.item(li).name] + \
            [li for li in range(libs.count)
             if LIB_APPEARANCE not in libs.item(li).name]
    for li in order:
        try:
            aps = libs.item(li).appearances
        except Exception:
            continue
        for ai in range(aps.count):
            name = aps.item(ai).name
            if CLEAR in name:
                continue
            if kind == "black" and BLACK in name and GRAY not in name:
                return aps.item(ai)
            if kind == "white" and WHITE in name and BLACK not in name:
                return aps.item(ai)
            if kind == "gray" and GRAY in name and BLACK not in name:
                return aps.item(ai)
    return None


def _fillet_all(root, radius_mm=2.0, log=None):
    """给各组件所有边做圆角（注塑件观感）。个别实体失败不影响整体，退一档半径再试一次。"""
    if log is None:
        log = []
    for occ in root.occurrences:
        comp = occ.component
        for body in comp.bRepBodies:
            edges = adsk.core.ObjectCollection.create()
            for e in body.edges:
                edges.add(e)
            if edges.count == 0:
                continue
            done = False
            for r in (radius_mm, radius_mm * 0.5):
                try:
                    feats = comp.features.filletFeatures
                    inp = feats.createInput()
                    inp.addConstantRadiusEdgeSet(
                        edges, adsk.core.ValueInput.createByReal(r * MM), True)
                    feats.add(inp)
                    log.append("%s:R%.1f" % (body.name, r))
                    done = True
                    break
                except Exception:
                    continue
            if not done:
                log.append("%s:跳过" % body.name)
    return log


def _paint(root):
    """给各组件上色（幂等，重建后会自动重来一遍）"""
    app = adsk.core.Application.get()
    colors = {k: _find_appearance(app, v) for k, v in PAINT.items()}
    for occ in root.occurrences:
        base = occ.name.split(":")[0]
        ap = colors.get(base)
        if ap:
            occ.appearance = ap
    return "%s / %s" % (
        colors["base"].name if colors["base"] else "未找到黑色",
        colors["link1"].name if colors["link1"] else "未找到浅灰")


def _collision(design, root):
    """给所有实体加物理碰撞：把全部零件放进一个接触集（总开关建完关回默认）。
    ⚠ 必须先打开总开关（UI 的「启用接触」）才能建接触集，否则报
      "Contact analysis is disabled."（实测）；建完再关回去。
    ⚠ 总开关常开时，接触求解器会介入 API 驱动关节（实测 15 格点自检由 0.26mm 变 162mm），
      还会把插件带崩 → 一律保持关闭，需要手动拖拽验证干涉时在 UI 里点「启用接触」。
    """
    design.isContactAnalysisEnabled = True
    cs = design.contactSets
    for i in range(cs.count - 1, -1, -1):
        try:
            cs.item(i).deleteMe()
        except Exception:
            pass
    occs = [o for o in root.occurrences]
    s = cs.add(occs) if occs else None
    if s:
        # 中文名用码位构造：脚本可能经 GBK 临时文件中转
        s.name = chr(0x78B0) + chr(0x649E) + chr(0x96C6)  # 碰撞集
    design.isContactSetAnalysis = True
    design.isContactAnalysisEnabled = False
    return s


def _hide_helpers(comps):
    """收尾：熄灭构造平面 / 草图（含各组件里的「平面1…」），只留实体可见。"""
    n_pl = n_sk = 0
    for comp in comps:
        for pl in comp.constructionPlanes:
            try:
                if pl.isLightBulbOn:
                    pl.isLightBulbOn = False
                    n_pl += 1
            except Exception:
                pass
        for sk in comp.sketches:
            try:
                if sk.isVisible:
                    sk.isVisible = False
                    n_sk += 1
            except Exception:
                pass
        for folder in ("isConstructionFolderLightBulbOn",
                       "isSketchFolderLightBulbOn",
                       "isOriginFolderLightBulbOn"):
            try:
                if getattr(comp, folder):
                    setattr(comp, folder, False)
            except Exception:
                pass
    return n_pl, n_sk


def _clean(root, prefixes=("base", "link1", "link2", "slider"),
           joint_names=("joint_th1", "joint_th2", "joint_d3")):
    """幂等清理：删掉上一次脚本生成的关节与组件，使脚本能在同一文档里重复运行（原地替换）。
    先删关节再删组件，否则会留下悬空引用。
    """
    removed = []
    for j in list(root.joints):
        if j.name in joint_names:
            removed.append("关节:" + j.name)
            j.deleteMe()
    for occ in list(root.occurrences):
        if occ.name.split(":")[0] in prefixes:
            removed.append("组件:" + occ.name)
            occ.deleteMe()
    return removed


# ==================== 关节 / 刚性组 ====================

def _revolute(root, f_move, f_ref, name, lo_deg, hi_deg, isflip=True):
    g1 = adsk.fusion.JointGeometry.createByPlanarFace(f_move, None, KPT.CenterKeyPoint)
    g2 = adsk.fusion.JointGeometry.createByPlanarFace(f_ref, None, KPT.CenterKeyPoint)
    ji = root.joints.createInput(g1, g2)
    ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    ji.isFlipped = isflip
    j = root.joints.add(ji)
    j.name = name
    lim = adsk.fusion.RevoluteJointMotion.cast(j.jointMotion).rotationLimits
    lim.isMinimumValueEnabled = True
    lim.minimumValue = lo_deg * RAD
    lim.isMaximumValueEnabled = True
    lim.maximumValue = hi_deg * RAD
    return j


def _make_joints(root, occ):
    """建三个关节。occ 是 {键: Occurrence}；面一律取自 occurrence.bRepBodies（装配上下文），
    这样单文件模式与外部参照装配模式走的是同一条代码路径。
    """
    P = PARAMS
    _revolute(root,
              _circ_face(occ["link1"], P["base_collar_z1"], P["base_collar_r"] * 2),
              _circ_face(occ["base"], P["base_collar_z1"], P["base_collar_r"] * 2),
              "joint_th1", P["th1_min"], P["th1_max"])
    _revolute(root,
              _circ_face(occ["link2"], Z_LINK1, P["elbow_lobe_r"] * 2),
              _circ_face(occ["link1"], Z_LINK1, P["elbow_hub_r"] * 2),
              "joint_th2", -P["th2_max"], -P["th2_min"])

    g1s = adsk.fusion.JointGeometry.createByPlanarFace(
        _circ_face(occ["shaft"], 0.0, P["ee_s2_d"]), None, KPT.CenterKeyPoint)
    g2s = adsk.fusion.JointGeometry.createByPlanarFace(
        _annulus_face(occ["sleeve"], EE_MOUNT, P["ee_s1_d"], P["ee_s1_bore_d"]),
        None, KPT.CenterKeyPoint)
    jis = root.joints.createInput(g1s, g2s)
    jis.setAsSliderJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    # 轴顶面(朝上) ↔ 套筒顶环面(朝上)：两面法线同向，isFlipped 必须为 False，否则整体翻转装反
    jis.isFlipped = False
    j3 = root.joints.add(jis)
    j3.name = "joint_d3"
    slim = adsk.fusion.SliderJointMotion.cast(j3.jointMotion).slideLimits
    slim.isMinimumValueEnabled = True
    slim.minimumValue = -P["d3_travel"] * MM     # 下压为负
    slim.isMaximumValueEnabled = True
    slim.maximumValue = 0.0


def _make_rigid(root, occ, log=None):
    """用【刚性组】把套筒固定在小臂上、吸嘴固定在伸缩轴上（6 个真·单件才连得起来）。
    刚性组没有 isFlipped 那种正反面坑，比刚性关节稳。
    """
    if log is None:
        log = []
    for names, gname in ((("sleeve", "link2"), "rigid_sleeve_link2"),
                         (("nozzle", "shaft"), "rigid_nozzle_shaft")):
        try:
            coll = adsk.core.ObjectCollection.create()
            for n in names:
                coll.add(occ[n])
            g = root.rigidGroups.add(coll, False)
            g.name = gname
            log.append(gname)
        except Exception as e:
            log.append("%s 失败: %s" % (gname, e))
    return log


# ==================== 三种模式 ====================

def _finish(root, radius_mm=None):
    """零件级收尾：倒角 + 上色 + 熄灯。（外部参照的零件在装配体里是只读的，所以这三件事
    必须在零件文件里做，装配模式不再调用它。）
    """
    _fillet_all(root, PARAMS["fillet_r"] if radius_mm is None else radius_mm)
    paint_info = _paint(root)
    n_pl, n_sk = _hide_helpers([root] + [o.component for o in root.occurrences])
    return paint_info, n_pl, n_sk


def _run_single(app, ui, design, root):
    removed = _clean(root)          # 幂等：重复运行时先删掉旧件，原地重建

    # ---------- base：独立组件并固定（底盘+球体+环带+肩部，一个旋转体） ----------
    b_occ = _new_comp(root, "base")
    b_occ.isGrounded = True
    _build_base(b_occ.component)

    # ---------- link1（大臂）----------
    c1 = _new_comp(root, "link1").component
    _build_link1(c1)

    # ---------- link2（小臂）+ 套筒 ----------
    c2 = _new_comp(root, "link2").component
    _build_link2(c2)
    _build_sleeve(c2)

    # ---------- slider：② 第二段（伸缩节） + ③ 吸嘴（仅随第二段平移） ----------
    sc = _new_comp(root, "slider").component
    _build_shaft(sc)
    _build_nozzle(sc)

    # 单文件模式下套筒并进 link2 组件、伸缩轴与吸嘴并进 slider 组件；
    # 这里映射成与装配模式一致的键，好让 _make_joints 走同一条代码路径。
    o = {x.name.split(":")[0]: x for x in root.occurrences}
    occ = {"base": o["base"], "link1": o["link1"], "link2": o["link2"],
           "sleeve": o["link2"], "shaft": o["slider"], "nozzle": o["slider"]}
    _make_joints(root, occ)
    cset = _collision(design, root)
    paint_info, n_pl, n_sk = _finish(root)

    msg = "\n".join([
        "== SCARA 骨架已生成（single）==",
        "清理   : %s" % ("、".join(removed) if removed else "（首次运行，无需清理）"),
        "base   : 组件(固定) 底盘φ%.0f/球φ%.0f/肩φ%.0f  顶面 z=%.0f  @ (0,%.0f)" % (
            PARAMS["base_disc_r"] * 2, PARAMS["base_sphere_r"] * 2,
            PARAMS["base_top_r"] * 2, PARAMS["axis_h"], PARAMS["base_y"]),
        "link1  : 大臂(整体) %.0f 宽%.0f  层 z=%.0f" % (
            PARAMS["L1"], PARAMS["arm_w"], Z_LINK1),
        "link2  : 小臂(整体) %.0f 宽%.0f  层 z=%.0f（在大臂下方）" % (
            PARAMS["L2"], PARAMS["arm_w2"], Z_LINK2),
        "slider : (1)φ%.0f×%.0f套筒(固定) (2)φ%.0f轴×%.0f(全伸露出%.0f) (3)φ%.0f×%.0f吸嘴" % (
            PARAMS["ee_s1_d"], PARAMS["ee_s1_h"], PARAMS["ee_s2_d"], EE_SHAFT,
            EE_EXPOSED, PARAMS["ee_tip_d"], PARAMS["ee_tip_h"]),
        "吸嘴口 : 挂点 z=%.0f → 收缩 z=%.1f（实测 z_top=%.0f %s）/ 全伸 z=%.1f" % (
            EE_MOUNT, EE_TIP_REST, PARAMS["z_top"],
            "一致" if abs(EE_TIP_REST - PARAMS["z_top"]) < 0.5 else "**不一致**",
            EE_TIP_LOW),
        "关节   : %s" % ", ".join("%s(健康=%s)" % (x.name, x.healthState)
                               for x in root.joints),
        "限位   : θ1[%.0f,%.0f]  θ2[%.0f,%.0f]（负值=折叠）  d3[%.0f,0]mm" % (
            PARAMS["th1_min"], PARAMS["th1_max"], -PARAMS["th2_max"],
            -PARAMS["th2_min"], -PARAMS["d3_travel"]),
        "配色   : 黑=%s   浅灰=%s" % tuple(paint_info.split(" / ")),
        "碰撞   : 接触集=%s（总开关关，手动拖拽时在 UI 点「启用接触」）" % (
            cset.name if cset else "无"),
        "收尾   : 熄灭构造平面 %d 个、草图 %d 个" % (n_pl, n_sk),
    ])
    print(msg)
    ui.messageBox(msg)


# ==================== 场景模式（棋盘 + 棋子阵列） ====================
# 用途: 给扶正附件设计(主战场: 棋盒/散乱区)与 b 阶段接触仿真提供真实几何环境。
# 坐标系沿用 params.md: 棋盘中心 = 原点, 棋盘面 z=0, 板体向 -Z。
# 棋盒(214x62x26 x2)位置与朝向待实测(P0), 本版暂不建。
# 全部尺寸可在 SCENE_* 常量里调, 数值来源见 docs/params.md 与 docs/扶正附件设计方案.md。

SCENE_BOARD_W = 330.0    # 棋盘边长
SCENE_BOARD_T = 12.0     # 棋盘厚度(占位, P0 待实测)
SCENE_GRID_W = 255.0     # 网格区边长
SCENE_GRID_N = 12        # 网格区格数(格距 255/12=21.25 假设, P0 待实测)
SCENE_GO_N = 13          # 围棋路数(13 路 = 12 格 = 169 交点); 0 = 不摆子
SCENE_LINE_W = 1.2       # 网格印刷线宽
SCENE_LINE_H = 0.3       # 网格印刷线总高(对称拉伸: 一半嵌板一半外露, 黑色贴在浅色板上)

# 围棋子(扁椭球/双凸透镜): D19 x 高4.5
GO_D, GO_H = 19.0, 4.5

# 围棋子摆位模式:
#   "full"   = 13x13 满盘阵列(密排压力测试/扶正净空演示)
#   "record" = 按 GO_STONES 棋谱摆子(真实工作环境)
SCENE_GO_MODE = "record"
# 13 路真实对局终局面(71 手, 黑 35 白 36, 1 次提子后)。
# 来源: OGS game 9676316, "Ladder Challenge: Nichtogo(#249) vs anoek(#198)",
# 2017-08-13, 8k vs 8k, 白中盘胜; SGF 经吃子模拟得到终局面。
# 交点坐标 (col,row) 0..12, (0,0) 在棋盘左下角; 颜色 = 手序奇偶(第 0 手黑)。
GO_STONES = [
    (9, 9), (2, 3), (3, 9), (9, 3), (6, 6), (10, 6), (2, 6), (10, 10),
    (10, 9), (9, 10), (8, 10), (8, 11), (7, 10), (7, 11), (11, 10), (11, 11),
    (11, 9), (5, 2), (10, 2), (10, 3), (9, 2), (8, 2), (11, 3), (11, 4),
    (11, 2), (2, 10), (3, 10), (5, 11), (5, 10), (2, 9), (1, 11), (3, 12),
    (4, 11), (6, 11), (6, 12), (6, 10), (3, 11), (4, 10), (2, 12), (11, 5),
    (10, 5), (12, 4), (10, 4), (11, 6), (11, 7), (12, 5), (8, 7), (7, 8),
    (2, 8), (8, 6), (9, 6), (8, 5), (8, 4), (7, 4), (7, 3), (6, 4),
    (4, 4), (4, 5), (3, 4), (1, 4), (1, 3), (1, 7), (3, 7), (4, 7),
    (2, 5), (3, 6), (1, 5), (2, 7), (3, 8), (4, 8), (8, 1),
]


def _fillet_body_edges(comp, body, radius_mm):
    """单个实体的全部边圆角, 失败退半径重试, 返回实际半径或 None。
    与 _fillet_all 的区别: 只处理指定体(棋盘板), 不波及棋子(透镜厚 4.5 吃不起 R2)。"""
    for r in (radius_mm, radius_mm * 0.5):
        try:
            edges = adsk.core.ObjectCollection.create()
            for e in body.edges:
                edges.add(e)
            feats = comp.features.filletFeatures
            inp = feats.createInput()
            inp.addConstantRadiusEdgeSet(
                edges, adsk.core.ValueInput.createByReal(r * MM), True)
            feats.add(inp)
            return r
        except Exception:
            continue
    return None


def _build_scene_board(comp):
    """棋盘板 330x330x板厚(顶面 z=0, 浅色) + 网格黑色印刷线(浮条)。
    返回(格距, 板体, 线体列表)。
    线改贴条不再切槽(2026-09-15 睿君反馈"看不到网格"):
    1) 黑板上的黑槽视觉上隐形, 真机是浅色面板+黑色印刷线;
    2) 切槽方向依赖草图面法线, 歧义; 贴条用 setSymmetricExtent 对称拉伸,
       一半嵌进板内一半外露, 无方向问题。NewBody 不与板合体才能单独上黑。"""
    sk = _rect(comp, _plane(comp, -SCENE_BOARD_T),
               -SCENE_BOARD_W * 0.5, SCENE_BOARD_W * 0.5, SCENE_BOARD_W)
    _ext(comp, sk, SCENE_BOARD_T)
    board_body = comp.bRepBodies.item(0)

    n_lines = SCENE_GRID_N + 1
    pitch = SCENE_GRID_W / SCENE_GRID_N
    half = SCENE_GRID_W * 0.5
    hw = SCENE_LINE_W * 0.5
    gapv = 0.05   # 竖段与横线环之间留缝: 相交/贴合环会被 Fusion 碎成 581 个
                  # profile 并熔成实心方片盖住白板(2026-09-15 实测体积 19691)
    sk2 = comp.sketches.add(_plane(comp, 0.0))
    ln = sk2.sketchCurves.sketchLines
    he = half + hw
    for i in range(n_lines):
        c = -half + i * pitch
        # 横线通长(端头包住边缘竖线)
        ln.addTwoPointRectangle(
            adsk.core.Point3D.create(-he * MM, (c - hw) * MM, 0.0),
            adsk.core.Point3D.create(he * MM, (c + hw) * MM, 0.0))
        # 竖线分段夹在横线之间, 环互不相交 -> 各自独立成体
        if i < n_lines - 1:
            y0, y1 = c + hw + gapv, c + pitch - hw - gapv
            for j in range(n_lines):
                d = -half + j * pitch
                ln.addTwoPointRectangle(
                    adsk.core.Point3D.create((d - hw) * MM, y0 * MM, 0.0),
                    adsk.core.Point3D.create((d + hw) * MM, y1 * MM, 0.0))
    profs = adsk.core.ObjectCollection.create()
    for i in range(sk2.profiles.count):
        profs.add(sk2.profiles.item(i))
    fe = comp.features.extrudeFeatures
    pi = fe.createInput(profs, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    pi.setSymmetricExtent(
        adsk.core.ValueInput.createByReal(SCENE_LINE_H * MM), True)
    line_feat = fe.add(pi)
    # 板外缘全圆角 R2(12 条边; 失败退 R1)。圆角特征重算后体引用失效 ->
    # 按体积重取: 板最大(约 1.3e6 mm3), 其余全是网格线细条。
    _fillet_body_edges(comp, board_body, 2.0)
    all_bodies = sorted(comp.bRepBodies, key=lambda b: b.volume, reverse=True)
    board_body = all_bodies[0]
    line_bodies = list(all_bodies[1:])
    return pitch, board_body, line_bodies


def _scene_lens(comp):
    """一颗围棋子 = 双凸透镜(俯视 D19 x 厚 4.5), 建在原点, 关于 z=0 上下对称
    (z 属于 [-2.25, 2.25]); 抬升落位由调用者的 move 负责。
    过轴半剖面(上下对称两段弧+竖轴线)画在 XZ 草图上, 绕竖直构造轴转 360 度。
    三条实测教训(2026-09-15, 别再踩):
    1) sketch 点输入按【草图局部坐标】解释, 第三分量是法向偏移必须给 0;
       当成全局坐标喂, 剖面/轴会画歪(XZ 草图上轴跑到全局 Y, 透镜立起来)。
    2) 旋转轴不可与剖面边共线重合, 否则 extent 只转 90 度 -> 用独立构造线。
    3) 透镜是绕【厚度对称轴】的回转体; "弓形绕弦转"得到的是径向 4.5 的腰鼓棒。"""
    a, H = GO_D * 0.5, GO_H
    hm = H * 0.5
    R = (a * a + hm * hm) / (2.0 * hm)
    sk = comp.sketches.add(comp.xZConstructionPlane)

    def p(u, v):
        return adsk.core.Point3D.create(u * MM, v * MM, 0.0)

    ln = sk.sketchCurves.sketchLines
    ln.addByTwoPoints(p(0, -hm), p(0, hm))
    zc1 = hm - R
    ph_a = math.pi * 0.5
    ph_b = math.atan2(hm - zc1, a)
    ph_m = (ph_a + ph_b) * 0.5
    mx = R * math.cos(ph_m)
    mz1 = zc1 + R * math.sin(ph_m)
    arcs = sk.sketchCurves.sketchArcs
    arcs.addByThreePoints(p(0, hm), p(mx, mz1), p(a, 0.0))
    arcs.addByThreePoints(p(a, 0.0), p(mx, -mz1), p(0, -hm))
    axis = ln.addByTwoPoints(p(0, -3.0), p(0, 3.0))
    axis.isConstruction = True

    rf = comp.features.revolveFeatures
    ri = rf.createInput(sk.profiles.item(0), axis,
                        adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    ri.setAngleExtent(False, adsk.core.ValueInput.createByReal(2.0 * math.pi))
    return rf.add(ri).bodies.item(0)


def _build_scene_go(comp):
    """围棋子摆位, 返回(子数, 黑外观, 白外观)。
    full = 13x13 满盘阵列(密排压力测试); record = 按 GO_STONES 棋谱摆子。
    性能关键: 逐颗建草图+旋转特征曾拖死 API(2026-09-15 实测 20 分钟不收敛)。
    full 用 1 颗种子 + 两次矩形阵列; record 用 TemporaryBRep 内核复制
    (copy -> transform -> add, 无特征、秒级), 颜色按手序奇偶。"""
    app = adsk.core.Application.get()
    ap_black = _find_appearance(app, "black")
    ap_white = _find_appearance(app, "white")
    pitch = SCENE_GRID_W / SCENE_GRID_N
    half_span = (SCENE_GO_N - 1) * 0.5 * pitch

    if SCENE_GO_MODE == "record":
        assert len(set(GO_STONES)) == len(GO_STONES), "GO_STONES 有重叠落点"
        # 棋子 = 黑/白各 1 个定义组件 + 41 个实例。上色历程(都实测过):
        # tempBRep+BaseFeature 体上色被特征重算清掉; 深层 occurrence.appearance
        # 赋值静默失效(proxy 限制)。颜色赋在【普通特征体】上稳定(full 模式已验证),
        # 黑白各一个定义、颜色随定义走, 实例不用单独上色。
        # 落位: transform2 只能写 root 直接子代 -> 定义几何建在【各自第一颗】
        # 的落点, 后续实例插入矩阵 = 相对该首颗的偏移。
        blacks = [pt for i, pt in enumerate(GO_STONES) if i % 2 == 0]
        whites = [pt for i, pt in enumerate(GO_STONES) if i % 2 == 1]
        half_n = (SCENE_GO_N - 1) * 0.5
        hm = GO_H * 0.5
        n = 0
        for name, group, ap in (("go_piece_black", blacks, ap_black),
                                ("go_piece_white", whites, ap_white)):
            if not group:
                continue
            def_occ = comp.occurrences.addNewComponent(
                adsk.core.Matrix3D.create())
            def_occ.component.name = name
            cdef = def_occ.component
            body = _scene_lens(cdef)
            c0, r0 = group[0]
            oc = adsk.core.ObjectCollection.create()
            oc.add(body)
            mi = cdef.features.moveFeatures.createInput2(oc)
            mi.defineAsTranslateXYZ(
                adsk.core.ValueInput.createByReal((c0 - half_n) * pitch * MM),
                adsk.core.ValueInput.createByReal((r0 - half_n) * pitch * MM),
                adsk.core.ValueInput.createByReal(hm * MM), True)
            cdef.features.moveFeatures.add(mi)
            if ap:
                body.appearance = ap   # 定义体上色 -> 全部实例跟随
            n += 1
            mat = adsk.core.Matrix3D.create()
            for col, row in group[1:]:
                mat.translation = adsk.core.Vector3D.create(
                    (col - c0) * pitch * MM,
                    (row - r0) * pitch * MM,
                    0.0)
                comp.occurrences.addExistingComponent(def_occ.component, mat)
                n += 1
        return n, ap_black, ap_white

    seed = _scene_lens(comp)

    # 种子平移到网格左下角交点(-half_span,-half_span)并抬到底面贴盘 z=0:
    # 阵列沿轴【正方向】展开(实测 x=0..+255), 种子不放左下角, 整片就偏到棋盘右上象限
    oc = adsk.core.ObjectCollection.create()
    oc.add(seed)
    mf = comp.features.moveFeatures
    mi = mf.createInput2(oc)
    mi.defineAsTranslateXYZ(
        adsk.core.ValueInput.createByReal(-half_span * MM),
        adsk.core.ValueInput.createByReal(-half_span * MM),
        adsk.core.ValueInput.createByReal(GO_H * 0.5 * MM), True)
    mf.add(mi)

    pf = comp.features.rectangularPatternFeatures
    # 本版 API 的 createInput 是单方向签名(实测 createInput2 不存在),
    # 但 FeatureInput 的第二方向默认数量是 3 -> 必须显式 quantityTwo=1,
    # 否则 13x13=169 颗会炸成 1521 颗(2026-09-15 实测 39=13x3/层)。
    # 先沿 X 阵出一行 13 颗, 再把整行沿 Y 阵出 13x13。
    oc = adsk.core.ObjectCollection.create()
    oc.add(seed)
    ri1 = pf.createInput(
        oc, comp.xConstructionAxis,
        adsk.core.ValueInput.createByReal(SCENE_GO_N),
        adsk.core.ValueInput.createByReal(pitch * MM),
        adsk.fusion.PatternDistanceType.SpacingPatternDistanceType)
    ri1.quantityTwo = adsk.core.ValueInput.createByReal(1)
    pf.add(ri1)

    oc2 = adsk.core.ObjectCollection.create()
    for b in comp.bRepBodies:
        oc2.add(b)
    ri2 = pf.createInput(
        oc2, comp.yConstructionAxis,
        adsk.core.ValueInput.createByReal(SCENE_GO_N),
        adsk.core.ValueInput.createByReal(pitch * MM),
        adsk.fusion.PatternDistanceType.SpacingPatternDistanceType)
    ri2.quantityTwo = adsk.core.ValueInput.createByReal(1)
    pf.add(ri2)

    n = 0
    for b in comp.bRepBodies:
        bb = b.boundingBox
        cx = ((bb.minPoint.x + bb.maxPoint.x) * 0.5) / MM
        cy = ((bb.minPoint.y + bb.maxPoint.y) * 0.5) / MM
        i = int(round((cx + half_span) / pitch))
        j = int(round((cy + half_span) / pitch))
        ap = ap_black if (i + j) % 2 == 0 else ap_white
        if ap:
            b.appearance = ap
        n += 1
    return n, ap_black, ap_white


def _run_scene(app, ui, quiet=False):
    design = adsk.fusion.Design.cast(app.activeProduct)
    if not design:
        ui.messageBox("scene 模式: 请先新建一个设计再运行本脚本。")
        return
    root = design.rootComponent
    _set_mm(design)
    removed = _clean(root, prefixes=("scene_board", "scene_go"))

    b_occ = _new_comp(root, "scene_board")
    pitch, board_body, line_bodies = _build_scene_board(b_occ.component)
    ap_board = _find_appearance(app, "white") or _find_appearance(app, "gray")
    if board_body and ap_board:
        board_body.appearance = ap_board
    ap_line = _find_appearance(app, "black")
    if ap_line:
        for lb in line_bodies:
            lb.appearance = ap_line

    n_pieces = 0
    ap_white = None
    if SCENE_GO_N > 0:
        g_occ = _new_comp(root, "scene_go")
        n_pieces, _, ap_white = _build_scene_go(g_occ.component)

    n_pl, n_sk = _hide_helpers([o.component for o in root.occurrences])

    # 外观库诊断(上色失败时用这个定位: 库没加载/根本没黑白色)
    BLACK = chr(0x9ED1)
    WHITE = chr(0x767D)
    n_lib = 0
    n_black = n_white = 0
    for li in range(app.materialLibraries.count):
        try:
            aps = app.materialLibraries.item(li).appearances
        except Exception:
            continue
        n_lib += 1
        for ai in range(aps.count):
            nm = aps.item(ai).name
            if BLACK in nm:
                n_black += 1
            if WHITE in nm:
                n_white += 1

    gap = pitch - GO_D
    msg = "\n".join([
        "== 场景已生成 (scene) ==",
        "清理: %s" % ("、".join(removed) if removed else "(首次运行, 无需清理)"),
        "棋盘: %.0f x %.0f x %.1f (顶面 z=0, 厚度占位待实测)" % (
            SCENE_BOARD_W, SCENE_BOARD_W, SCENE_BOARD_T),
        "网格: %.0f 方区 / %d 格, 格距 %.3f, 印刷线 %.1f 宽 x %.1f 高 x %d 条" % (
            SCENE_GRID_W, SCENE_GRID_N, pitch, SCENE_LINE_W, SCENE_LINE_H,
            len(line_bodies)),
        "围棋子: D%.1f x %.1f 透镜 x %d 个 (%s 模式%s)" % (
            GO_D, GO_H, n_pieces, SCENE_GO_MODE,
            (", 交点覆盖 +-%.1f" % ((SCENE_GO_N - 1) * 0.5 * pitch))
            if SCENE_GO_MODE == "full" else (", 棋谱 %d 手" % len(GO_STONES))),
        "密排净空: 格距 - 子径 = %.2f mm" % gap,
        "棋盒: 未建(位置朝向待 P0 实测)",
        "外观库: 可用 %d 库, 名含黑 %d 个 / 含白 %d 个" % (n_lib, n_black, n_white),
        "棋盘外观: %s (网格线: %s)" % (
            ap_board.name if ap_board else "未找到, 用默认色",
            ap_line.name if ap_line else "未找到, 用默认色"),
        "白子外观: %s" % (ap_white.name if ap_white else "未找到, 用默认色"),
        "收尾: 熄灭构造平面 %d 个、草图 %d 个" % (n_pl, n_sk),
    ])
    print(msg)
    if not quiet:
        ui.messageBox(msg)


def _set_mm(design):
    """新建文档的显示单位跟随 Fusion 偏好设置，不一定是 mm → 强制成 mm，
    免得零件/装配文件里尺寸都显示成英寸。"""
    design.fusionUnitsManager.distanceDisplayUnits = \
        adsk.fusion.DistanceUnits.MillimeterDistanceUnits


def _active_project(app, log):
    """取目标云端项目。⚠ 数据面板没打开时 activeProject 会抛
    InternalValidationError : id.size()（2026-09-11 睿君实测），但 dataProjects 仍可用，
    所以按「指定名 → 活动项目 → 第一个非 Assets 项目」依次兜底。
    """
    d = app.data
    # 1) 指定了项目名：直接在列表里找（数据面板没开也能用，最稳）
    if PROJECT_NAME:
        try:
            ps = d.dataProjects
            for i in range(ps.count):
                if ps.item(i).name == PROJECT_NAME:
                    p = ps.item(i)
                    try:
                        d.activeProject = p      # 顺手设成活动项目
                    except Exception:
                        pass
                    log.append("项目=%s（按 PROJECT_NAME 指定）" % p.name)
                    return p
            log.append("项目列表里没有 %r" % PROJECT_NAME)
        except Exception as e:
            log.append("按名找项目失败: %s" % e)
    # 2) 活动项目
    for attempt in range(2):
        try:
            p = d.activeProject
            if p:
                log.append("项目=%s%s" % (
                    p.name, "" if attempt == 0 else "（打开数据面板后取到）"))
                return p
        except Exception as e:
            log.append("activeProject 第 %d 次失败: %s" % (attempt + 1, e))
        if attempt == 0:
            try:
                if not d.isDataPanelVisible:
                    d.isDataPanelVisible = True
                    log.append("已自动打开数据面板")
                d.refreshDataPanel()
            except Exception as e:
                log.append("打开数据面板失败: %s" % e)
    # 3) 兜底：第一个不是 Assets 的项目（Assets 是系统的占位项目）
    try:
        ps = d.dataProjects
        for i in range(ps.count):
            if ps.item(i).name != "Assets":
                p = ps.item(i)
                try:
                    d.activeProject = p
                except Exception:
                    pass
                log.append("退回第一个非 Assets 项目=%s" % p.name)
                return p
    except Exception as e:
        log.append("兜底取项目也失败: %s" % e)
    raise RuntimeError(
        "取不到云端项目，无法保存文件。\n\n"
        "请在 Fusion 里点开左侧的「数据面板」（工具栏上那个九宫格/文件夹图标），\n"
        "确认已登录、且能看到至少一个项目，然后重新运行本脚本。\n\n"
        "诊断记录：\n  " + "\n  ".join(log))


def _hub_folder(app, log):
    """云端项目根目录下的 FOLDER_NAME 文件夹（没有就建）。"""
    proj = _active_project(app, log)
    root_folder = proj.rootFolder
    f = root_folder.dataFolders.itemByName(FOLDER_NAME)
    if not f:
        f = root_folder.dataFolders.add(FOLDER_NAME)
    log.append("目标文件夹=%s/%s" % (proj.name, FOLDER_NAME))
    return f


def _run_parts(app, ui):
    log = []
    folder = _hub_folder(app, log)
    lines = ["== 零件文件已生成（parts）=="] + ["定位   : " + x for x in log]
    for key, fname in PART_FILES:
        # 守卫：目标零件文档还开着就别动，否则会把它对应的云端文件删掉（文档变孤儿）
        for d in app.documents:
            if d.name.startswith(fname):
                raise RuntimeError(
                    "零件文档「%s」还开着。请先关闭 SCARA-* 这些标签页，再重新运行本脚本。"
                    % d.name)
        # 幂等：同名旧文件先删掉，否则 saveAs 会另存成 "SCARA-base (1)" 之类，assy 就找不到了
        old = _find_datafile(folder, fname)
        if old:
            if old.isInUse:
                raise RuntimeError(
                    "云端已存在 %s 且正在被打开使用。请先关闭它的文档标签页，再重新运行。"
                    % old.name)
            old.deleteMe()
            lines.append("%-8s 已删除同名旧文件 %s v%s" % (key, old.name, old.versionNumber))
        doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(app.activeProduct)
        _set_mm(design)
        comp = _new_comp(design.rootComponent, key)
        BUILDERS[key](comp.component)
        paint_info, n_pl, _ = _finish(design.rootComponent, FILLET_R[key])
        ok = doc.saveAs(fname, folder, "", "")
        lines.append("%-8s %-16s 实体=%d  倒角R%.1f/色=%s  保存=%s" % (
            key, fname, comp.component.bRepBodies.count, FILLET_R[key],
            paint_info.split(" / ")[0], "OK" if ok else "失败"))
    lines.append("")
    lines.append("零件文件都是【独立文档】，请自己关闭这 %d 个标签页，再跑 TASK=\"assy\"。" % len(PART_FILES))
    msg = "\n".join(lines)
    print(msg)
    ui.messageBox(msg)


def _find_datafile(folder, name):
    for df in folder.dataFiles:
        if df.name == name or df.name.startswith(name + "."):
            return df
    return None


def _run_assy(app, ui):
    log = []
    folder = _hub_folder(app, log)
    missing = [fn for _, fn in PART_FILES if not _find_datafile(folder, fn)]
    if missing:
        raise RuntimeError("云端 %s/ 里找不到这些零件文件：%s\n请先跑 TASK=\"parts\"。" %
                           (FOLDER_NAME, ", ".join(missing)))

    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    _set_mm(design)
    root = design.rootComponent
    occ = {}
    for key, fname in PART_FILES:
        df = _find_datafile(folder, fname)
        # ⚠ Occurrence.name 是只读的（Component.name 才可写），别想着改名；
        # 零件文件里的组件名本来就是 base/link1/…，插入后 occurrence 自然继承。
        o = root.occurrences.addByInsert(df, adsk.core.Matrix3D.create(), True)
        tr = o.transform.translation
        if abs(tr.x) + abs(tr.y) + abs(tr.z) > 1e-6:
            # ⚠ addByInsert 偶尔不按单位阵落位（2026-09-11 实测：6 个里有 2 个被平移了，
            # 例如 link2 偏 (-5.34, 2.79, 13.34)mm）→ 强制复位，否则关节面按绝对 z 找不到。
            log.append("%s 插入时被偏移 (%.2f,%.2f,%.2f)mm，已复位" % (
                key, tr.x * 10, tr.y * 10, tr.z * 10))
            o.transform2 = adsk.core.Matrix3D.create()
        occ[key] = o
    occ["base"].isGrounded = True      # 基座固定（关节的参考端）

    # ⚠ 顺序不能反：必须【先编刚性组，再建关节】（2026-09-11 实测踩过）。
    # 刚性组记的是"编组那一刻的相对位置"。joint_d3 一建好就会把伸缩轴顶到套筒顶环面
    # （+95mm 落位）；若此时喷嘴还没和轴编组，喷嘴会留在原地 → 组里记下"轴在上、喷嘴在下
    # 95mm"的错位，喷嘴自此永远脱离。先编组：轴 noz 都还在零件文件里的原始相对位置，
    # 编组正确 → 再建 joint_d3 把整组一起抬到位。
    rigid_log = _make_rigid(root, occ)
    _make_joints(root, occ)
    cset = _collision(design, root)

    # 幂等：同名旧装配体先删掉，否则 saveAs 会另存成 "SCARA-assembly (1)"
    for d in app.documents:
        if d.name.startswith(ASSY_NAME) and not d.isActive:
            raise RuntimeError("装配体文档「%s」还开着，请先关闭它的标签页再运行。" % d.name)
    old = _find_datafile(folder, ASSY_NAME)
    if old:
        if old.isInUse:
            raise RuntimeError("云端已存在 %s 且正在被使用，请先关闭它的标签页再运行。" % old.name)
        old.deleteMe()
        log.append("已删除同名旧装配体 %s v%s" % (old.name, old.versionNumber))
    ok = doc.saveAs(ASSY_NAME, folder, "", "")

    msg = "\n".join([
        "== 装配体已生成（assy）=="] + ["定位     : " + x for x in log] + [
        "云端文件 : %s/%s" % (FOLDER_NAME, ASSY_NAME),
        "引用零件 : %s" % ", ".join("%s(%s)" % (k, "引用" if occ[k].isReferencedComponent
                                              else "非引用")
                                  for k, _ in PART_FILES),
        "关节     : %s" % ", ".join("%s(健康=%s)" % (x.name, x.healthState)
                                  for x in root.joints),
        "刚性组   : %s" % ("、".join(rigid_log) if rigid_log else "无"),
        "碰撞     : 接触集=%s（总开关关）" % (cset.name if cset else "无"),
        "自检     : 轴 z=%s  吸嘴 z=%s（期望 轴 55~95、吸嘴 30~55）" % (
            _fmt_range(_zrange(occ["shaft"])), _fmt_range(_zrange(occ["nozzle"]))),
        "保存     : %s" % ("OK" if ok else "失败"),
    ])
    print(msg)
    ui.messageBox(msg)


# ==================== 入口 ====================

def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        if TASK == "single":
            design = adsk.fusion.Design.cast(app.activeProduct)
            if not design:
                ui.messageBox("single 模式：请先新建一个设计再运行本脚本。")
                return
            _run_single(app, ui, design, design.rootComponent)
        elif TASK == "parts":
            _run_parts(app, ui)
        elif TASK == "assy":
            _run_assy(app, ui)
        elif TASK == "scene":
            _run_scene(app, ui)
        else:
            ui.messageBox("TASK 只能是 \"single\" / \"parts\" / \"assy\" / \"scene\"，当前是 %r。" % TASK)
    except Exception:
        if ui:
            ui.messageBox("运行失败:\n{}".format(traceback.format_exc()))
        else:
            print(traceback.format_exc())


def stop(context):
    pass
