"""任务3｜靶子距离 + 圆心。见 README 任务3。

靶面同心结构：外红盘 Ø100 → 白环 → Ø35 → Ø30 → Ø15 → Ø10。
粗测：一帧内同时检测红/绿/蓝三种外盘，按颜色各取最优候选，最小外接圆
      → 尺寸法测距 Z=fx·100/d_px，圆心即 (u,v)；
精测：外圈椭圆拟合，在距离之外给出靶面倾角/法向，作为鲁棒性升级项。
点位顺序由 main.py 的 PositionTracker 统一维护，本模块只返回逐色检测结果。
"""
import math
from dataclasses import dataclass

import cv2

from common import geometry
from common.color import build_color_mask, detect_all_colors

_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))


@dataclass
class TargetResult:
    """单个靶子的定位结果。"""
    u: float          # 圆心像素 u
    v: float          # 圆心像素 v
    r: float          # 目标色块像素半径
    Z: float          # 距离 mm
    yaw: float        # 靶心相对光轴偏航角（度），供瞄准
    pitch: float      # 靶心相对光轴俯仰角（度），供瞄准
    tilt_deg: float = 0.0      # 靶面相对光轴的倾角（度），椭圆精测给出（0=正对）
    normal_yaw: float = 0.0    # 靶面法向偏航角（度），椭圆精测给出
    normal_pitch: float = 0.0  # 靶面法向俯仰角（度），椭圆精测给出
    conf: float = 0.5          # 检测置信度（圆度×填充率×色相集中度），供调阈值时观察
    color: int = 0             # 颜色编号：1=红 2=绿 3=蓝


def detect(frame, cfg, intr):
    """粗测：一帧内同时检测三种颜色的靶面 → 尺寸法测距 + 圆心 + 角度偏移。

    原理：
      - 按 cfg["hsv"] 中的红/绿/蓝阈值分别分割，每色取置信度最高的一个候选
        （共用 common.color.detect_all_colors，内含形态学去噪/填洞、颜色复核与置信度过滤）；
      - 取该候选（靶面）的像素半径 r，尺寸法 Z = fx·outer_diameter_mm/(2r)；
      - 圆心像素经 pixel_to_angles 换算为相对光轴的 yaw/pitch，可直接给云台/枪口。

    参数:
        frame: BGR 图像。
        cfg:   target 配置（hsv / outer_diameter_mm 等）。
        intr:  内参字典。
    返回:
        TargetResult 列表，按颜色编号升序，每色最多 1 个；未检出任何靶面时为空列表。
    """
    # 仅检测中央ROI，结果还原到整幅画面坐标，保持底盘对中基准不变。
    ox = oy = 0
    detect_frame = frame
    box = cfg.get("detect_roi")
    if box is not None:
        h, w = frame.shape[:2]
        ox, oy = int(box[0] * w), int(box[1] * h)
        x1, y1 = int(box[2] * w), int(box[3] * h)
        detect_frame = frame[oy:y1, ox:x1]
    results = []
    for color, u, v, r, conf in detect_all_colors(detect_frame, cfg):
        u, v = u + ox, v + oy
        if cfg.get("center_only", False):
            # 底盘只需要圆心；跳过距离和角度解算，保持原有颜色筛选及置信度算法。
            Z = yaw = pitch = 0.0
        else:
            Z = geometry.distance_from_radius(intr["fx"], cfg["outer_diameter_mm"], r)
            yaw, pitch = geometry.pixel_to_angles(u, v, intr)
        results.append(TargetResult(u, v, r, Z, yaw, pitch, conf=conf, color=color))
    return results


def _fit_ellipse(contour):
    """对轮廓做椭圆拟合，返回 ((cx,cy), 长半轴 a, 短半轴 b, 长轴倾角°)；点数不足返回 None。

    投影下平面圆变成椭圆：长半轴 a = fx·R/Z 与透视压缩无关（直接给出距离），
    短半轴 b = a·cosτ 携带靶面倾角 τ（靶面法向与光轴夹角）。
    """
    if len(contour) < 5:                       # cv2.fitEllipse 至少需要 5 个点
        return None
    (cx, cy), (w, h), ang = cv2.fitEllipse(contour)
    if w >= h:
        a, b, major = w / 2.0, h / 2.0, ang
    else:
        a, b, major = h / 2.0, w / 2.0, ang + 90.0
    if a <= 0.0:
        return None
    return (float(cx), float(cy)), float(a), float(b), float(major)


def fit_concentric(roi, cfg, intr):
    """精测（推荐）：椭圆拟合最外圈，给出距离 + 靶面倾角/法向。

    原理：
      - 平面圆在透视投影下成为椭圆：长半轴 a = fx·R/Z（R 为外盘真实半径）与透视
        压缩无关，故外盘长半轴直接给出距离 Z = fx·(outer_diameter_mm/2)/a；
      - 短半轴 b = a·cosτ，τ 是靶面法向与光轴的夹角，于是 τ = arccos(b/a)；
        椭圆短轴方向即倾斜方位，据此把 τ 分解为法向的偏航/俯仰。

    与 detect() 的分工：detect 用最小外接圆做尺寸法粗测；本函数改用椭圆拟合，除距离
    外多给靶面姿态，且对高光造成的边缘缺口更不敏感。

    注意：单圆法向的方位存在 ±180° 镜像二义性（倾斜向上/向下投影同形），此处取椭圆
    短轴方向为约定解；只关心距离与靶心时不受影响。

    参数:
        roi:  靶面附近的区域（BGR，比整帧小以减少背景干扰）。
        cfg:  target 配置（color / hsv / outer_diameter_mm）。
        intr:  内参字典。
    返回:
        TargetResult；未检出外盘轮廓时返回 None。

    坐标约定：返回的 u、v 及 yaw/pitch 都以 roi 为坐标系。若 roi 是整帧的局部裁剪，
    调用方需让 intr["cx"]、intr["cy"] 减去 roi 左上角偏移后再传入，否则角度会偏。
    """
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = build_color_mask(hsv, cfg["hsv"][cfg["color"]])
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _KERNEL)     # 去噪
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _KERNEL)    # 填洞，别让外盘变环
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        return None
    fit = _fit_ellipse(max(contours, key=cv2.contourArea))     # 最大色块 = 外盘
    if fit is None:
        return None
    (u, v), a_out, b_out, major_deg = fit
    Z = geometry.distance_from_radius(intr["fx"], cfg["outer_diameter_mm"], a_out)
    # 靶面法向：倾角 τ = arccos(b/a)，方位取椭圆短轴方向（长轴 +90°）
    cos_tau = min(1.0, max(0.0, b_out / a_out))
    tilt = math.degrees(math.acos(cos_tau))
    az = math.radians(major_deg + 90.0)
    st = math.sin(math.radians(tilt))
    n_x, n_y, n_z = st * math.cos(az), st * math.sin(az), cos_tau
    normal_yaw = math.degrees(math.atan2(n_x, n_z))
    normal_pitch = math.degrees(math.atan2(n_y, n_z))
    yaw, pitch = geometry.pixel_to_angles(u, v, intr)          # 指向靶心的瞄准角
    return TargetResult(u, v, a_out, Z, yaw, pitch, tilt, normal_yaw, normal_pitch)
