"""像素 ↔ 相机系坐标变换（针孔模型）。

坐标约定：
  - 像素系：原点在图像左上角，u 向右、v 向下，单位 px。
  - 相机系：原点在光心，X 向右、Y 向下、Z 沿光轴向前，单位 mm。

核心是针孔（pinhole）成像模型：空间点 P=(X, Y, Z) 投影到像素为
    u = fx · X / Z + cx
    v = fy · Y / Z + cy
其中内参矩阵 K = [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]；fx、fy 以像素为单位，
一般相等（正方形像素），(cx, cy) 是光轴与像面的交点（主点）。
反投影（已知像素与深度 Z）就是上式的逆，即下面 pixel_to_camera 的实现。
"""
import json
import math

import numpy as np


def load_intrinsics(path):
    """读取内参 JSON，返回内参字典。

    测距、反投影、角度偏移全部依赖 fx/fy/cx/cy；换分辨率后内参会随像素缩放，
    需要单独标定。

    参数:
        path: intrinsics.json 路径。
    返回:
        dict，含 "K"(3x3 内参矩阵)、"dist"(畸变系数)、"fx"、"fy"、"cx"、"cy"。
    兼容两种存储格式：直接存 K 矩阵，或分开存 fx/fy/cx/cy。
    """
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    if "K" in data:
        K = np.array(data["K"], dtype=np.float64).reshape(3, 3)
    else:
        K = np.array([[data["fx"], 0.0, data["cx"]],
                      [0.0, data["fy"], data["cy"]],
                      [0.0, 0.0, 1.0]], dtype=np.float64)
    dist = np.array(data.get("dist", [0, 0, 0, 0, 0]), dtype=np.float64)
    size = data.get("image_size") or [0, 0]
    return {"K": K, "dist": dist,
            "fx": float(K[0, 0]), "fy": float(K[1, 1]),
            "cx": float(K[0, 2]), "cy": float(K[1, 2]),
            "image_size": (int(size[0]), int(size[1]))}


def scale_intrinsics(intr, width, height):
    """把内参从标定分辨率缩放到实际采集分辨率。

    数字缩放（ISP 降采样 / resize）下像素坐标等比变化，故
        sx = width / iw,  sy = height / ih
        fx' = fx·sx,  fy' = fy·sy,  cx' = cx·sx,  cy' = cy·sy
    畸变系数定义在归一化像平面上，与分辨率无关，保持不变。
    测距 Z = fx·D/(2r) 中 fx 与 r 同比缩放，结果不变；yaw/pitch 用比值也不变。

    参数:
        intr:         load_intrinsics 返回的内参字典（含 image_size）。
        width, height: 实际采集分辨率。
    返回:
        缩放后的新字典；标定尺寸缺失或与目标一致时原样返回。
    """
    iw, ih = intr.get("image_size", (0, 0))
    if iw <= 0 or ih <= 0:
        return intr
    sx, sy = float(width) / iw, float(height) / ih
    if abs(sx - 1.0) < 1e-6 and abs(sy - 1.0) < 1e-6:
        return intr
    K = intr["K"].copy()
    K[0, 0] *= sx
    K[0, 2] *= sx
    K[1, 1] *= sy
    K[1, 2] *= sy
    return {"K": K, "dist": intr["dist"],
            "fx": intr["fx"] * sx, "fy": intr["fy"] * sy,
            "cx": intr["cx"] * sx, "cy": intr["cy"] * sy,
            "image_size": (int(width), int(height))}


def distance_from_radius(fx, diameter_mm, radius_px):
    """尺寸法（已知尺寸）单目测距：Z = fx · D / (2r)。

    原理：由针孔模型，直径 D(mm) 的物体在距离 Z(mm) 处的像尺寸为
        d_px = fx · D / Z
    于是 Z = fx · D / d_px = fx · D / (2r)，r 为像素半径。

    精度特点：误差 ∝ Z² / (fx·D)，即越远、物体越小误差越大；
    所以必须用准确的内参 fx，且球半径/靶盘半径要量准（高光/边缘会影响 r）。

    参数:
        fx:         x 方向焦距（像素）。
        diameter_mm: 已知的真实直径（球的 D=40mm，靶外盘 Ø100）。
        radius_px:   实测像素半径 r。
    返回:
        距离 Z（mm）。r 趋近 0 时做下限保护，避免除零。
    """
    r = max(float(radius_px), 1e-6)
    return float(fx) * float(diameter_mm) / (2.0 * r)


def pixel_to_camera(u, v, Z, intr):
    """反投影：像素坐标 + 深度 → 相机坐标系 (X, Y, Z)。

    原理：针孔模型反解。已知像素 (u, v) 与其深度 Z，
        X = (u - cx) · Z / fx,   Y = (v - cy) · Z / fy
    Z 无法从单张图直接得到，需由尺寸法、深度相机或双目等外部手段提供。

    参数:
        u, v: 像素坐标（px）。
        Z:    该点深度（mm）。
        intr: load_intrinsics 返回的内参字典。
    返回:
        (X, Y, Z) 相机系坐标（mm）。
    """
    X = (u - intr["cx"]) * Z / intr["fx"]
    Y = (v - intr["cy"]) * Z / intr["fy"]
    return float(X), float(Y), float(Z)


def pixel_to_angles(u, v, intr):
    """像素 → 相对光轴的角度偏移（度），用于给云台/枪口瞄准。

    原理：由反投影 X/Z = (u-cx)/fx、Y/Z = (v-cy)/fy，
        yaw   ≈ atan((u - cx) / fx)
        pitch ≈ atan((v - cy) / fy)
    这是目标相对光轴的偏航/俯仰角，可直接作为云台转角指令（忽略畸变与滚转）。

    参数:
        u, v: 靶心像素坐标。
        intr: 内参字典。
    返回:
        (yaw, pitch) 角度（度）。
    """
    yaw = math.degrees(math.atan((u - intr["cx"]) / intr["fx"]))
    pitch = math.degrees(math.atan((v - intr["cy"]) / intr["fy"]))
    return yaw, pitch