"""中央 ROI 数字放大：仅 target 任务使用。

思路：
  - 采集分辨率仍是宽高等比（默认 320×240），不增加采集开销；
  - 只把画面中央一小块裁出来，再放大到 zoom_width 后检测，等效「数字长焦」：
    远景靶在检测坐标系里占更多像素，最小外接圆与椭圆拟合的量化误差被摊薄；
  - 信息量不增加（仍是原来的像素），但测距 Z、圆心 u/v、yaw/pitch 的抖动会明显变小。

代价：ROI 外的目标看不到，因此只在 ROI 内计点位。

坐标约定：
  检测结果在「放大后的 ROI 坐标系」内；绘制回原帧用 to_frame()，内参用
  zoom_intrinsics() 同步换算后，测距与瞄准角仍按原物理量计算。

只依赖 cv2，不 import maix，可在 PC 上离线调试。
"""
import cv2

DEFAULT_ROI = [0.30, 0.30, 0.70, 0.70]   # 缺省：画面正中 40%×40%


def extract(frame, cfg):
    """按 cfg 的 detect_roi 裁剪中央区域并放大，返回 (crop, ox, oy, scale)。

    参数:
        frame: BGR 原帧。
        cfg:   task 配置段，含 detect_roi=[x0,y0,x1,y1]（归一化）与 zoom_width。
    返回:
        crop  —— 裁剪并放大后的图像（检测输入）；
        ox,oy —— ROI 左上角在原帧中的像素坐标；
        scale —— 放大倍数（crop 宽 / ROI 宽）；还原用 u_原 = ox + u/scale。
    """
    h, w = frame.shape[:2]
    x0, y0, x1, y1 = cfg.get("detect_roi", DEFAULT_ROI)
    x0 = int(max(0, min(w - 1, round(x0 * w))))
    y0 = int(max(0, min(h - 1, round(y0 * h))))
    x1 = int(max(x0 + 1, min(w, round(x1 * w))))
    y1 = int(max(y0 + 1, min(h, round(y1 * h))))
    crop = frame[y0:y1, x0:x1]

    zoom_w = int(cfg.get("zoom_width", 0) or 0)
    scale = 1.0
    if zoom_w > 0:
        crop_h, crop_w = crop.shape[:2]
        scale = zoom_w / float(crop_w)
        crop = cv2.resize(crop, (zoom_w, max(1, int(round(crop_h * scale)))),
                          interpolation=cv2.INTER_LINEAR)
    return crop, x0, y0, scale


def to_frame(u, v, ox, oy, scale):
    """ROI 坐标 → 原帧坐标（点）。"""
    return ox + u / scale, oy + v / scale


def len_to_frame(value, scale):
    """ROI 中的长度（半径 / 框宽高）→ 原帧长度。"""
    return value / scale


def zoom_intrinsics(intr, ox, oy, scale):
    """把采集分辨率下的内参换算成「放大后的 ROI」内参。

    推导：原帧 u = fx·X/Z + cx，ROI 像素 u' = scale·(u - ox)，故
        fx' = scale·fx,  cx' = scale·(cx - ox)
    fx' 与放大后的像素半径 r' 同比放大，尺寸法测距 Z 与原帧一致；
    yaw/pitch 是比值，同样不受裁剪与放大影响。
    """
    if intr is None or scale == 1.0:
        return intr
    K = intr["K"].copy()
    K[0, 0] *= scale
    K[0, 2] = scale * (intr["cx"] - ox)
    K[1, 1] *= scale
    K[1, 2] = scale * (intr["cy"] - oy)
    return {"K": K, "dist": intr["dist"],
            "fx": intr["fx"] * scale, "fy": intr["fy"] * scale,
            "cx": K[0, 2], "cy": K[1, 2],
            "image_size": (0, 0)}          # ROI 坐标系无标定分辨率，下游不再二次缩放
