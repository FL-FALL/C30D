"""任务4｜救援：用 YOLO 神经网络模型识别白色人质（圆柱 / 圆锥 / 腰鼓）。

模型来自 PC 训练的 best.pt，不能直接在 MaixCam2 上运行，需一次性适配：
  1. 导出 ONNX（PC）：
       yolo export model=best.pt format=onnx imgsz=240,320 opset=17 \
           simplify=True dynamic=False
  2. MaixHub 网页转换（PC）：目标设备选 MaixCAM2、YOLO 版本与权重一致，
     得到 best.mud + best_npu.axmodel + best_vnpu.axmodel 三个文件；
  3. 三个文件放板子同一目录（默认 /root/models/），代码只加载 .mud。

接口与旧版保持一致：main.py 仍以 BGR ndarray 调 detect()，返回 Hostage 列表；
本模块依赖 maix（nn/image），只能在 MaixCam2 上运行，PC 离线调试不再适用。
"""
from dataclasses import dataclass

import cv2

_SHAPE_NAMES = {1: "cylinder", 2: "cone", 3: "barrel"}
# 训练集标签名 → 形状码别名：MaixHub 转换出的 .mud labels = cylinder, frustum, drum
_LABEL_ALIASES = {"cylinder": 1, "frustum": 2, "cone": 2, "drum": 3, "barrel": 3}

SHOW_MASK_VIEW = False  # 产出检测结果可视化（main.py: SHOW_MASK_VIEW）
DEBUG_VIEW = None       # 最近一帧的可视化 ndarray（BGR，与原帧同尺寸）

_DETECTOR = None        # nn.YOLOxx 惰性单例
_CLASS2SHAPE = {}       # class_id → (shape, code)


def configure(show_mask_view=False):
    """由 main.py 注入 SHOW_MASK_VIEW 开关。"""
    global SHOW_MASK_VIEW
    SHOW_MASK_VIEW = bool(show_mask_view)


@dataclass
class Hostage:
    """一件人质的检测结果。"""
    shape: str        # 形状名：cylinder / cone / barrel
    code: int         # 形状码：1 圆柱 / 2 圆锥 / 3 腰鼓
    u: float          # 框中心像素 u（仅用于居中判定与左右排序，不是 3D 坐标）
    v: float          # 框中心像素 v
    w: float          # 框像素宽
    h: float          # 框像素高
    conf: float       # 检测置信度 [0,1]


def _load(cfg):
    """惰性加载 .mud 模型，并按类别标签建立 class_id → 形状码映射。"""
    global _DETECTOR, _CLASS2SHAPE
    if _DETECTOR is not None:
        return
    from maix import nn
    arch = str(cfg.get("model_arch", "yolo11")).lower()
    cls = {"yolo11": nn.YOLO11, "yolov8": nn.YOLOv8, "yolov5": nn.YOLOv5}.get(arch)
    if cls is None:
        raise ValueError("rescue.model_arch must be yolo11/yolov8/yolov5, got %r" % arch)
    path = str(cfg.get("model_path", "/root/models/best.mud"))
    _DETECTOR = cls(model=path, dual_buff=bool(cfg.get("dual_buff", False)))

    labels = list(getattr(_DETECTOR, "labels", None) or [])
    mapping = {}
    for i, name in enumerate(labels):                 # 先按标签名（含别名）匹配形状码
        code = _LABEL_ALIASES.get(name.strip().lower())
        if code is not None:
            mapping[i] = (_SHAPE_NAMES[code], code)
    for i in range(len(labels)):                      # 未匹配的按顺序回退：类0→码1…
        mapping.setdefault(i, (_SHAPE_NAMES.get(i + 1, labels[i]), i + 1))
    _CLASS2SHAPE = mapping
    print("[rescue] loaded %s arch=%s labels=%s" % (path, arch, labels))


def _crop_roi(frame_bgr, cfg):
    """按 config 的归一化 roi=[x0,y0,x1,y1] 裁剪，返回 (roi, 原图偏移 x0, y0)。"""
    h, w = frame_bgr.shape[:2]
    x0, y0, x1, y1 = cfg.get("roi", [0.0, 0.0, 1.0, 1.0])
    x0 = int(max(0, min(w - 1, round(x0 * w))))
    y0 = int(max(0, min(h - 1, round(y0 * h))))
    x1 = int(max(x0 + 1, min(w, round(x1 * w))))
    y1 = int(max(y0 + 1, min(h, round(y1 * h))))
    return frame_bgr[y0:y1, x0:x1], x0, y0


def _debug_view(frame_bgr, found):
    """把检测框画到原帧副本上供上屏（黄框 + 形状码文字）。"""
    vis = frame_bgr.copy()
    for o in found:
        x0, y0 = int(o.u - o.w / 2.0), int(o.v - o.h / 2.0)
        x1, y1 = int(o.u + o.w / 2.0), int(o.v + o.h / 2.0)
        cv2.rectangle(vis, (x0, y0), (x1, y1), (0, 255, 255), 2)
        cv2.putText(vis, "%s:%d %.2f" % (o.shape, o.code, o.conf),
                    (x0, max(12, y0 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
    return vis


def detect(frame_bgr, cfg):
    """YOLO 检测一帧，返回全部候选（按置信度降序，最多 cfg["max_candidates"] 件）。

    点位顺序与目标形状的匹配仍由 main.py 的 PositionTracker 处理。

    参数:
        frame_bgr: BGR 图像（OpenCV ndarray，由 main.py 的 image2cv 转换）。
        cfg:       config.yaml 的 rescue 段。
    返回:
        Hostage 列表；无候选返回空列表。
    """
    global DEBUG_VIEW
    _load(cfg)
    from maix import image as mimage

    roi, ox, oy = _crop_roi(frame_bgr, cfg)
    objs = _DETECTOR.detect(mimage.cv2image(roi),
                            conf_th=float(cfg.get("conf_th", 0.5)),
                            iou_th=float(cfg.get("iou_th", 0.45)))
    found = []
    for o in objs:
        pair = _CLASS2SHAPE.get(o.class_id)
        if pair is None:
            continue
        shape, code = pair
        x, y, w, h = o.box()
        found.append(Hostage(shape, code, ox + x + w / 2.0, oy + y + h / 2.0,
                             float(w), float(h), float(o.score)))
    found.sort(key=lambda o: o.conf, reverse=True)
    found = found[:int(cfg.get("max_candidates", 3))]
    if SHOW_MASK_VIEW:
        DEBUG_VIEW = _debug_view(frame_bgr, found)
    return found
