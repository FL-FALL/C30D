"""任务1｜二维码解码 —— MaixCam 板载实现（设备侧）。

main.py 只调用 decode() 与 draw_results()。本模块的图像类型是 MaixPy 的 image.Image
（不是 ndarray），依赖 maix 模块，只能在 MaixCam 上运行。
"""
from maix import image

_DETECTOR = None        # image.QRCodeDetector 惰性单例；False 表示当前固件不支持


def _codes(img):
    """优先用 NPU 加速的 QRCodeDetector（MaixPy ≥4.7.9），否则退回 find_qrcodes。"""
    global _DETECTOR
    if _DETECTOR is None:
        cls = getattr(image, "QRCodeDetector", None)
        _DETECTOR = cls() if cls is not None else False
    return img.find_qrcodes() if _DETECTOR is False else _DETECTOR.detect(img)


def decode(img):
    """识别一帧中的二维码，返回 [{"payload", "corners"}, ...]；未识别到返回空列表。"""
    return [{"payload": qr.payload(), "corners": qr.corners()}
            for qr in _codes(img)]


def draw_results(img, results):
    """用四顶点画框（不绘制文字，内容只走终端打印）；就地修改并返回 img。"""
    for r in results:
        corners = r["corners"]
        for i in range(4):
            x1, y1 = corners[i]
            x2, y2 = corners[(i + 1) % 4]
            img.draw_line(x1, y1, x2, y2, image.COLOR_RED)
    return img


def release():
    """Drop the QR accelerator before changing camera resolution."""
    global _DETECTOR
    _DETECTOR = None
