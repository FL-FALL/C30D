"""Single target-color XY only. No depth, calibration, hue statistics or tracking."""
import cv2
import numpy as np


class XYDetector:
    def __init__(self, cfg, target_color):
        self.width = max(32, int(cfg.get("xy_detect_width", 160)))
        self.min_area = max(1.0, float(cfg.get("xy_min_area_px", 6)))
        self.min_ratio = max(0.0, float(cfg.get("min_area_ratio", 0.000217)))
        self.min_aspect = float(cfg.get("xy_min_aspect", 0.4))
        self.target = target_color
        self.ranges_by_color = {
            c: [(np.array(lo, dtype=np.uint8), np.array(hi, dtype=np.uint8)) for lo, hi in ranges]
            for c, ranges in cfg["hsv"].items()}


    def detect(self, frame):
        return self.detect_colors(frame, (self.target,)).get(self.target)

    def detect_colors(self, frame, colors=(1, 2, 3)):
        h, w = frame.shape[:2]
        sw = min(w, self.width)
        sh = max(1, int(round(h * sw / w)))
        small = cv2.resize(frame, (sw, sh), interpolation=cv2.INTER_NEAREST) if sw != w else frame
        hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
        points = {}
        for color in colors:
            ranges = self.ranges_by_color.get(color, [])
            if not ranges:
                continue
            point = self._point(hsv, ranges, w, h, sw, sh)
            if point is not None:
                points[color] = point
        return points

    def _point(self, hsv, ranges, w, h, sw, sh):
        mask = None
        for lo, hi in ranges:
            part = cv2.inRange(hsv, lo, hi)
            mask = part if mask is None else cv2.bitwise_or(mask, part)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None
        best_area = max(self.min_area, self.min_ratio * sw * sh)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < best_area:
                continue
            x, y, bw, bh = cv2.boundingRect(contour)
            clipped = x == 0 or y == 0 or x+bw >= sw or y+bh >= sh
            if not clipped and min(bw, bh) / max(bw, bh) < self.min_aspect:
                continue
            best, best_area = contour, area
        if best is None:
            return None
        # Filled external contour centroid: interior highlight holes do not bias it.
        m = cv2.moments(best)
        if m["m00"] <= 0:
            return None
        x = round((m["m10"] / m["m00"]) * w / sw)
        y = round((m["m01"] / m["m00"]) * h / sh)
        return max(0, min(w-1, x)), max(0, min(h-1, y))
