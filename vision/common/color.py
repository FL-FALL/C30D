"""按颜色分割并提取圆形目标：target 与 balls 共用的核心逻辑。

两个任务的第一步本质相同——按颜色阈值筛选色块，再对每个候选色块求最小外接圆，
得到圆心 (u, v) 与像素半径 r。
  - detect_color_circles：只处理 cfg["color"] 指定的单色（保留给只需要单色的场景）；
  - detect_all_colors：一次预处理同时处理 cfg["hsv"] 中的全部颜色，供按出现顺序记点位。
区别只在下游：balls 由 r 做尺寸法测距并反投影到相机系；target 取最大色块求距离与角度。
"""
import cv2
import numpy as np

# 以下参数与开关都由 main.py 在启动时统一注入（参数值来自 config.yaml 的 detect 段，开关定义在
# main.py 顶部）。模块里的初值只是离线直接 import 本模块时的兜底默认，设备运行以注入值为准。
_DETECT_WIDTH = 320     # 检测降采样宽度上限 px（config: detect.detect_width）；0=不降采样
_KERNEL = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 1))   # config: detect.kernel_size；
                                                               # 矩形核走可分离优化（5+5 抽头），
                                                               # 椭圆核不可分离（25 抽头），RISC-V 上慢 2-3 倍
_DEBUG = False          # 逐帧打印检测细节（main.py: DEBUG_DETECT；帧率杀手，量产须关）
SHOW_MASK_VIEW = False  # 产出掩膜可视化（main.py: SHOW_MASK_VIEW）
DEBUG_VIEW = None       # 最近一帧的掩膜可视化 ndarray（BGR，与原帧同尺寸）


def configure(detect_cfg=None, debug=False, show_mask_view=False):
    """由 main.py 注入 config.yaml 的 detect 段与两个调试开关。

    参数:
        detect_cfg:     config.yaml 的 detect 段，含 detect_width（降采样宽度上限，0=不降采样）
                        与 kernel_size（形态学椭圆核边长 px）。
        debug:          是否逐帧打印检测细节（来自 main.py 的 DEBUG_DETECT 开关）。
        show_mask_view: 是否产出掩膜可视化并存入 DEBUG_VIEW（来自 main.py 的 SHOW_MASK_VIEW 开关）。
    """
    global _DETECT_WIDTH, _KERNEL, _DEBUG, SHOW_MASK_VIEW
    detect_cfg = detect_cfg or {}
    _DETECT_WIDTH = int(detect_cfg.get("detect_width", _DETECT_WIDTH))
    k = int(detect_cfg.get("kernel_size", 5))
    _KERNEL = cv2.getStructuringElement(cv2.MORPH_RECT, (k, k))
    _DEBUG = bool(debug)
    SHOW_MASK_VIEW = bool(show_mask_view)


def build_color_mask(hsv, ranges):
    """多段 HSV 区间 → 单色二值掩膜（各区间按位或合并）。

    参数:
        hsv:    cv2.cvtColor(..., COLOR_BGR2HSV) 结果。
        ranges: [[lo, hi], ...]；红色跨 H 的 0/180 边界需两段，故支持多区间。
    返回:
        单通道 0/255 掩膜。
    """
    mask = None
    for lo, hi in ranges:
        m = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        mask = m if mask is None else cv2.bitwise_or(mask, m)
    return mask


def _hue_in_ranges(h, ranges):
    """色相 h 是否落在任一段目标色区间内（每段格式 [[H_lo,S_lo,V_lo],[H_hi,...]]）。"""
    return any(lo[0] <= h <= hi[0] for lo, hi in ranges)


def _region_hue_stats(hsv, mask, contour, bbox):
    """在轮廓的「实际色块像素」上统计色相，返回 (中位色相, 集中度)。

    取「轮廓填充区域 ∩ 颜色掩膜」，而不是整个填充区域：靶面是同心结构，外盘轮廓的填充
    区域会把中间的白环（非目标色）像素也算进去，只看填充区域会把中位色相拉偏、把集中度
    压低，导致环形靶面被判成非目标色而漏检。与掩膜求交后只统计真正属于目标色的像素，
    对球（实心）和靶（同心/环形）都成立。

    集中度用环形统计：H∈[0,180) 先映射到单位圆角度，再求合成向量长度 R（resultant
    length）。纯色块色相高度集中 R→1；杂色/噪声像素分散 R→0；红色跨 0/180 的两段区间
    在圆上本就相邻，故 R 不会因分段而失真。直接算标准差会在 H 的 0/180 相接处失真，不用。

    参数:
        hsv:      cv2.cvtColor(..., COLOR_BGR2HSV) 结果。
        mask:     目标色二值掩膜，与 hsv 同尺寸。
        contour:  cv2.findContours 得到的单个轮廓（坐标为整幅图像坐标系）。
        bbox:     cv2.boundingRect(contour) 结果，用于把临时掩膜限制在小范围内。
    返回:
        (中位色相, 集中度)；交集内无像素时返回 (-1, 0.0)。
    """
    x, y, w, h = bbox
    local = np.zeros((h, w), np.uint8)
    cv2.drawContours(local, [contour - (x, y)], -1, 255, cv2.FILLED)
    sub = hsv[y:y + h, x:x + w]
    sel = (local > 0) & (mask[y:y + h, x:x + w] > 0)     # 填充区域 ∩ 掩膜
    hues = sub[:, :, 0][sel]
    if hues.size == 0:
        return -1.0, 0.0
    med = float(np.median(hues))                        # 中位数抗高光/阴影离群像素
    ang = np.deg2rad(hues.astype(np.float32) * 2.0)     # H∈[0,180) → 单位圆角度
    conc = float(np.hypot(np.mean(np.cos(ang)), np.mean(np.sin(ang))))
    return med, conc


def _dbg(i, fmt, *a):
    """候选轮廓的分阶段调试打印；_DEBUG 关闭时不求值（格式化参数惰性传入）。"""
    if _DEBUG:
        print("[dbg]  c%d " % i + fmt % a)


def _debug_dump(hsv, mask, color, ranges, scale, contours, min_area, min_circ, min_conf):
    """设备端调试：打印掩膜规模、高饱和像素色相主峰、当前颜色窗口覆盖率。

    只在 _DEBUG=True 时调用。用它确认某个颜色死在哪一环：掩膜像素太少或色相跑出窗口
    （本函数），还是下游被面积/圆度/置信度滤掉（每个候选的 DROP 行）。高饱和指
    S、V 均 ≥ 60。
    """
    sh, sw = mask.shape[:2]
    print("[dbg] frame %dx%d scale=%.3f color=%d mask=%dpx min_area=%.1f min_circ=%.2f min_conf=%.2f 轮廓数=%d"
          % (sw, sh, scale, color, int((mask > 0).sum()), min_area, min_circ, min_conf, len(contours)))
    S, V, H = hsv[..., 1], hsv[..., 2], hsv[..., 0]
    sel = (S >= 60) & (V >= 60)
    hues = H[sel]
    if hues.size == 0:
        print("[dbg] 高饱和(S,V>=60)像素为 0：画面里没有明显有色目标")
        return
    hist = np.bincount(hues, minlength=180).astype(np.float64)
    pad = np.concatenate([hist[-4:], hist, hist[:4]])
    sm = np.convolve(pad, np.ones(9) / 9.0, mode="same")[4:184]
    top = np.argsort(sm)[::-1][:5]
    peaks = " ".join("H%d:%d" % (p, int(hist[p])) for p in sorted(top))
    print("[dbg] 高饱和像素 H 主峰 -> %s ；落在当前颜色窗口内的占 %.0f%%（窗口=%s）"
          % (peaks, float((mask[sel] > 0).mean()) * 100.0, ranges))


def _debug_view(frame, mask, contours, best, scale):
    """把「算法真正看到的红色区域」画成一张与原帧同尺寸的 BGR 图，供 main.py 开关直接上屏。

    掩膜像素染绿、全部候选轮廓画黄、最终选中的拟合圆画蓝并标红色圆心十字。一看绿区形态即可
    判断病因：圆盘缺一块（月牙/凹口，凸包能救）、圆球拖出细长尾巴（与背景连通，凸包救不了，
    需换几何检测）、被切成多块（高光/阴影割断，需调形态学或阈值）。坐标已还原到原帧分辨率。
    """
    h, w = frame.shape[:2]
    m = cv2.resize(mask, (w, h), interpolation=cv2.INTER_NEAREST)
    vis = frame.copy()
    tint = np.zeros_like(vis)
    tint[m > 0] = (0, 255, 0)                                          # 掩膜像素染绿
    vis = cv2.addWeighted(vis, 0.55, tint, 0.45, 0)
    cs = contours if scale >= 1.0 else \
        [(c.astype(np.float32) * (1.0 / scale)).astype(np.int32) for c in contours]
    cv2.drawContours(vis, cs, -1, (0, 255, 255), 2)                    # 全部候选轮廓：黄
    bests = best if isinstance(best, list) else [best]
    for b in bests:
        if b is None:
            continue
        u, v, r = int(round(b[0])), int(round(b[1])), int(round(b[2]))
        cv2.circle(vis, (u, v), r, (255, 0, 0), 2)                     # 选中的拟合圆：蓝
        cv2.drawMarker(vis, (u, v), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)   # 圆心：红
    return vis


def _preprocess(frame):
    """按 _DETECT_WIDTH 降采样，返回 (small, scale)；scale 用于把结果坐标还原回原帧。"""
    h, w = frame.shape[:2]
    scale = min(1.0, _DETECT_WIDTH / float(w)) if _DETECT_WIDTH else 1.0   # 只在过宽时降采样
    if scale < 1.0:
        small = cv2.resize(frame, (max(1, int(round(w * scale))), max(1, int(round(h * scale)))),
                           interpolation=cv2.INTER_AREA)
    else:
        small = frame
    return small, scale


def _detect_one(small, hsv, cfg, color, scale):
    """在已转 HSV 的小图上按指定颜色筛选，返回 (best, mask, contours)。

    best 为 (u, v, r, conf)，坐标已按 scale 还原到原分辨率；未检出为 None。
    判据：面积下限 → 凸包圆度/填充率（凸包可免疫高光凹口）→ 颜色复核 →
    置信度 = 圆度 × 填充率 × 色相集中度 → 置信度下限 → 取最高者。
    """
    sh, sw = small.shape[:2]
    total_px = float(sh * sw)
    # 面积下限按占整帧像素的比例换算成绝对 px²，与分辨率无关（降采样后同样成立）
    min_area = cfg.get("min_area_ratio", 0.0) * total_px
    min_circularity = cfg.get("min_circularity", 0.0)
    min_confidence = cfg.get("min_confidence", 0.0)
    ranges = cfg["hsv"][color]
    mask = build_color_mask(hsv, ranges)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _KERNEL)     # 去噪
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _KERNEL)    # 填洞（高光）
    best = None
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if _DEBUG:
        _debug_dump(hsv, mask, color, ranges, scale, contours, min_area, min_circularity, min_confidence)
    for i, c in enumerate(contours):
        area = cv2.contourArea(c)
        if area < min_area:                                  # 面积下限（无上限：避免近距离大球被丢）
            _dbg(i, "area=%.0f<%.0f DROP", area, min_area)
            continue
        (u, v), r = cv2.minEnclosingCircle(c)
        if r <= 0:
            continue
        x, y, bw, bh = cv2.boundingRect(c)
        # 凸包：球面镜面高光/遮挡会在掩膜外缘啃出凹口，使周长暴涨、圆度与填充率一起崩
        #（实测红球 area=11250 却 circ=0.33/fill=0.52 正是这种凹口）。改用凸包算形状指标可填掉
        # 凹口；干净圆球的凸包≈原轮廓，结果不变。sol=实心度（面积/凸包面积），用于设备端定位凹口。
        hull = cv2.convexHull(c)
        hull_area = cv2.contourArea(hull)
        sol = area / hull_area if hull_area > 0 else 0.0
        # 轮廓触到画面边缘即被截断，此时的圆度必然偏低但并非形状不规则，故跳过圆度判据
        clipped = x <= 0 or y <= 0 or x + bw >= sw or y + bh >= sh
        if clipped:
            circ = 1.0
        else:
            peri = cv2.arcLength(hull, True)                    # 圆度 = 4πA/P²，圆的值为 1
            circ = 4.0 * np.pi * hull_area / (peri * peri) if peri > 0 else 0.0
            if circ < min_circularity:
                _dbg(i, "area=%.0f sol=%.2f circ=%.2f<%.2f DROP", area, sol, circ, min_circularity)
                continue
        fill = min(1.0, hull_area / (np.pi * r * r))            # 填充率：凸包占外接圆的比例
        med, conc = _region_hue_stats(hsv, mask, c, (x, y, bw, bh))
        if med < 0 or not _hue_in_ranges(med, ranges):
            _dbg(i, "area=%.0f medH=%.0f 色相出窗 DROP", area, med)
            continue                                         # 主色调不在目标色窗内：同色异物
        conf = circ * fill * conc
        if conf < min_confidence:                            # 置信度过低直接过滤
            _dbg(i, "area=%.0f sol=%.2f circ=%.2f fill=%.2f conc=%.2f conf=%.2f<%.2f DROP",
                 area, sol, circ, fill, conc, conf, min_confidence)
            continue
        _dbg(i, "area=%.0f sol=%.2f circ=%.2f fill=%.2f medH=%.0f conc=%.2f conf=%.2f KEEP",
             area, sol, circ, fill, med, conc, conf)
        if best is None or conf > best[3]:
            best = (float(u) * (1.0 / scale), float(v) * (1.0 / scale),
                    float(r) * (1.0 / scale), float(conf))
    return best, mask, contours


def detect_color_circles(frame, cfg):
    """按 cfg["color"] 筛选出置信度最高的一个圆形色块，返回 [(u, v, r, confidence)] 或 []。

    流程：降采样（见 _DETECT_WIDTH）→ BGR→HSV → 取 cfg["hsv"][cfg["color"]] 阈值分割
    → 形态学开/闭去噪与填洞 → 面积/凸包圆度/颜色复核/置信度过滤 → 取最高者
    → 坐标按降采样比例还原回原分辨率。
    需要同时识别多种颜色时用 detect_all_colors。

    参数:
        frame: BGR 图像。
        cfg:   颜色相关配置：color（颜色编号 1=红/2=绿/3=蓝）、hsv（以编号为键的各色区间）；
               可选 min_area_ratio / min_circularity / min_confidence（缺省不筛）。
    返回:
        [(u, v, r, confidence)]，r 为像素半径（已还原到 frame 原分辨率），
        confidence ∈ [0,1]；无匹配或全部低于阈值时为空列表。
    """
    global DEBUG_VIEW
    small, scale = _preprocess(frame)
    hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
    best, mask, contours = _detect_one(small, hsv, cfg, cfg["color"], scale)
    if SHOW_MASK_VIEW:                                   # 产出掩膜可视化，供 main.py 开关上屏
        DEBUG_VIEW = _debug_view(frame, mask, contours, best, scale)
    return [best] if best is not None else []


def detect_all_colors(frame, cfg):
    """对 cfg["hsv"] 中的每种颜色各取置信度最高的一个圆形色块。

    三种任务（balls/target）需要同时识别三种颜色并按出现顺序记点位，故一次降采样、
    一次 BGR→HSV，再逐色分割；每色最多返回一个候选（同色异物取置信度最高者）。
    返回 [(color, u, v, r, confidence)]，按颜色编号升序，坐标已还原到原帧分辨率；
    某颜色无候选时不出现在结果里。
    """
    global DEBUG_VIEW
    small, scale = _preprocess(frame)
    hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
    results, bests = [], []
    combined = None
    all_contours = []
    for color in sorted(cfg["hsv"].keys()):
        best, mask, contours = _detect_one(small, hsv, cfg, color, scale)
        combined = mask if combined is None else cv2.bitwise_or(combined, mask)
        all_contours.extend(contours)
        if best is not None:
            u, v, r, conf = best
            results.append((int(color), u, v, r, conf))
            bests.append(best)
    if SHOW_MASK_VIEW and combined is not None:
        DEBUG_VIEW = _debug_view(frame, combined, all_contours, bests, scale)
    return results
