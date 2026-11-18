"""设备端主程序（跑在 MaixCam 上）：按任务调度 + 实时上屏显示。

读参数 → 建显示 → 循环取帧交给当前任务 → 叠加标注 → 上屏 → 打印结果。
任务的启动与切换整体交给 common.uart_comm.TaskSwitcher，本文件只构造它、定义 runner 工厂并跑主循环。

四个任务：
    qr     —— 二维码解码：解出三位数字（排爆物颜色 / 反恐靶颜色 / 救援形状码）后只上报是否检测到；
    balls  —— 目标单色球心XY，发现即停车，停车确认后逐帧发送坐标；
    target —— 靶子距离与圆心：中央 ROI 裁剪放大后同时检测三种颜色，按出现顺序记点位，
              目标颜色居中时请求停车；
    rescue —— 白色人质形状识别：全画面同时检测三种形状，按出现顺序记点位，目标形状居中时请求停车。
点位与停车逻辑统一由 common.position.PositionTracker 维护。
串口为单字节协议：下行任务指令，上行事件字节（高 4 位事件、低 4 位参数），详见 PROTOCOL.md。

重要：本文件依赖 maix 模块，只能在 MaixCam 上运行。
"""
import time
import gc

from maix import app, display, image, gpio, pinmap, sys, err
from common import color, config, roi, uart_comm
from common.position import PositionTracker
from common.ball_order import BallOrder
from common.landmark_order import LandmarkOrder
from tasks import balls as balls_task
from tasks import qr_maixcam
from tasks import rescue as rescue_task
from tasks import target as target_task

TEST_BALL_COLOR = 0  # No QR test: 1=red, 2=blue, 3=green; 0=disable fallback.
TASK = "qr"                      # 启动默认任务：qr / balls / target / rescue
PRINT_EVERY = 60                 # 每 N 帧打印一次结果；1 表示逐帧打印target（终端 I/O 会拖慢帧率）
SHOW_MASK_VIEW = False            # 调试开关：上屏显示检测掩膜；balls、target、rescue 有效
DEBUG_DETECT = False             # 调试开关：逐帧打印检测细节；量产/比赛必须为 False

# ---- 中央 ROI 大小（仅 target 使用）----------------------------------------
ROI_WIDTH_RATIO = 0.70          # 中央ROI宽度占画面宽度的50%，增大后左右范围变宽。
ROI_HEIGHT_RATIO = 0.50         # 中央ROI高度占画面高度的20%，减小后矩形更扁。

# ---- 上行单字节事件（与 PROTOCOL.md、config.yaml 的 comm 段一致）-------------
# 每条上行消息固定 1 字节：高 4 位=事件类型，低 4 位=参数（点位 / 任务号）。
EV_IDLE = 0x00      # 0x0T：T=任务号（qr=1 balls=2 target=3 rescue=4）
EV_POINT = 0x10     # 0x1N：新身份首次出现，已记为点位 N（1..3）
EV_STOP = 0x20      # 0x2N：目标已居中，请求停车，目标点位 N（1..3）
EV_QR = 0x30        # 0x30：二维码三位任务码解析完成
TASK_ID = {"qr": 1, "balls": 2, "target": 3, "rescue": 4}


def _codes_from(results):
    """取首个有效 payload 的三位数字 → (排爆物颜色, 反恐靶颜色, 救援形状码)。

    赛题约定：第 1 位排爆物颜色、第 2 位反恐靶颜色、第 3 位救援目标形状，均为 1/2/3。
    没有足够位数的有效结果时返回 (None, None, None)。
    """
    for r in results:
        digits = [int(ch) for ch in r.get("payload", "") if ch.isdigit()]
        if len(digits) >= 3 and all(d in (1, 2, 3) for d in digits[:3]):
            return digits[0], digits[1], digits[2]
    return None, None, None


def chassis_ball_color(official_color):
    """QR/detector: 1 red,2 green,3 blue -> wire: 1 red,2 blue,3 green."""
    return {1: 1, 2: 3, 3: 2}[official_color]


def crc8_payload(data):
    """CRC-8/poly 0x07 over packet payload, excluding the header bytes."""
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ (0x07 if crc & 0x80 else 0)) & 255
    return crc


def landmark_packet(kind, slot, x, width):
    """8字节：帧头、任务类型、经过位次、中心X、整图宽度、CRC；不传Y。"""
    x = 65535 if x is None else x
    data = bytearray([0xAA, kind, slot, x & 255, x >> 8, width & 255, width >> 8])
    data.append(crc8_payload(data[2:]))
    return bytes(data)


def ball_packet(valid, color_id, x, y, width, height, sequence, slot=0):
    """AA 55, valid/color, five little-endian uint16 fields, CRC8(poly 07)."""
    data = bytearray([0xAA, 0x55, int(valid), (slot << 4) | color_id])
    for value in (x, y, width, height, sequence):
        data.extend((value & 255, (value >> 8) & 255))
    data.append(crc8_payload(data[2:]))
    return bytes(data)


def fmt_qr(results):
    """任务1：紧凑单行，空格分隔各二维码内容；无结果输出 '-'。"""
    return " ".join(r["payload"] for r in results) if results else "-"


def fmt_balls(result):
    return "x=%d y=%d" % result if result is not None else "lost"


def fmt_target(results, tracker):
    """任务3：逐色 'color u v r Z yaw pitch conf pos'；无结果输出 '-'。"""
    if not results:
        return "-"
    parts = [f"{r.color} u={r.u:.0f} v={r.v:.0f} r={r.r:.0f} Z={r.Z:.0f} "
             f"yaw={r.yaw:.1f} pitch={r.pitch:.1f} conf={r.conf:.2f} "
             f"pos={tracker.position_of(r.color)}"
             for r in results]
    tag = f" | stop@{tracker.stop_position}" if tracker.stopped else ""
    return " | ".join(parts) + tag


def fmt_rescue(results, tracker):
    """任务4：逐件 'shape(code) pos'；无目标输出 '-'。"""
    if not results:
        return "-"
    parts = [f"{o.shape}({o.code}) pos={tracker.position_of(o.code)}" for o in results]
    tag = f" | stop@{tracker.stop_position}" if tracker.stopped else ""
    return " | ".join(parts) + tag


def draw_qr(img, results):
    """任务1 标注：四顶点画框（不绘制文字，内容只走终端打印）。"""
    return qr_maixcam.draw_results(img, results)


def draw_balls(img, result):
    if result is not None:
        img.draw_circle(result[0], result[1], 3, image.COLOR_RED, thickness=1)
    img.draw_circle(img.width()//2, img.height()//2, 3, image.COLOR_GREEN, thickness=1)
    return img


def draw_target(img, results, target_color, view):
    """任务3 标注：目标色红圈，其余颜色绿圈；ROI 坐标先还原到原帧。"""
    ox, oy, s = view["ox"], view["oy"], view["scale"]
    for res in results:
        color = image.COLOR_RED if res.color == target_color else image.COLOR_GREEN
        u = ox + res.u / s
        v = oy + res.v / s
        img.draw_circle(int(u), int(v), int(res.r / s), color, thickness=2)
    return img


def draw_rescue(img, results, target_code):
    """任务4 标注：目标形状红框、其余候选绿框（用四条 draw_line 画矩形）。"""
    for o in results:
        color = image.COLOR_RED if o.code == target_code else image.COLOR_GREEN
        x0, y0 = int(o.u - o.w / 2.0), int(o.v - o.h / 2.0)
        x1, y1 = int(o.u + o.w / 2.0), int(o.v + o.h / 2.0)
        img.draw_line(x0, y0, x1, y0, color, thickness=2)
        img.draw_line(x1, y0, x1, y1, color, thickness=2)
        img.draw_line(x1, y1, x0, y1, color, thickness=2)
        img.draw_line(x0, y1, x0, y0, color, thickness=2)
    return img


def draw_roi_box(img, box):
    """按归一化边框 box=[x0,y0,x1,y1] 在原帧上画出 target 的检测 ROI（只画框）。"""
    w, h = img.width(), img.height()
    x0, y0, x1, y1 = box
    x0, y0 = int(x0 * w), int(y0 * h)
    x1, y1 = int(x1 * w), int(y1 * h)
    img.draw_line(x0, y0, x1, y0, image.COLOR_GREEN, thickness=3)
    img.draw_line(x1, y0, x1, y1, image.COLOR_GREEN, thickness=3)
    img.draw_line(x1, y1, x0, y1, image.COLOR_GREEN, thickness=3)
    img.draw_line(x0, y1, x0, y0, image.COLOR_GREEN, thickness=3)
    return img

def turn_off_fill_light():
    """熄灭板载补光灯（低电平）。失败只告警不抛异常。"""
    try:
        _fill_light_gpio().value(0)
        return True
    except Exception as e:
        print(f"[light] fill light off failed: {type(e).__name__}: {e}")
        return False
def main():
    """读参数、建显示 → 构造 TaskSwitcher → 主循环：取帧/处理/标注/上屏/打印。"""
    print("[BUILD] TARGET_ROI_RECT_V3")
    turn_off_fill_light()
    cfg_all = config.load_config()
    disp = display.Display()                        # 屏幕输出，实际显示分辨率由板子决定
    color.configure(cfg_all.get("detect"), debug=DEBUG_DETECT, show_mask_view=SHOW_MASK_VIEW)
    rescue_task.configure(show_mask_view=SHOW_MASK_VIEW)
    every = config.comm_data_every(cfg_all)         # 上行行帧周期：每 every 帧发一次
    # qr 解出的三位任务码，跨任务持久：balls 用排爆物颜色、target 用反恐靶颜色、rescue 用形状码
   
   # state = {"bomb_color": 1, "target_color": 2, "shape_code": 1}

    state = {"bomb_color": None, "target_color": None, "shape_code": None}
    locate_cfg = cfg_all.get("locate", {})
    center_tol = float(locate_cfg.get("center_tol", 0.10))
    # 宽高独立设置，检测范围、停车计数范围和显示框使用同一个中央矩形。
    edge_x = (1.0 - ROI_WIDTH_RATIO) / 2.0
    edge_y = (1.0 - ROI_HEIGHT_RATIO) / 2.0
    roi_box = [edge_x, edge_y, 1.0 - edge_x, 1.0 - edge_y]

    def make_counter():
        """每调用一次前进一帧，到第 every 帧返回 True。"""
        n = 0

        def tick():
            nonlocal n
            n += 1
            return n % every == 0

        return tick

    def make_runner(name, intr, send):
        """按任务生成 {handle, fmt, draw}：把 cfg / intr / send 与任务私有状态固化进闭包。

        send(payload) 由 TaskSwitcher 提供，发送一个或多个字节给主控。
        每任务独立持有一个 PositionTracker：切换任务重建 runner 时点位与停车锁存自动清零。
        上行规则：新点位 (0x1N) 与停车 (0x2N) 在触发当帧立即发一次；
        之后每 data_every 帧发一次心跳，已锁存则持续重发停车字节。
        """
        tick = make_counter()
        if name in ("wait", "standby"):
            return {"handle": lambda f: None,
                    "fmt": lambda r: "waiting for chassis search command 0x36",
                    "draw": lambda f, r: None}
        tid = TASK_ID[name]
        if name == "qr":
            confirmed = False
            last_tx = None
            attempts = 0

            def handle(f):
                nonlocal confirmed, last_tx, attempts
                results = qr_maixcam.decode(f)
                bomb, tgt, shape = _codes_from(results)
                if bomb is not None and not confirmed:
                    state.update(bomb_color=bomb, target_color=tgt, shape_code=shape)
                    confirmed = True
                    print(f"[qr] accepted {bomb},{tgt},{shape}; waiting for chassis reply")
                now = time.monotonic()
                if confirmed and (last_tx is None or now-last_tx >= 0.2):
                    ok = send(bytes([EV_QR]))
                    attempts += 1
                    if attempts == 1 or attempts % 10 == 0:
                        print(f"[qr] QR_OK 0x30 attempt={attempts} uart_write={ok}")
                    last_tx = now
                elif not confirmed and tick():
                    send(bytes([EV_IDLE | tid]))
                return results

            return {"handle": handle, "fmt": fmt_qr, "draw": draw_qr}

        cfg = config.task_section(cfg_all, name)
        if name == "target":
            cfg = dict(cfg, detect_roi=roi_box)
        if name == "balls":
            if state["bomb_color"] not in (1, 2, 3):
                raise RuntimeError("No saved QR target color; scan QR before balls")
            print("[balls] saved QR target=%d (1 red,2 green,3 blue)" % state["bomb_color"])
            detector = balls_task.XYDetector(cfg, state["bomb_color"])
            order = BallOrder(cfg, state["bomb_color"])
            sequence = 0
            stream_xy = False
            xy_sent = 0

            def chassis_stopped():
                nonlocal stream_xy
                if not stream_xy:
                    print("[xy] received 0x35; streaming XY now")
                stream_xy = True

            def handle(f):
                nonlocal sequence, xy_sent
                frame = image.image2cv(f)
                if order.target_slot:
                    point = detector.detect(frame)
                else:
                    points = detector.detect_colors(frame)
                    previous_count = len(order.slots)
                    order.update(points, f.width(), f.height())
                    if len(order.slots) != previous_count:
                        print("[count] colors->slots=%s target_slot=%d" % (order.slots, order.target_slot))
                    point = points.get(state["bomb_color"])
                target_color = state["bomb_color"]
                if target_color not in (1, 2, 3):
                    return None
                # One NEW result per processed frame. No timer cap, no stale retransmits.
                if stream_xy and point is not None:
                    x, y = point
                    if send(ball_packet(point is not None, chassis_ball_color(target_color),
                                     x, y, f.width(), f.height(), sequence, order.target_slot)):
                        xy_sent += 1
                    sequence = (sequence+1) & 0xffff
                elif not stream_xy and point is not None and order.target_slot:
                    send(bytes([0x50 | chassis_ball_color(target_color)]))
                return point

            return {"handle": handle, "chassis_stopped": chassis_stopped,
                    "fmt": lambda point: "slot=%d stream=%d tx=%d rx=%d rx35=%d %s" % (order.target_slot, stream_xy, xy_sent, uart_comm.RX_BYTES, uart_comm.RX_ALIGN, fmt_balls(point)), "draw": draw_balls}

        # 靶子按颜色、救援按形状区分身份；位次按画面从右向左的经过顺序计数。
        is_target = name == "target"
        if is_target:
            print("[target ROI V3] box=%s; count/stop only inside ROI" % roi_box)
        selected = state["target_color"] if is_target else state["shape_code"]
        if selected not in (1, 2, 3):
            raise RuntimeError("二维码缺少打靶颜色或救援形状，请使用三位任务码")
        order = LandmarkOrder(selected, int(cfg.get("count_direction", -1)))
        stream_x = False
        packet_kind = 0x57 if is_target else 0x58
        stop_event = 0x54 if is_target else 0x64

        def chassis_stopped():
            nonlocal stream_x
            stream_x = True

        def handle(f):
            frame = image.image2cv(f)
            results = target_task.detect(frame, cfg, intr) if is_target else rescue_task.detect(frame, cfg)
            best = {}
            for obj in results:
                # 只有靶心位于框内才参与计数和触发停车；不把框外颜色记入位次。
                if is_target and not (roi_box[0]*f.width() <= obj.u < roi_box[2]*f.width()
                                      and roi_box[1]*f.height() <= obj.v < roi_box[3]*f.height()):
                    continue
                ident = obj.color if is_target else obj.code
                if ident not in best or obj.conf > best[ident].conf:
                    best[ident] = obj
            order.update({ident: (obj.u, obj.v) for ident, obj in best.items()})
            chosen = best.get(selected)
            x = None if chosen is None else max(0, min(f.width()-1, round(chosen.u)))
            if stream_x and order.target_slot:
                # 停车通知后逐帧回传整图X及位次，未检测到目标时发送无效坐标。
                send(landmark_packet(packet_kind, order.target_slot, x, f.width()))
            elif x is not None and order.target_slot:
                # 目标进入检测区域便停车，随后由底盘按中心X进行对中。
                send(bytes([stop_event]))
            return None if chosen is None else (x, round(chosen.v))

        def draw_landmark(img, result):
            if is_target:
                draw_roi_box(img, roi_box)
            return draw_balls(img, result)

        return {"handle": handle, "chassis_stopped": chassis_stopped,
                "fmt": lambda p: "id=%d slot=%d stream=%d center=%s" % (selected, order.target_slot, stream_x, p),
                "draw": draw_landmark}



    # 切换逻辑（UART 输入、内参加载、按需重建相机、上行发送）整体封装在 TaskSwitcher
    switcher = uart_comm.TaskSwitcher(cfg_all, TASK, make_runner)

    n = 0
    t_last = time.monotonic()                       # 帧率统计窗口起点
    img = res = view = None
    pending_task = None
    switch_attempts = 0
    retry_at = 0.0
    preview_at = 0.0
    preview_hz = max(0.0, float(cfg_all.get("balls", {}).get("preview_hz", 5)))
    while not app.need_exit():
        new = switcher.poll()
        if new not in (None, "align", "balls", "qr", "standby", "target", "rescue"):
            new = None
        if new == "align":
            if switcher.task in ("balls", "target", "rescue") and pending_task is None:
                switcher.runner["chassis_stopped"]()
            new = None
        if switcher.accept_command(new):
            if new == switcher.task and pending_task is None:
                # Acknowledge retries, but never reset a running ball tracker.
                if new == "balls":
                    switcher.send(bytes([0x42]))
            elif new != pending_task:
                pending_task = new
                switch_attempts = 0
                retry_at = 0.0
        if pending_task is not None and switch_attempts < 3 and time.monotonic() >= retry_at:
            img = res = view = None
            if switcher.task == "qr" and pending_task != "qr":
                qr_maixcam.release()
            gc.collect()
            switch_attempts += 1
            try:
                switcher.activate(pending_task)
                if pending_task == "balls":
                    switcher.send(bytes([0x42]))
                pending_task = None
                n, t_last = 0, time.monotonic()
            except Exception as e:
                print(f"[task] switch attempt {switch_attempts}/3 failed: {type(e).__name__}: {e}")
                retry_at = time.monotonic() + 1.0
                if switch_attempts == 3:
                    print("[task] retries stopped; restart after checking the first error above")
        if pending_task is not None:
            # Preserve the selected QR result without continuing the old task/accelerator.
            time.sleep(0.02)
            continue
        t0 = time.monotonic()
        try:
            img = switcher.cam.read()
        except Exception as e:                      # 偶发无新帧，跳本帧重试而非崩溃
            print(f"[cam] read 超时：{e}，跳过本帧")
            time.sleep(0.02)
            continue
        t1 = time.monotonic()
        try:
            res = switcher.runner["handle"](img)
        except Exception as e:
            print(f"[vision:{switcher.task}] {type(e).__name__}: {e}")
            try:
                if switcher.task == "target":
                    draw_roi_box(img, roi_box)
                disp.show(img, fit=image.Fit.FIT_CONTAIN)
            except Exception as display_error:
                print(f"[display] {display_error}")
            time.sleep(0.1)
            continue
        t2 = time.monotonic()
        n += 1
        if n == 1:                                  # 实际出帧尺寸：确认请求的分辨率真的生效
            print(f"[cam] 实际出帧 {img.width()}x{img.height()}")
        view = None
        if switcher.task in ("balls", "target"):    # 各任务只刷新自己的 DEBUG_VIEW
            view = color.DEBUG_VIEW
        elif switcher.task == "rescue":
            view = rescue_task.DEBUG_VIEW
        show_due = switcher.task != "balls" or (preview_hz > 0 and time.monotonic() >= preview_at)
        if show_due:
            try:
                if switcher.task == "target":
                    draw_balls(img, res)
                    draw_roi_box(img, roi_box)
                    disp.show(img, fit=image.Fit.FIT_CONTAIN)
                elif SHOW_MASK_VIEW and view is not None:     # 按任务限定，避免上屏残留掩膜
                    disp.show(image.cv2image(view), fit=image.Fit.FIT_CONTAIN)
                else:
                    switcher.runner["draw"](img, res)       # 在原始 MaixPy 帧上叠加标注
                    disp.show(img, fit=image.Fit.FIT_CONTAIN)   # 等比缩放上屏，避免拉伸变形
            except Exception as e:
                print(f"[display:{switcher.task}] {type(e).__name__}: {e}")
                time.sleep(0.1)
                continue
            preview_at = time.monotonic() + (1.0/preview_hz if preview_hz > 0 else 0)
        t3 = time.monotonic()
        if n % PRINT_EVERY == 0:                    # 节流打印：逐帧输出是阻塞 I/O
            now = time.monotonic()
            fps = PRINT_EVERY / (now - t_last)      # 本窗口平均帧率（含取帧/检测/上屏全链路）
            t_last = now
            # 分段耗时定位瓶颈：read=取帧 / handle=转换+检测 / show=标注+上屏。
            # 只在每 PRINT_EVERY 帧的汇总行追加，开销可忽略，无需开 DEBUG_DETECT。
            detail = (f"  read={(t1 - t0) * 1000:.0f}ms"
                      f" handle={(t2 - t1) * 1000:.0f}ms"
                      f" show={(t3 - t2) * 1000:.0f}ms")
            print(f"[{switcher.task}] {switcher.runner['fmt'](res)}  fps={fps:.1f}{detail}")


if __name__ == "__main__":
    main()
