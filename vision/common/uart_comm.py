"""任务切换：把「切换输入（UART）」与「任务激活（内参、相机、runner）」
封装成一个 TaskSwitcher，main 只负责构造它并调用 poll() / activate()。

主控发送 '1'/'2'/'3'/'4' → qr/balls/target/rescue，驱动自动推进。

激活时只在分辨率真的变化时才重建相机——无条件重建会打断 ISP/VO 管线，导致
disp.show 一直报 `mmf_vo_frame_push failed`、画面变黑。
"""
import time
import gc

from maix import camera, uart, pinmap, err

from common import config, geometry

_TASK_BY_CHAR = {ord("0"): "standby", ord("1"): "qr", ord("2"): "balls",
                 ord("3"): "target", ord("4"): "rescue", ord("5"): "align"}   # UART 指令字符 → 任务
_VALID_TASKS = ("standby", "qr", "balls", "target", "rescue")                 # 可激活的任务名
_CAM_WARMUP_FRAMES = 3              # 打开相机后先丢弃的帧数：ISP 未就绪时读会超时/读到坏帧


def open_command_uart(cfg_all):
    """打开接收切换指令的串口；失败返回 None（此时无法切换任务，只能运行启动任务）。"""
    port, baud = config.comm_uart(cfg_all)
    try:
        if port == "/dev/ttyS4":
            err.check_raise(pinmap.set_pin_function("A21", "UART4_TX"), "UART4 TX")
            err.check_raise(pinmap.set_pin_function("A22", "UART4_RX"), "UART4 RX")
        serial = uart.UART(port, baud)
        print(f"[comm V10] opened {port}@{baud}, polling read/write")
        return serial
    except Exception as e:
        print(f"[comm] 串口 {port}@{baud} 打开失败，UART 任务切换不可用: {e}")
        return None


RX_BYTES = 0
RX_ALIGN = 0

def poll_command(uart_obj):
    """非阻塞读串口，返回收到的最新任务名；无指令返回 None。

    指令协议：主控发送 ASCII '1'/'2'/'3'/'4' → qr/balls/target/rescue，其余字节忽略。
    """
    global RX_BYTES, RX_ALIGN
    if uart_obj is None:
        return None
    data = uart_obj.read()               # 非阻塞：无数据返回 b""
    if not data:
        return None
    RX_BYTES += len(data)
    if 0x35 in data:
        RX_ALIGN += data.count(0x35)
        if RX_ALIGN == 1:
            print("[comm V10] received 35 -> XY enabled")
        return "align"
    name = None
    for ch in data:                      # bytes 逐字节遍历得到 int
        name = _TASK_BY_CHAR.get(ch, name)
    return name


def open_camera(w, h, fps, attempts=3):
    """打开相机并确认能出帧；失败重试，仍失败返回 None。

    MaixPy FAQ#28：`RuntimeError: Value is NULL: camera read failed`（日志里的 camera read
    timeout）是「缓冲区里还没有新帧」，多因 open 后读得太快、ISP 还没开始出帧；官方解法是
    捕获异常 → 短暂 sleep → 重试。这里 open 后先丢几帧 warm-up，再确认能稳定读一帧，避免
    首帧超时直接把程序打崩。
    """
    for i in range(attempts):
        try:
            cam = camera.Camera(w, h, fps=fps)
        except Exception as e:
            print(f"[cam] 打开 {w}x{h} 失败：{e}（重试 {i + 1}/{attempts}）")
            time.sleep(0.3)
            continue
        for _ in range(_CAM_WARMUP_FRAMES):          # 丢弃开机不稳定帧（读失败也无妨）
            try:
                cam.read()
            except Exception:
                pass
        try:
            frame = cam.read()
            if frame is None or frame.width() != w or frame.height() != h:
                raise RuntimeError("invalid camera warmup frame")
            frame = None
            return cam
        except Exception as e:
            print(f"[cam] {w}x{h} 读取超时：{e}（重试 {i + 1}/{attempts}）")
            try:
                cam.close()
            except Exception:
                pass
            time.sleep(0.3)
    return None


class TaskSwitcher:
    """任务切换器：封装切换输入与任务激活，向 main 暴露 task / cam / runner 三个属性。

    参数:
        cfg_all:        完整配置（config.load_config()）。
        default:        启动任务名（qr / balls / target / rescue）。
        runner_factory: 形如 f(name, intr, send) -> {"handle", "fmt", "draw"} 的工厂，由 main 提供；
                        send(payload) 为一整行上行数据发送（发给主控）。
    """

    def __init__(self, cfg_all, default, runner_factory):
        self.cfg = cfg_all
        self._runner_factory = runner_factory
        self.task = None
        self.runner = None
        self.cam = None
        self.intr = None
        self._intr_base = None                       # 标定分辨率下的原始内参（只读一次）
        self._cam_size = None
        self._uart = open_command_uart(cfg_all)      # 下行接收（主控）+ 上行发送
        self.activate(default)

    def poll(self):
        """非阻塞轮询串口指令，返回请求切换到的任务名；无请求返回 None。"""
        return poll_command(self._uart)

    def send(self, payload):
        """上行发送一个或多个字节给主控（与下行同一串口）；串口不可用返回 False。

        payload 为 bytes 或 str（str 会编码为 ASCII）；v1 协议每条消息只有 1 字节，
        故调用方传 bytes([字节]) 即可。发送失败只打印提示，不抛异常。
        """
        if self._uart is None:
            print("[comm] 串口未打开，上行指令无法发送")
            return False
        try:
            self._uart.write(payload.encode() if isinstance(payload, str) else payload)
            return True
        except Exception as e:
            print(f"[comm] 上行发送失败: {e}")
            return False

    def activate(self, name):
        """Prepare the runner first, then release and reopen only if size changes."""
        if name != "wait" and name not in _VALID_TASKS:
            raise ValueError(f"unknown task: {name}")
        base = self._intr_base
        if name not in ("wait", "standby", "qr", "rescue", "balls") and base is None:
            base = geometry.load_intrinsics(config.intrinsics_path(self.cfg))
        w, h = config.camera_size_for(self.cfg, name)
        intr = self.intr
        if base is not None and name not in ("wait", "standby", "qr", "rescue", "balls"):
            intr = geometry.scale_intrinsics(base, w, h)
        # Invalid calibration/runner configuration must not destroy the live camera.
        runner = self._runner_factory(name, intr, self.send)
        if self.cam is None:
            self.cam = open_camera(w, h, config.camera_fps(self.cfg))
            if self.cam is None:
                raise RuntimeError(f"camera initial open failed: {w}x{h}")
            self._cam_size = (w, h)
        elif (w, h) != self._cam_size:
            previous_size = self._cam_size
            print(f"[cam] resize {previous_size} -> {(w, h)}")
            # Keep the camera/display pipeline object alive instead of close/recreate.
            try:
                self.cam.set_resolution(w, h)
                self._verify_size(w, h)
            except Exception:
                # A failed command must not leave the active task using wrong dimensions.
                try:
                    self.cam.set_resolution(*previous_size)
                    self._verify_size(*previous_size)
                except Exception as restore_error:
                    raise RuntimeError(f"camera resize and restore failed: {restore_error}")
                raise
            self._cam_size = (w, h)
        self._intr_base = base
        self.intr = intr
        self.runner = runner
        self.task = name
        print(f"[task] -> {name}  {w}x{h}")

    def _verify_size(self, w, h):
        # Validate real frames, not just a successful API call.
        for _ in range(8):
            frame = None
            try:
                frame = self.cam.read()
                if frame is not None and frame.width() == w and frame.height() == h:
                    frame = None
                    return
            except Exception:
                pass
            finally:
                frame = None
            time.sleep(0.05)
        raise RuntimeError(f"no valid {w}x{h} frame after resize")

    def accept_command(self, name):
        # A delayed/repeated QR request must not bounce back from low resolution.
        order = {"wait": 0, "qr": 1, "standby": 2, "balls": 3, "target": 4, "rescue": 5}
        if name is None:
            return False
        if order.get(name, -1) < order.get(self.task, 0):
            print(f"[task] ignored stale {name} command in {self.task}")
            return False
        return True
