"""统一参数加载：main.py 与各任务共用，单一数据源为 config.yaml。"""
import yaml

CONFIG_PATH = "config.yaml"      # 默认配置路径（相对工程根目录）


def load_config(path=CONFIG_PATH):
    """读取 YAML 配置并返回整份字典；文件缺失或解析失败直接抛异常。"""
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def task_section(cfg_all, task):
    """取指定任务（balls / target / rescue）的配置段。"""
    return cfg_all[task]


def intrinsics_path(cfg_all):
    """内参文件路径（calib.intrinsics_path）。"""
    return cfg_all["calib"]["intrinsics_path"]


def camera_fps(cfg_all):
    """采集帧率（缺省 30）。"""
    return cfg_all["camera"].get("fps", 30)


def camera_size_for(cfg_all, task):
    """按任务取采集分辨率：qr 用 qr_width/qr_height，其余用 width/height。"""
    cam = cfg_all["camera"]
    if task in ("wait", "qr"):
        return cam["qr_width"], cam["qr_height"]
    return cam["width"], cam["height"]


def comm_uart(cfg_all):
    """任务切换与上行串口 (port, baudrate)。"""
    comm = cfg_all.get("comm", {})
    return comm.get("uart_port", "/dev/ttyS4"), comm.get("baudrate", 115200)


def comm_data_every(cfg_all):
    """上行行帧发送周期：每 N 帧一次（缺省 3）。"""
    return max(1, int(cfg_all.get("comm", {}).get("data_every", 3)))
