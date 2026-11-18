# SLOT BUCKET V3

This version requires updated chassis firmware. The 15-byte AA55 XY packet byte 3 (zero-based) now contains (slot << 4) | wire_color. Slot is 1..3, wire_color is 1 red / 2 blue / 3 green. CRC covers this packed byte. Target-slot assignment occurs after three central-gate observations; then STOP is sent. XY streaming starts only after chassis command 0x35. This section supersedes older descriptions below.

# XY FAST VERSION

Balls mode: target-color XY only. ASCII 5 (0x35) from chassis enables XY streaming after wheels stop. Before that, only target-found STOP events are sent. Every processed frame sends the existing 15-byte AA55 packet (valid, wire color, x, y, width, height, sequence, CRC8); no Z/3D/confidence/text is transmitted. Invalid detections send valid=0. This section supersedes older balls descriptions below.

# 当前抓球协议更新

请以同目录 BALL_CHAIN_NOTES.md 为准：颜色线上编号1红2蓝3绿，发现事件0x51/0x52/0x53；恢复完整抓取/回桶/松爪流程。下文旧说明仅作历史参考。

# 2026-11-04 抓球流程协议（优先于下方旧说明）

下行 ASCII：`0`=识球分辨率预览、不识别；`1`=扫码；`2`=重建球检测状态并开始识别，成功后相机回 `0x42`。
上电相机 wait 状态只预览；底盘到第一个路口后发 `1`。二维码前三位必须均为 1/2/3 才回 `0x30`。扫码结束底盘发 `0`，第五拐角伸臂等待后发 `2`。

识别到二维码指定颜色的球，先发 `0x43` 请求停车，再发坐标。约 20Hz 连续发送，丢失时立刻发 valid=0，不能把旧坐标当成当前坐标。
坐标包共15字节：AA 55 valid color xLo xHi yLo yHi widthLo widthHi heightLo heightHi seqLo seqHi CRC。
CRC8(poly=0x07, init=0)，覆盖 valid 到 seqHi（索引2..13）。所有 uint16 均为小端。
XY 为当前图像的像素坐标，原点左上，x向右，y向下；不是场地坐标。seq每包递增，16位回绕。
包内字节不得作为旧的单字节事件解释。底盘UART中断仅解析、校验和保存，Route_task原子读取快照。

底盘停车后仅用x横移居中（默认最大0.05m/s、中心容差半画宽的0.04、5个新帧且停稳），y只保存。
数据超过250ms或valid=0时停车等待；15秒未对准或横移离起始点超过0.25m时停止本次跑图，不自动抓取。
BALL_ALIGN_Y_SIGN默认-1：假设画面右侧对应车右侧，底盘+Y为左。相机安装若反向须实测修改。
机械臂动作在HARDWARE/arm_task.c的ArmMission_Apply内直接改Arm_SetAngle参数：EXTEND、APPROACH、PICK、BUCKET、BUCKET_FINISH。抓爪为关节2；角度是绝对目标，不是增量。当前角度不保证实物抓取成功，需要用户标定。

---
以下为原协议说明；抓球部分以以上新版为准。

# 视觉相机 ↔ 主控（底盘）串口通信协议 v1.0

适用：MaixCam 视觉端（本仓库 `main.py`）与底盘主控之间的 UART 通信。
实现位置：`main.py` 顶部 `EV_*` 常量、`common/uart_comm.py` 的 `poll_command` / `send`、
`config.yaml` 的 `comm` 段。三者必须与本文件保持一致。

## 1. 物理层

| 项目 | 约定 |
| --- | --- |
| 接口 | UART TTL 3.3V，共地 |
| 参数 | 115200 bps，8 数据位，无校验，1 停止位（8N1），无流控 |
| 端口 | 相机侧 `/dev/ttyS0`（可在 `config.yaml` 的 `comm.uart_port` 修改） |
| 方向 | 同一条串口双向：下行=主控→相机，上行=相机→主控 |

## 2. 链路层（通用规则）

- **每条消息固定 1 字节**，无帧头、无长度、无校验、无换行符，收到即完整消息。
- 字节按位域解释：**高 4 位 = 事件类型，低 4 位 = 参数**。
- 只有 1 字节，天然按字节流对齐，接收端逐个字节处理，不存在粘包/半包。
- 事件类型 `0x4`～`0xE` 预留；`0xF` 预留给错误码。参数 `0x0` 表示该事件无参数。
- 新增事件时只增加高 4 位的类型码，不改变已有字节含义，保持向下兼容。

## 3. 下行指令（主控 → 相机）

单字节 ASCII，收到即切换任务；其余字节忽略。

| 字节 | HEX | 含义 |
| --- | --- | --- |
| `'1'` | 0x31 | 切换到任务1：二维码 |
| `'2'` | 0x32 | 切换到任务2：三色球 |
| `'3'` | 0x33 | 切换到任务3：反恐靶 |
| `'4'` | 0x34 | 切换到任务4：救援 |

切换后相机重建任务运行器：点位从 0 重新计数、停车锁存解除、从 `0x0T` 心跳开始上报。
重复发送当前任务编号等于复位该任务。

## 4. 上行事件（相机 → 主控）

| 字节 | 事件 | 参数 | 含义 | 发送时机 |
| --- | --- | --- | --- | --- |
| `0x01` | IDLE | 任务号 1 | 二维码任务空闲，无新事件 | 每 `data_every` 帧 |
| `0x02` | IDLE | 任务号 2 | 三色球任务空闲，无新事件 | 每 `data_every` 帧 |
| `0x03` | IDLE | 任务号 3 | 反恐靶任务空闲，无新事件 | 每 `data_every` 帧 |
| `0x04` | IDLE | 任务号 4 | 救援任务空闲，无新事件 | 每 `data_every` 帧 |
| `0x11` | POINT | 点位 1 | 第 1 个身份首次出现，已记为点位 1 | 分配当帧立即发一次 |
| `0x12` | POINT | 点位 2 | 第 2 个身份首次出现，已记为点位 2 | 分配当帧立即发一次 |
| `0x13` | POINT | 点位 3 | 第 3 个身份首次出现，已记为点位 3 | 分配当帧立即发一次 |
| `0x21` | STOP | 点位 1 | 目标居中，请求停车，目标点位 1 | 居中当帧立即发；锁存后每 `data_every` 帧重发 |
| `0x22` | STOP | 点位 2 | 目标居中，请求停车，目标点位 2 | 同上 |
| `0x23` | STOP | 点位 3 | 目标居中，请求停车，目标点位 3 | 同上 |
| `0x30` | QR_OK | 0 | 二维码三位任务码解析完成 | 解出期间每 `data_every` 帧 |
| `0xF0`～`0xFF` | ERROR | 错误码 | 预留 | — |

`data_every` 默认 3 帧（约 10 Hz，30 fps 时），见 `config.yaml` 的 `comm.data_every`。

## 5. 语义约定

### 5.1 点位 position

- 点位是**沿行进方向第几个被相机首次看到的不同身份**，不是颜色编号。
- 身份：balls/target 为颜色（1 红 / 2 绿 / 3 蓝），rescue 为形状（1 圆柱 / 2 圆锥 / 3 腰鼓）。
- 同一种身份只计一次；同一帧出现多个新身份时，按「像素尺寸大 = 离相机近」由近到远依次编号。
- 因此 `0x11` 表示「第 1 个目标」，具体是什么颜色/形状由二维码任务码与赛道摆放决定。

### 5.2 停车

- balls / rescue 在全画面检测；target 采 320×240，中央 ROI（`main.py` 顶部 `ROI_SIZE`）裁剪后
  放大到 `target.zoom_width` 再检测。
- 目标身份的中心进入画面中央容差框即触发 `0x2N`：target 用 `target.center_tol`（相对 ROI），
  其余任务用 `locate.center_tol`（相对整帧，默认 10% 宽高）。
- `0x2N` 是**电平语义**：锁存后每 `data_every` 帧重复发送同一个字节，主控按「最近一次收到」执行即可，重复收到视为同一请求（幂等）。
- 停车锁存不会因目标移出画面而解除，只在收到新的下行任务编号时解除。

### 5.3 可靠性

- 单字节无校验，允许丢字节；IDLE 心跳每 `data_every` 帧一次，STOP 持续重发，主控可在数个周期内自行恢复状态。
- 主控收到未定义字节应忽略，不要复位状态机。

## 6. 典型时序（balls 任务，`data_every=3`）

```
主控 → 相机  0x32                     切换三色球任务
相机 → 主控  0x02                     心跳：空闲
相机 → 主控  0x02                     心跳：空闲
相机 → 主控  0x11                     首次看到颜色A → 点位1
相机 → 主控  0x02                     心跳
相机 → 主控  0x12                     首次看到颜色B → 点位2
相机 → 主控  0x02                     心跳
相机 → 主控  0x02                     心跳
相机 → 主控  0x13                     首次看到颜色C → 点位3
相机 → 主控  0x02                     心跳
相机 → 主控  0x22                     目标颜色B进入画面中央 → 请求停车，点位2
相机 → 主控  0x22                     锁存重发
相机 → 主控  0x22                     锁存重发
主控 → 相机  0x34                     切换救援任务（解除锁存）
```

## 7. 不在协议内传输的数据

像素坐标 `u/v`、距离 `Z`、反投影 `X/Y`、瞄准角 `yaw/pitch`、置信度等仅用于相机本地
终端打印与调试，不上传。若后续机械臂需要这些数值，按第 2 节扩展规则新增多字节事件
（例如 `0xE0` 后跟固定长度载荷），不修改 v1 已有字节的含义。
