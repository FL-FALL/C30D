"""顺序点位统计：balls / target / rescue 共用的「边找边记点位」流程。

规则（与赛题摆放顺序抽签一致）：
  1. 每种身份（颜色编号 1/2/3 或形状码 1/2/3）只记一次点位，初始 position=0；
  2. 当前帧首次出现的身份按「由近到远」依次占用 1、2、3 号点位
     （weight 用像素半径或外接框面积，越大越近）；
  3. 目标身份的中心进入画面中央容差框时，锁存停车标志并返回其点位；
     锁存后不再接受新身份、不再改变点位，主程序据此持续发送停车信号。

不依赖内参：只做像素级的顺序与居中判断。
"""


class PositionTracker:
    """维护「身份 → 点位」映射与目标居中停车锁存。

    参数:
        center_tol: 居中容差，占画面宽/高的比例；|u-cx| ≤ tol·W 且 |v-cy| ≤ tol·H 视为居中。
    """

    def __init__(self, center_tol=0.5):
        self.center_tol = float(center_tol)
        self._positions = {}      # 身份 → 点位（1 起）
        self._next = 1            # 下一个待分配的点位
        self.stopped = False      # 目标已居中并锁存停车
        self.stop_position = 0    # 锁存时的目标点位

    def reset(self):
        """清空点位与锁存状态（切换任务时由新 runner 重建，通常无需显式调用）。"""
        self._positions.clear()
        self._next = 1
        self.stopped = False
        self.stop_position = 0

    def position_of(self, ident):
        """查询某身份已分配的点位；尚未出现返回 0。"""
        return self._positions.get(ident, 0)

    def update(self, items, target_id, width, height):
        """用当前帧检测结果更新点位与停车锁存。

        参数:
            items:     [(ident, u, v, weight)]；ident 为颜色编号或形状码，
                       weight 越大越近（球/靶用像素半径，人质用外接框面积）。
            target_id: 目标身份（二维码给出的颜色或形状）；None 表示目标未知。
            width, height: 当前帧像素尺寸，用于计算画面中心与容差框。
        返回:
            (target_pos, centered, new_positions)：
              target_pos     目标已分配的点位（未出现为 0）；
              centered       本帧/锁存后是否处于居中状态，True 即请求底盘停车；
              new_positions  本帧新分配的点位列表（升序），供上行 POINT 事件使用。
        """
        if self.stopped:                           # 已锁存：点位与停车状态保持不变
            return self.stop_position, True, []

        # 同一身份可能有多件（如同形状误检），只保留最近的一件
        best = {}
        for ident, u, v, weight in items:
            if ident not in best or weight > best[ident][2]:
                best[ident] = (u, v, weight)

        # 本帧首次出现的身份按由近到远分配点位
        new_positions = []
        for ident, (_u, _v, _weight) in sorted(best.items(), key=lambda kv: -kv[1][2]):
            if ident not in self._positions:
                self._positions[ident] = self._next
                new_positions.append(self._next)
                self._next += 1

        pos = self._positions.get(target_id, 0)
        if pos == 0:
            return 0, False, new_positions

        cx, cy = width / 2.0, height / 2.0
        tol_x, tol_y = self.center_tol * width, self.center_tol * height
        u, v, _ = best.get(target_id, (None, None, 0.0))
        if u is not None and abs(u - cx) <= tol_x and abs(v - cy) <= tol_y:
            self.stopped = True
            self.stop_position = pos
            return pos, True, new_positions
        return pos, False, new_positions
