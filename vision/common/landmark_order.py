"""靶子和救援目标按进入中央检测区域的顺序编号；颜色或形状不是位次。"""

class LandmarkOrder:
    def __init__(self, target, direction=-1):
        self.target = target
        self.direction = direction
        self.slots = {}
        self.target_slot = 0

    def update(self, points):
        """points为类别到整图中心坐标的映射；同一类别只编号一次。"""
        if self.target_slot:
            return self.target_slot
        # 画面从右向左经过时，同时出现的目标按左到右编号。
        candidates = sorted(((point[0], code) for code, point in points.items()
                             if code in (1, 2, 3) and code not in self.slots),
                            reverse=self.direction > 0)
        for _, code in candidates:
            self.slots[code] = len(self.slots) + 1
            if code == self.target:
                self.target_slot = self.slots[code]
                break
        return self.target_slot
