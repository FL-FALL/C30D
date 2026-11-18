"""Count one red/green/blue ball each, at a central gate, in travel order."""
class BallOrder:
    def __init__(self, cfg, target):
        self.target = target
        self.axis = cfg.get("count_axis", "x")
        self.direction = int(cfg.get("count_direction", -1))
        self.center = float(cfg.get("count_gate_center", 0.5))
        self.half = float(cfg.get("count_gate_half_width", 0.2))
        self.confirm = max(1, int(cfg.get("count_confirm_frames", 3)))
        self.hits = {}
        self.slots = {}
        self.target_slot = 0

    def update(self, points, width, height):
        if self.target_slot:
            return self.target_slot
        axis = 0 if self.axis == "x" else 1
        extent = width if axis == 0 else height
        candidates = []
        for color in (1, 2, 3):
            point = points.get(color)
            inside = point is not None and abs(point[axis]/extent-self.center) <= self.half
            self.hits[color] = self.hits.get(color, 0)+1 if inside else 0
            if color not in self.slots and self.hits[color] >= self.confirm:
                candidates.append((point[axis], color))
        # Right->left: the leftmost of simultaneous candidates passed first.
        candidates.sort(reverse=self.direction > 0)
        for _, color in candidates:
            self.slots[color] = len(self.slots)+1
            if color == self.target:
                self.target_slot = self.slots[color]
                break
        return self.target_slot
