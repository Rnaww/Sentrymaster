import math

# 区域中心沿用 diagram；停留检测半径与主分支使用相同 map 坐标。
POINTS = {'supply': (15.55, 1.75), 'central': (8.85, 10.05), 'trapezoid': (3.15, 3.45), 'fortress': (8.95, 2.55)}

class Scoring:
    def __init__(self):
        self.score = 0
        self.events = []
        self.hold_since = {}
        self.supply_minutes = set()
        self.done = set()
        self.last_supply = (7.25, 2.05)
        self.excursion = math.dist(self.last_supply, POINTS['supply'])
        self.rough_entered = False
        self.rough_crossed = False
        self.slope_entered = False
        self.slope_crossed = False
        self.in_supply = False
        self.spin_reward = False
        self.spin_valid = True
        self.scoring_time = 0
        self.last_time = 0

    def update(self, t, x, y, speed, wz, posture, terrain_exception):
        dt = max(0, min(t-self.last_time, 0.2))
        self.last_time = t
        if posture == 1: self.scoring_time += dt
        self.excursion = max(self.excursion, math.dist((x,y), self.last_supply))
        in_supply = math.dist((x,y), POINTS['supply']) <= 0.6
        if in_supply and not self.in_supply:
            self.spin_reward = True
            self.spin_valid = True
        self.in_supply = in_supply
        traveling = speed > 0.05 and all(math.dist((x,y), point) > 0.6 for point in POINTS.values())
        if self.spin_reward and traveling and not terrain_exception and abs(wz) < math.radians(30): self.spin_valid = False
        if math.dist((x,y), (15.15,2.15)) < 0.6: self.rough_entered = True
        if self.rough_entered and math.dist((x,y), (15.15,6.35)) < 0.6: self.rough_crossed = True
        if math.dist((x,y), (4.15,6.25)) < 0.6: self.slope_entered = True
        if self.slope_entered and math.dist((x,y), (2.75,4.55)) < 0.6: self.slope_crossed = True
        for name, point in POINTS.items():
            valid = math.dist((x,y), point) <= 0.4 and speed < 0.05
            if name == 'supply': valid &= self.excursion >= 5 and int(t//60) not in self.supply_minutes
            elif name == 'central': valid &= t < 180 and self.rough_crossed and name not in self.done
            elif name == 'trapezoid': valid &= t < 180 and self.slope_crossed and name not in self.done
            else: valid &= t >= 180 and abs(wz) >= math.radians(30) and name not in self.done
            if not valid:
                self.hold_since.pop(name, None)
                continue
            # 姿态必须在整个有效停留中保持，不能到结算时才切换。
            key = (posture, int(t//60) if name == 'supply' else 0)
            hold = self.hold_since.get(name)
            if hold is None or hold[1] != key:
                self.hold_since[name] = (t, key)
                hold = self.hold_since[name]
            duration = 5 if name == 'supply' else 20
            if t-hold[0] < duration: continue
            base = {'supply':20,'central':100,'trapezoid':50,'fortress':80}[name]
            bonus = 20 if name != 'supply' and self.spin_reward and self.spin_valid else 0
            multiplier = (1.5 if self.scoring_time <= 180 else 1.2) if posture == 1 else 1.0
            earned = (base+bonus)*multiplier
            self.score += earned
            self.events.append({'task':name, 'time':round(t,2), 'score':earned, 'spin_bonus':bonus})
            self.hold_since.pop(name, None)
            if name == 'supply':
                self.supply_minutes.add(int(t//60))
                self.last_supply = (x,y)
                self.excursion = 0
                self.spin_reward = True
                self.spin_valid = True
            else: self.done.add(name)
