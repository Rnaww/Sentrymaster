import math, py_trees

# 按 diagram 的 701 点表组织任务。坐标为 map 米，自旋为度/s。
B = (7.25, 2.05)
S = (15.55, 1.75)
U0 = (15.15, 2.15)
U1 = (15.15, 6.35)
C = (8.85, 10.05)
T0 = (4.15, 6.25)
T1 = (2.75, 4.55)
T = (3.15, 3.45)
F = (8.95, 2.55)
R = (8.05, 4.05)

class Unsafe(py_trees.behaviour.Behaviour):
    def __init__(self, node):
        super().__init__('检查比赛与定位')
        self.node = node

    def update(self):
        if not self.node.inputs_valid(): return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class Stop(py_trees.behaviour.Behaviour):
    def __init__(self, node):
        super().__init__('停车')
        self.node = node

    def update(self):
        self.node.move_v = 0
        self.node.wz = 0
        self.node.posture = 3
        return py_trees.common.Status.RUNNING

class Navigate(py_trees.behaviour.Behaviour):
    def __init__(self, node, name, point, speed=3500, spin=90, is_cross=False, deadline=300):
        super().__init__(name)
        self.node = node
        self.point = point
        self.speed = speed
        self.spin = spin
        self.is_cross = is_cross
        self.deadline = deadline
        self.complete = False

    def initialise(self):
        self.start_time = self.node.time_now()
        self.fail_time = None
        self.retries = 0
        self.retry_until = 0

    def update(self):
        node = self.node
        if self.complete: return py_trees.common.Status.SUCCESS
        node.goal = self.point
        node.is_cross = self.is_cross
        node.posture = 3
        node.wz = self.spin
        node.move_v = self.speed
        if node.elapsed >= self.deadline:
            node.move_v = 0
            self.complete = True
            return py_trees.common.Status.SUCCESS
        if math.hypot(node.x-self.point[0], node.y-self.point[1]) <= 0.25:
            self.complete = True
            return py_trees.common.Status.SUCCESS
        now = node.time_now()
        if now < self.retry_until:
            node.move_v = 0
            return py_trees.common.Status.RUNNING
        failed = node.nav_status is not None and node.nav_status.state == 3 and math.hypot(node.nav_status.aim_x-self.point[0], node.nav_status.aim_y-self.point[1]) < 0.01
        if failed:
            if self.fail_time is None: self.fail_time = now
            if now-self.fail_time > 2:
                self.retries += 1
                self.fail_time = None
                self.retry_until = now+0.5
                if self.retries > 2:
                    node.abort = True
                    node.get_logger().error(f'{self.name}: navigation failed')
                    return py_trees.common.Status.FAILURE
        else: self.fail_time = None
        if now-self.start_time > 60:
            node.abort = True
            node.get_logger().error(f'{self.name}: navigation timeout')
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING

class Hold(py_trees.behaviour.Behaviour):
    def __init__(self, node, name, point, duration, earliest=0, deadline=300, spin=0, supply=False):
        super().__init__(name)
        self.node = node
        self.point = point
        self.duration = duration
        self.earliest = earliest
        self.deadline = deadline
        self.complete = False
        self.spin = spin
        self.supply = supply

    def initialise(self):
        self.hold_since = None
        self.stable_since = None

    def update(self):
        node = self.node
        if self.complete: return py_trees.common.Status.SUCCESS
        node.goal = self.point
        node.is_cross = False
        node.move_v = 0
        node.wz = self.spin
        node.posture = 1
        if node.elapsed >= self.deadline:
            self.complete = True
            return py_trees.common.Status.SUCCESS
        valid = math.hypot(node.x-self.point[0], node.y-self.point[1]) <= 0.4 and node.linear_speed < 0.05
        valid = valid and (not self.spin or abs(node.actual_wz) >= math.radians(30))
        now = node.time_now()
        if valid:
            if self.stable_since is None: self.stable_since = now
        else:
            self.stable_since = None
            self.hold_since = None
        if not valid or node.actual_posture != 1 or node.elapsed < self.earliest:
            self.hold_since = None
            return py_trees.common.Status.RUNNING
        if self.hold_since is None: self.hold_since = now
        # 稳定确认包含在有效停留中，不能再额外加两秒。
        if now-self.stable_since >= 2 and now-self.hold_since >= self.duration:
            self.complete = True
            node.get_logger().info(f'{self.name}: complete, elapsed={node.elapsed:.1f}')
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

def create_SentryMaster_tree(node):
    safety = py_trees.composites.Sequence('安全停车', memory=False, children=[Unsafe(node), Stop(node)])
    tasks = py_trees.composites.Sequence('新生赛任务', memory=True, children=[
        Navigate(node, '启动到补给1', S, spin=0),
        Hold(node, '补给1', S, 5.5, deadline=60, supply=True),
        Navigate(node, '起伏入口', U0, speed=650, deadline=180),
        Navigate(node, '通过起伏路段', U1, speed=450, spin=90, is_cross=True, deadline=180),
        Navigate(node, '中央高地', C, deadline=180),
        Hold(node, '中央高地停留', C, 20.5, deadline=180),
        Navigate(node, '中央返回起伏出口', U1, spin=0),
        Navigate(node, '返回起伏入口', U0, speed=450, spin=0, is_cross=True),
        Navigate(node, '返回补给2', S, speed=650, spin=0),
        Hold(node, '补给2', S, 5.5, earliest=60, deadline=120, supply=True),
        Navigate(node, '梯形坡道入口', T0, deadline=180),
        Navigate(node, '通过梯形坡道', T1, speed=450, spin=90, is_cross=True, deadline=180),
        Navigate(node, '梯形高地', T, speed=450, deadline=180),
        Hold(node, '梯形高地停留', T, 20.5, deadline=180),
        Navigate(node, '梯形返回坡道上方', T1, speed=450, spin=0),
        Navigate(node, '梯形返回坡道下方', T0, speed=450, spin=0, is_cross=True),
        Navigate(node, '返回补给3', S, spin=0),
        Hold(node, '补给3', S, 5.5, earliest=120, deadline=180, supply=True),
        Navigate(node, '堡垒中央', F),
        Hold(node, '堡垒旋转停留', F, 20.5, earliest=180, spin=90),
        Navigate(node, '返回补给4', S, spin=0),
        Hold(node, '补给4', S, 5.5, earliest=180, deadline=240, supply=True),
        # 等到最后一分钟再执行离区往返。
        Hold(node, '等待第5分钟', S, 0, earliest=240.5),
        Navigate(node, '补给离区检查点', R, spin=0),
        Navigate(node, '返回补给5', S, spin=0),
        Hold(node, '补给5', S, 5.5, earliest=240, supply=True),
    ])
    return py_trees.composites.Selector('SentryMaster', memory=False, children=[safety, tasks, Stop(node)])
