import math, py_trees

# 按 diagram 的 701 点表组织任务。坐标为 map 米，自旋为度/s。
B = (7.25, 2.70)
S = (15.45, 1.95)
U0 = (15.45, 2.55)
U1 = (15.45, 6.35)
C = (8.85, 10.05)
T = (2.5, 3.0)
F = (8.95, 2.55)
# 控制用的平地对正点和通道出口，不是新增计分任务。
C_ENTRY = (15.15, 9.0)
C_RAMP_TOP = (13.3, 9.0)
F_ENTRY = (10.3, 2.5)
T_ENTRY = (4.8, 5.8)
ROUGH_EXIT = (15.45, 7.3)
# diagram 的第五次补给离区参考点，不是裁判任务点。
R = (10.05, 3.85)


class Unsafe(py_trees.behaviour.Behaviour):
    def __init__(self, node) -> None:
        super().__init__('检查比赛与定位')
        self.node = node

    def update(self) -> py_trees.common.Status:
        if not self.node.inputs_valid():
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class Stop(py_trees.behaviour.Behaviour):
    def __init__(self, node) -> None:
        super().__init__('停车')
        self.node = node

    def update(self) -> py_trees.common.Status:
        self.node.move_v = 0
        self.node.aim_chassis_yaw = None
        self.node.wz = 0
        self.node.posture = 3
        return py_trees.common.Status.RUNNING


class Navigate(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        node,
        name,
        point,
        speed=4500,
        spin=None,
        is_cross=False,
        deadline=300,
        reach_thresh=0.25,
        heading=None,
        stop_at_goal=True,
        keep_heading=False
    ) -> None:
        super().__init__(name)
        self.node = node
        self.point = point
        self.speed = speed
        self.spin = node.spin_wz if spin is None else spin
        self.is_cross = is_cross
        self.deadline = deadline
        self.reach_thresh = reach_thresh
        self.heading = heading
        self.stop_at_goal = stop_at_goal
        self.keep_heading = keep_heading
        self.terrain_entered = False
        self.complete = self.failed = False
        self.start_time = None
        self.travel_time = 0
        self.progress_time = None
        self.progress_position = None
        self.retries = 0
        self.retry_until = 0

    def fail(self, reason) -> py_trees.common.Status:
        self.failed = True
        self.node.move_v = 0
        self.node.get_logger().warning(f'{self.name}: skipped, {reason}, elapsed={self.node.elapsed:.1f}')
        return py_trees.common.Status.FAILURE

    def update(self) -> py_trees.common.Status:
        node = self.node
        if self.complete:
            return py_trees.common.Status.SUCCESS
        if self.failed:
            return py_trees.common.Status.FAILURE
        now = node.time_now()
        if self.start_time is None:
            self.start_time = now
        self.travel_time = now - self.start_time
        if node.elapsed >= self.deadline:
            return self.fail('deadline')
        point, heading, speed, is_cross = self.point, self.heading, self.speed, self.is_cross
        escaping = self.heading is None and node.passage_exit is not None
        if escaping:
            if math.dist((node.x, node.y), node.passage_exit) <= 0.15 and node.linear_speed < 0.05:
                node.passage_exit = None
                escaping = False
            else:
                point, heading, is_cross = node.passage_exit, node.passage_yaw, True
        node.goal = point
        node.is_cross = is_cross
        node.posture = 3
        node.wz = self.spin
        if heading is not None:
            if node.terrain_no_spin:
                self.terrain_entered = True
            if not node.terrain_no_spin and (not self.keep_heading or self.terrain_entered):
                heading = None
            else:
                heading = node.path_heading(point, heading)
                # 目标附近的纵向回调允许倒车修正，不能为了几厘米误差原地转 180 度。
                if math.dist((node.x, node.y), point) < 0.2 and math.cos(heading - node.yaw) < 0:
                    heading = math.atan2(math.sin(heading + math.pi), math.cos(heading + math.pi))
        node.aim_chassis_yaw = heading
        node.aligning = False
        node.move_v = speed
        if heading is not None and abs(math.atan2(math.sin(heading - node.yaw), math.cos(heading - node.yaw))) > 0.25:
            node.move_v = 0
        if math.dist((node.x, node.y), point
                     ) <= self.reach_thresh and (not self.stop_at_goal or node.linear_speed < 0.05) and not escaping:
            self.complete = True
            return py_trees.common.Status.SUCCESS
        if now < self.retry_until:
            node.move_v = 0
            return py_trees.common.Status.RUNNING
        position = (node.x, node.y)
        if self.progress_time is None or math.dist(position, self.progress_position) >= 0.1:
            self.progress_time, self.progress_position = now, position
        if now - self.progress_time >= node.no_progress_timeout:
            if self.retries >= node.navigation_retries:
                return self.fail('no progress after replanning')
            self.retries += 1
            self.progress_time, self.progress_position = now, position
            self.retry_until = now + 0.25
            node.move_v = 0
            node.get_logger().warning(f'{self.name}: replan {self.retries}/{node.navigation_retries}')
        return py_trees.common.Status.RUNNING


class AlignChassis(py_trees.behaviour.Behaviour):
    def __init__(self, node, name, point, heading, deadline) -> None:
        super().__init__(name)
        self.node, self.point, self.heading, self.deadline = node, point, heading, deadline
        self.stable_since = self.start_time = None
        self.stopped_since = None
        self.travel_time = 0

    def initialise(self) -> None:
        self.stable_since = self.stopped_since = None

    def update(self) -> py_trees.common.Status:
        node = self.node
        now = node.time_now()
        if self.start_time is None:
            self.start_time = now
        self.travel_time = now - self.start_time
        node.goal = self.point
        node.move_v = 0
        node.wz = node.spin_wz
        node.posture = 3
        node.aim_chassis_yaw = None
        node.aligning = False
        if node.elapsed >= self.deadline or self.travel_time >= 10:
            node.get_logger().warning(f'{self.name}: alignment timeout')
            return py_trees.common.Status.FAILURE
        # 先确认平移停稳，避免残余速度与入口对正混在一起。
        if node.linear_speed >= 0.02:
            self.stopped_since = None
        elif self.stopped_since is None:
            self.stopped_since = now
        if self.stopped_since is None or now - self.stopped_since < 0.2:
            return py_trees.common.Status.RUNNING
        node.wz = 0
        node.aim_chassis_yaw = self.heading
        node.aligning = True
        error = math.atan2(math.sin(self.heading - node.yaw), math.cos(self.heading - node.yaw))
        # 沿用旧 CheckYawAction：0.1rad 内连续保持 0.5s。
        if abs(error) < 0.1 and node.linear_speed < 0.05:
            if self.stable_since is None:
                self.stable_since = now
            if now - self.stable_since >= 0.5:
                return py_trees.common.Status.SUCCESS
        else:
            self.stable_since = None
        return py_trees.common.Status.RUNNING


class PassageState(py_trees.behaviour.Behaviour):
    def __init__(self, node, name, end=None, heading=None) -> None:
        super().__init__(name)
        self.node, self.end, self.heading = node, end, heading

    def update(self) -> py_trees.common.Status:
        self.node.passage_exit = self.end
        self.node.passage_yaw = self.heading
        return py_trees.common.Status.SUCCESS


class CrossTerrainSequence(py_trees.composites.Sequence):
    # 参照旧 RMUC：入口、对正、跨越、更新跨越状态。
    def __init__(
        self,
        node,
        name,
        start_point,
        end_point,
        deadline=300,
        reach_thresh=0.15,
        stop_at_goal=False,
        heading=None,
        keep_heading=False
    ) -> None:
        if heading is None:
            heading = math.atan2(end_point[1] - start_point[1], end_point[0] - start_point[0])
        super().__init__(
            name,
            memory=True,
            children=[
                Navigate(node, name + '入口', start_point, deadline=deadline, reach_thresh=0.1),
                AlignChassis(node, name + '对正', start_point, heading, deadline),
                PassageState(node, name + '开始', end_point, heading),
                Navigate(
                    node,
                    name + '跨越',
                    end_point,
                    spin=node.spin_wz,
                    is_cross=True,
                    deadline=deadline,
                    reach_thresh=reach_thresh,
                    heading=heading,
                    stop_at_goal=stop_at_goal,
                    keep_heading=keep_heading
                ),
                PassageState(node, name + '结束'),
            ]
        )

    @property
    def travel_time(self) -> float:
        return sum(getattr(child, 'travel_time', 0) for child in self.children)


class Hold(py_trees.behaviour.Behaviour):
    def __init__(self, node, name, point, duration, earliest=0, deadline=300, spin=0, supply=False) -> None:
        super().__init__(name)
        self.node = node
        self.point = point
        self.duration = duration
        self.earliest = earliest
        self.deadline = deadline
        self.complete = False
        self.failed = False
        self.spin = spin
        self.supply = supply

    def initialise(self) -> None:
        self.hold_since = None
        self.stable_since = None

    def update(self) -> py_trees.common.Status:
        node = self.node
        if self.complete:
            return py_trees.common.Status.SUCCESS
        if self.failed:
            return py_trees.common.Status.FAILURE
        node.aim_chassis_yaw = None
        node.goal = self.point
        node.is_cross = False
        node.move_v = 0
        # 制动仍遵循地形禁转；停稳后执行停留要求。
        node.wz = self.spin if node.linear_speed < 0.05 else node.spin_wz
        node.posture = 1
        if node.elapsed >= self.deadline:
            self.failed = True
            node.get_logger().warning(f'{self.name}: skipped, deadline')
            return py_trees.common.Status.FAILURE
        valid = math.hypot(node.x - self.point[0], node.y - self.point[1]) <= 0.4 and node.linear_speed < 0.05
        valid = valid and (
            not self.spin or node.disable_spin or
            (not node.terrain_no_spin and abs(node.actual_wz) >= math.radians(30))
        )
        now = node.time_now()
        if valid:
            if self.stable_since is None:
                self.stable_since = now
        else:
            self.stable_since = None
            self.hold_since = None
        if not valid or node.actual_posture != 1 or node.elapsed < self.earliest:
            self.hold_since = None
            return py_trees.common.Status.RUNNING
        if self.hold_since is None:
            self.hold_since = now
        # 稳定确认包含在有效停留中，不能再额外加两秒。
        if now - self.stable_since >= 2 and now - self.hold_since >= self.duration:
            self.complete = True
            node.get_logger().info(f'{self.name}: complete, elapsed={node.elapsed:.1f}')
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class AtFortress(py_trees.behaviour.Behaviour):
    def __init__(self, node) -> None:
        super().__init__('仍在堡垒')
        self.node = node

    def update(self) -> py_trees.common.Status:
        if math.dist((self.node.x, self.node.y), F) <= 0.4:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class SupplyTimeAvailable(py_trees.behaviour.Behaviour):
    def __init__(self, node, fortress_leg) -> None:
        super().__init__('检查第4次补给时间')
        self.node = node
        self.fortress_leg = fortress_leg
        self.available = None

    def update(self) -> py_trees.common.Status:
        node = self.node
        if self.available is None:
            # 裁判时间是整数秒，另计收包年龄；返程先按去程快20%作乐观估计。
            elapsed = node.elapsed + 1 + max(0, node.time_now() - node.recv_times['referee'])
            distance = math.hypot(node.x - S[0], node.y - S[1])
            travel = max(distance / 4.5, self.fortress_leg.travel_time * 0.8)
            self.available = elapsed + travel + 5.5 <= 240
        if self.available:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


def create_SentryMaster_tree(node) -> py_trees.composites.Selector:
    safety = py_trees.composites.Sequence(
        name='安全停车',
        memory=False,
        children=[
            Unsafe(node),
            Stop(node),
        ],
    )

    def mission(name: str, children: list) -> py_trees.decorators.FailureIsSuccess:
        # 整项任务失败后跳过，不能继续执行它的停留计分节点。
        task = py_trees.composites.Sequence(
            name=name + '流程',
            memory=True,
            children=children,
        )
        return py_trees.decorators.FailureIsSuccess(name=name, child=task)

    fortress_leg = CrossTerrainSequence(node, '堡垒东侧上坡', F_ENTRY, F, reach_thresh=0.04, stop_at_goal=True)
    final_supply = py_trees.composites.Selector(
        name='最后两次补给',
        memory=True,
        children=[
            py_trees.composites.Sequence(
                name='按时完成第4次补给',
                memory=True,
                children=[
                    SupplyTimeAvailable(node, fortress_leg),
                    CrossTerrainSequence(node, '堡垒东侧下坡', F, F_ENTRY, deadline=240),
                    Navigate(node, '返回补给4', S, deadline=240),
                    Hold(node, '补给4', S, 5.5, earliest=180, deadline=240, supply=True),
                    Hold(node, '等待第5分钟', S, 0, earliest=240.5),
                    Navigate(node, '补给离区检查点', R),
                ],
            ),
            py_trees.composites.Sequence(
                name='直接完成第5次补给',
                memory=True,
                children=[
                    py_trees.composites.Selector(
                        name='选择第5分钟等待点',
                        memory=True,
                        children=[
                            py_trees.composites.Sequence(
                                name='堡垒等待',
                                memory=True,
                                children=[
                                    AtFortress(node),
                                    Hold(node, '等待第5分钟补给', F, 0, earliest=240.5, spin=node.spin_wz),
                                    CrossTerrainSequence(node, '堡垒东侧下坡5', F, F_ENTRY),
                                ],
                            ),
                            py_trees.composites.Sequence(
                                name='已离开堡垒',
                                memory=True,
                                children=[
                                    Navigate(node, '补给5离区', R),
                                    Hold(node, '离区等待第5分钟', R, 0, earliest=240.5, spin=node.spin_wz),
                                ],
                            ),
                        ],
                    ),
                ],
            ),
        ],
    )
    tasks = py_trees.composites.Sequence(
        name='新生赛任务',
        memory=True,
        children=[
            mission(
                '第1次补给', [
                    Navigate(node, '启动到补给1', S, deadline=60),
                    Hold(node, '补给1', S, 5.5, deadline=60, supply=True),
                ]
            ),
            mission(
                '中央任务', [
                    CrossTerrainSequence(node, '起伏正向', S, ROUGH_EXIT, deadline=180),
                    CrossTerrainSequence(node, '中央上坡', C_ENTRY, C, deadline=180, stop_at_goal=True, heading=math.pi),
                    Hold(node, '中央高地停留', C, 20.5, deadline=180),
                ]
            ),
            mission(
                '第2次补给', [
                    CrossTerrainSequence(node, '中央下坡', C_RAMP_TOP, C_ENTRY, deadline=120, heading=0),
                    CrossTerrainSequence(node, '起伏返回', ROUGH_EXIT, S, deadline=120),
                    Hold(node, '补给2', S, 5.5, earliest=60, deadline=120, supply=True),
                ]
            ),
            mission(
                '梯形任务', [
                    CrossTerrainSequence(
                        node, '梯形上坡', T_ENTRY, T, deadline=180, stop_at_goal=True
                    ),
                    Hold(node, '梯形高地停留', T, 20.5, deadline=180),
                ]
            ),
            mission(
                '第3次补给', [
                    CrossTerrainSequence(node, '梯形下坡', T, T_ENTRY, deadline=180, keep_heading=True),
                    Navigate(node, '返回补给3', S, deadline=180),
                    Hold(node, '补给3', S, 5.5, earliest=120, deadline=180, supply=True),
                ]
            ),
            mission('堡垒任务', [
                fortress_leg,
                Hold(node, '堡垒旋转停留', F, 20.5, earliest=180, spin=node.spin_wz),
            ]),
            mission(
                '最后补给', [
                    final_supply,
                    Navigate(node, '返回补给5', S),
                    Hold(node, '补给5', S, 5.5, earliest=240, supply=True),
                ]
            ),
            Stop(node),
        ],
    )
    return py_trees.composites.Selector(
        name='SentryMaster',
        memory=False,
        children=[
            safety,
            tasks,
            Stop(node),
        ],
    )
