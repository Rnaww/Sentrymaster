import rclpy, py_trees, math, os, signal
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from tf2_ros import Buffer, TransformListener
from rm_msgs.msg import RefereeData, ChassisData, DecisionMcuCmd, DecisionNavCmd
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
from .terrain import Terrain, rotate_vector
from decision_trees.SentryMaster_tree import create_SentryMaster_tree, B

TICK_RATE = 100


class DecisionNode(Node):
    def __init__(self) -> None:
        super().__init__("sentry_decision_node")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.is_sim = bool(self.declare_parameter("sim_mode", False).value)
        # 沿用旧 SLOW_ROTATE：仿真 50 度/s，实车 200 度/s。
        self.spin_wz = 50.0 if self.is_sim else 200.0
        self.disable_spin = self.declare_parameter('disable_spin', False).value
        self.spin_resume_delay = float(self.declare_parameter('spin_resume_delay', 0.5).value)
        self.terrain = Terrain(
            os.path.join(get_package_share_directory('sentry_planner'), 'map', '701_model', 'new_701_deep.png')
        )
        self.terrain_clear_since = None
        self.terrain_no_spin = True
        self.no_progress_timeout = float(self.declare_parameter('no_progress_timeout', 3.0).value)
        self.navigation_retries = int(self.declare_parameter('navigation_retries', 2).value)
        self.aim_chassis_yaw = None
        self.aligning = False
        self.passage_exit = self.passage_yaw = None
        self.path = None
        self.yaw = 0
        self.inputs = {}
        self.recv_times = {}
        self.goal = B
        self.move_v = 0
        self.wz = 0
        self.posture = 3
        self.is_cross = False
        self.actual_posture = 3
        self.last_posture_change = -5
        self.x, self.y = B
        self.linear_speed = 0
        self.world_vx = self.world_vy = 0
        self.actual_wz = 0
        self.elapsed = 0
        self.last_task = ""
        self.referee_suber = self.create_subscription(
            msg_type=RefereeData,
            topic='/referee_data',
            callback=self.referee_callback,
            qos_profile=10,
        )
        self.odom_suber = self.create_subscription(
            msg_type=Odometry,
            topic='/navigation_odom',
            callback=self.odom_callback,
            qos_profile=10,
        )
        self.path_suber = self.create_subscription(
            msg_type=Path,
            topic='/planned_path',
            callback=self.path_callback,
            qos_profile=10,
        )
        self.chassis_suber = self.create_subscription(
            msg_type=ChassisData,
            topic='/chassis_data',
            callback=self.chassis_callback,
            qos_profile=10,
        )
        self.nav_publisher = self.create_publisher(
            msg_type=DecisionNavCmd,
            topic='/decision2nav',
            qos_profile=10,
        )
        self.mcu_publisher = self.create_publisher(
            msg_type=DecisionMcuCmd,
            topic='/decision2mcu',
            qos_profile=10,
        )
        self.state_publisher = self.create_publisher(
            msg_type=String,
            topic='/decision_state',
            qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.tree = py_trees.trees.BehaviourTree(create_SentryMaster_tree(self))
        self.tick_timer = self.create_timer(1.0 / TICK_RATE, self.tick_callback)
        self.get_logger().info(
            f'DECISION YAML: disable_spin={self.disable_spin}, spin_resume_delay={self.spin_resume_delay}'
        )
        print("\33[32m sentry_decision_node has been built \33[0m", flush=True)

    def time_now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def inputs_valid(self) -> bool:
        now = self.time_now()
        return (
            'referee' in self.inputs
            and 'odom' in self.inputs
            and 120 < self.inputs['referee'].game_time <= 420
            and now - self.recv_times['referee'] < 1
            and now - self.recv_times['odom'] < 0.5
        )

    def referee_callback(self, msg : RefereeData) -> None:
        self.inputs['referee'] = msg
        self.recv_times['referee'] = self.time_now()
        # v1.4：比赛五分钟对应 420→120，裁判终止后为 0。
        self.elapsed = max(0, min(300, 420 - msg.game_time))
        if msg.sentry_posture != self.actual_posture:
            self.last_posture_change = self.time_now()
            self.actual_posture = msg.sentry_posture

    def odom_callback(self, msg : Odometry) -> None:
        try:
            trans = self.tf_buffer.lookup_transform('map', msg.header.frame_id, rclpy.time.Time())
        except Exception:
            return
        p = msg.pose.pose.position
        q = trans.transform.rotation
        # quaternion 旋转，包含 Odin 安装引起的非平面旋转。
        tx = 2 * (q.y * p.z - q.z * p.y)
        ty = 2 * (q.z * p.x - q.x * p.z)
        tz = 2 * (q.x * p.y - q.y * p.x)
        self.x = p.x + q.w * tx + q.y * tz - q.z * ty + trans.transform.translation.x
        self.y = p.y + q.w * ty + q.z * tx - q.x * tz + trans.transform.translation.y
        self.linear_speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.actual_wz = msg.twist.twist.angular.z
        v = msg.twist.twist.linear
        world_v = rotate_vector(msg.pose.pose.orientation, (v.x, v.y, v.z))
        world_v = rotate_vector(trans.transform.rotation, world_v)
        self.world_vx, self.world_vy = world_v[0], world_v[1]
        front = rotate_vector(msg.pose.pose.orientation, (1, 0, 0))
        front = rotate_vector(trans.transform.rotation, front)
        self.yaw = math.atan2(front[1], front[0])
        self.inputs['odom'] = msg
        self.recv_times['odom'] = self.time_now()

    def chassis_callback(self, msg : ChassisData) -> None:
        self.inputs['chassis'] = msg

    def path_callback(self, msg : Path) -> None:
        if msg.header.frame_id == 'map':
            self.path = msg

    def path_heading(self, goal, default) -> float:
        # 仅跨地形使用路径切向对正；曲线仍由导航规划，不能强制改成直线。
        if self.path is None or len(self.path.poses) < 2:
            return default
        poses = self.path.poses
        last = poses[-1].pose.position
        if math.dist((last.x, last.y), goal) > 0.1:
            return default
        nearest = None
        for i in range(1, len(poses)):
            a, b = poses[i - 1].pose.position, poses[i].pose.position
            dx, dy = b.x - a.x, b.y - a.y
            length2 = dx * dx + dy * dy
            if length2 < 1e-12:
                continue
            t = max(0, min(1, ((self.x - a.x) * dx + (self.y - a.y) * dy) / length2))
            x, y = a.x + t * dx, a.y + t * dy
            distance = (self.x - x)**2 + (self.y - y)**2
            if nearest is None or distance < nearest[0]:
                nearest = (distance, i, x, y)
        if nearest is None:
            return default
        _, i, x, y = nearest
        # 0.35m 弧长前瞻抑制栅格小折角，保留坡面上的实际行驶方向。
        px, py, remaining = x, y, 0.35
        for pose in poses[i:]:
            p = pose.pose.position
            length = math.hypot(p.x - px, p.y - py)
            if length >= remaining:
                px, py = px + (p.x - px) * remaining / length, py + (p.y - py) * remaining / length
                break
            remaining -= length
            px, py = p.x, p.y
        if math.hypot(px - x, py - y) < 1e-6:
            return default
        return math.atan2(py - y, px - x)

    def spin_command(self, requested) -> float:
        if self.aim_chassis_yaw is not None:
            # 入口已完成对正；跨越期间不再转头，也不因路径切向改变停车。
            if not self.aligning:
                return 0
            if not self.terrain.flat(self.x, self.y):
                return 0
            error = math.atan2(math.sin(self.aim_chassis_yaw - self.yaw), math.cos(self.aim_chassis_yaw - self.yaw))
            # 停车对正遵循右手系，逆时针为正。
            limit = 60
            return max(-limit, min(limit, math.degrees(2 * error)))
        return 0 if self.disable_spin or self.terrain_no_spin else requested

    def update_terrain(self) -> None:
        if not self.terrain.spin_allowed(self.x, self.y, self.world_vx, self.world_vy):
            self.terrain_clear_since = None
            self.terrain_no_spin = True
        else:
            if self.terrain_clear_since is None:
                pose = self.inputs.get('odom')
                self.terrain_clear_since = pose.header.stamp.sec + pose.header.stamp.nanosec / 1e9 if pose else self.time_now(
                )
            self.terrain_no_spin = self.time_now() - self.terrain_clear_since < self.spin_resume_delay

    def tick_callback(self) -> None:
        self.update_terrain()
        self.tree.tick()
        task = self.tree.root.tip().name
        if task != self.last_task:
            self.get_logger().info(f"TASK: {task}, elapsed={self.elapsed:.1f}, posture={self.actual_posture}")
            self.last_task = task
            state = String()
            state.data = task
            self.state_publisher.publish(state)
        nav = DecisionNavCmd()
        nav.aim_x, nav.aim_y = self.goal
        nav.is_cross = self.is_cross
        nav.move_v = int(self.move_v)
        mcu = DecisionMcuCmd()
        mcu.wz = float(self.spin_command(self.wz))
        if self.posture != self.actual_posture and self.time_now() - self.last_posture_change < 5:
            mcu.sentry_posture = self.actual_posture
        else:
            mcu.sentry_posture = self.posture
        self.mcu_publisher.publish(mcu)
        self.nav_publisher.publish(nav)

    def stop(self) -> None:
        self.nav_publisher.publish(DecisionNavCmd())
        msg = DecisionMcuCmd()
        msg.sentry_posture = 3
        self.mcu_publisher.publish(msg)


def cleanup_node(node: Node) -> None:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def spin_node(node: Node) -> None:
    stopped = False

    def request_stop(signum, frame) -> None:
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    executor = rclpy.get_global_executor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not stopped:
            executor.spin_once(timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # 退出时 context 可能先关闭；运行期间的异常仍正常抛出。
        if rclpy.ok():
            raise
    finally:
        executor.remove_node(node)


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = DecisionNode()
    try:
        spin_node(node)
    finally:
        cleanup_node(node)
