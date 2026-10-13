import rclpy, py_trees, math
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from rm_msgs.msg import RefereeData, ChassisData, DecisionMcuCmd, DecisionNavCmd
from nav_msgs.msg import Odometry
from decision_trees.SentryMaster_tree import create_SentryMaster_tree, B

TICK_RATE = 100

class DecisionNode(Node):
    def __init__(self) -> None:
        super().__init__("sentry_decision_node")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
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
        self.actual_wz = 0
        self.elapsed = 0
        self.last_task = ""
        self.referee_suber = self.create_subscription(RefereeData, '/referee_data', self.referee_callback, 10)
        self.odom_suber = self.create_subscription(Odometry, '/navigation_odom', self.odom_callback, 10)
        self.chassis_suber = self.create_subscription(ChassisData, '/chassis_data', self.chassis_callback, 10)
        self.nav_publisher = self.create_publisher(DecisionNavCmd, '/decision2nav', 10)
        self.mcu_publisher = self.create_publisher(DecisionMcuCmd, '/decision2mcu', 10)
        self.tree = py_trees.trees.BehaviourTree(create_SentryMaster_tree(self))
        self.tick_timer = self.create_timer(1.0/TICK_RATE, self.tick_callback)
        print("\33[32m sentry_decision_node has been built \33[0m", flush=True)

    def time_now(self):
        return self.get_clock().now().nanoseconds/1e9

    def inputs_valid(self):
        now = self.time_now()
        return 'referee' in self.inputs and 'odom' in self.inputs and 0 < self.inputs['referee'].game_time <= 300 and now-self.recv_times['referee'] < 1 and now-self.recv_times['odom'] < 0.5

    def referee_callback(self, msg : RefereeData) -> None:
        self.inputs['referee'] = msg
        self.recv_times['referee'] = self.time_now()
        self.elapsed = 300-msg.game_time
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
        tx = 2*(q.y*p.z-q.z*p.y)
        ty = 2*(q.z*p.x-q.x*p.z)
        tz = 2*(q.x*p.y-q.y*p.x)
        self.x = p.x+q.w*tx+q.y*tz-q.z*ty+trans.transform.translation.x
        self.y = p.y+q.w*ty+q.z*tx-q.x*tz+trans.transform.translation.y
        self.linear_speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.actual_wz = msg.twist.twist.angular.z
        self.inputs['odom'] = msg
        self.recv_times['odom'] = self.time_now()

    def chassis_callback(self, msg : ChassisData) -> None:
        self.inputs['chassis'] = msg

    def tick_callback(self):
        self.tree.tick()
        task = self.tree.root.tip().name
        if task != self.last_task:
            self.get_logger().info(f"TASK: {task}, elapsed={self.elapsed:.1f}, posture={self.actual_posture}")
            self.last_task = task
        nav = DecisionNavCmd()
        nav.aim_x, nav.aim_y = self.goal
        nav.is_cross = self.is_cross
        nav.move_v = int(self.move_v)
        mcu = DecisionMcuCmd()
        mcu.wz = float(self.wz)
        if self.posture != self.actual_posture and self.time_now()-self.last_posture_change < 5:
            mcu.sentry_posture = self.actual_posture
        else:
            mcu.sentry_posture = self.posture
        self.mcu_publisher.publish(mcu)
        self.nav_publisher.publish(nav)

    def stop(self):
        self.nav_publisher.publish(DecisionNavCmd())
        msg = DecisionMcuCmd()
        msg.sentry_posture = 3
        self.mcu_publisher.publish(msg)

def cleanup_node(node: Node) -> None:
    node.destroy_node()
    if rclpy.ok(): rclpy.shutdown()

def spin_node(node: Node) -> None:
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass

def main(args=None) -> None:
    rclpy.init(args=args)
    node = DecisionNode()
    try: spin_node(node)
    finally: cleanup_node(node)
