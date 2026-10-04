import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from rm_msgs.msg import RefereeData, ChassisData, DecisionMcuCmd, DecisionNavCmd
from nav_msgs.msg import Odometry

class DecisionNode(Node):
    def __init__(self) -> None:
        super().__init__("sentry_decision_node")
        # TF 监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.inputs = {}
        self.receive_counts = {'odom': 0, 'referee': 0, 'chassis': 0}
        self.publish_count = 0

        # 订阅话题
        self.referee_suber = self.create_subscription(
            msg_type=RefereeData,
            topic="/referee_data",
            callback=self.referee_callback,
            qos_profile=10,
        )

        self.odom_suber = self.create_subscription(
            msg_type=Odometry,
            topic="/odin1/odometry_highfreq",
            callback=self.odom_callback,
            qos_profile=10,
        )

        self.chassis_suber = self.create_subscription(
            msg_type=ChassisData,
            topic="/chassis_data",
            callback=self.chassis_callback,
            qos_profile=10,
        )

        self.nav_publisher = self.create_publisher(DecisionNavCmd, '/decision2nav', 10)
        self.mcu_publisher = self.create_publisher(DecisionMcuCmd, '/decision2mcu', 10)
        self.timer = self.create_timer(0.05, self.tick)

        print("\33[32m sentry_decision_node has been built \33[0m", flush=True)

    def cache(self, name, msg):
        self.inputs[name] = msg
        self.receive_counts[name] += 1

    def referee_callback(self, msg : RefereeData) -> None:
        self.cache('referee', msg)
        self.get_logger().info(
            f'DECISION RX referee_data: game_time={msg.game_time}, '
            f'sentry_posture={msg.sentry_posture}', throttle_duration_sec=1.0)

    def odom_callback(self, msg : Odometry) -> None:
        self.cache('odom', msg)
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.get_logger().info(
            f'DECISION RX odometry: position=({p.x}, {p.y}, {p.z}), '
            f'quaternion=({q.x}, {q.y}, {q.z}, {q.w})', throttle_duration_sec=1.0)

    def chassis_callback(self, msg : ChassisData) -> None:
        self.cache('chassis', msg)
        self.get_logger().info(
            f'DECISION RX chassis_data: vx={msg.vx}, vy={msg.vy} mm/s, '
            f'wz={msg.wz} deg/s', throttle_duration_sec=1.0)

    def tick(self):
        nav = DecisionNavCmd()
        # TEST
        nav.is_cross = False
        nav.aim_x, nav.aim_y, nav.move_v = 1.0, 1.0, 1
        self.nav_publisher.publish(nav)
        mcu = DecisionMcuCmd()
        mcu.wz, mcu.sentry_posture = 1.0, 3
        self.mcu_publisher.publish(mcu)
        self.publish_count += 1
        self.get_logger().info(
            f'DECISION PUB #{self.publish_count}: decision2nav(is_cross={nav.is_cross}, '
            f'aim=({nav.aim_x}, {nav.aim_y}), move_v={nav.move_v} mm/s), '
            f'decision2mcu(wz={mcu.wz} deg/s, posture={mcu.sentry_posture}); '
            f'RX counts={self.receive_counts}', throttle_duration_sec=1.0)

def cleanup_node(node: Node) -> None:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

def spin_node(node: Node) -> None:
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

def main(args=None) -> None:
    rclpy.init(args=args)
    node = DecisionNode()
    try:
        spin_node(node)
    finally:
        cleanup_node(node)
