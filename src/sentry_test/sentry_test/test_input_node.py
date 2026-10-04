"""Publish temporary inputs on the same topics and types as Odin and the MCU."""
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rm_msgs.msg import ChassisData, RefereeData
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header


class TestInputNode(Node):
    def __init__(self):
        super().__init__('test_input_node')
        self.rate = self.declare_parameter('publish_rate_hz', 20.0).value
        self.x = self.declare_parameter('x', 0.0).value
        self.y = self.declare_parameter('y', 0.0).value
        self.game_time = self.declare_parameter('game_time', 1).value
        self.posture = self.declare_parameter('sentry_posture', 3).value
        self.vx = self.declare_parameter('chassis_vx', 1.0).value
        self.vy = self.declare_parameter('chassis_vy', 1.0).value
        self.wz = self.declare_parameter('chassis_wz', 1.0).value
        if (not math.isfinite(self.rate) or not 0 < self.rate <= 1000 or
                not all(math.isfinite(v) and abs(v) <= 3.402823466e38
                        for v in (self.x, self.y, self.vx, self.vy, self.wz)) or
                not 0 <= self.game_time <= 65535 or self.posture not in (1, 2, 3)):
            raise ValueError('Invalid test input parameters')
        self.odom_pub = self.create_publisher(Odometry, '/odin1/odometry_highfreq', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/odin1/cloud_raw', 10)
        self.referee_pub = self.create_publisher(RefereeData, '/referee_data', 10)
        self.chassis_pub = self.create_publisher(ChassisData, '/chassis_data', 10)
        self.count = 0
        self.timer = self.create_timer(1.0 / self.rate, self.publish_inputs)
        self.get_logger().info(
            f'TEST INPUT YAML: publish_rate_hz={self.rate}, x={self.x}, y={self.y}, '
            f'game_time={self.game_time}, sentry_posture={self.posture}, '
            f'chassis_vx={self.vx}, chassis_vy={self.vy}, chassis_wz={self.wz}')

    def publish_inputs(self):
        # TEST INPUT: disable this node when using actual Odin/MCU publishers.
        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'odin1_base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation.w = 1.0  # Identity rotation, not a zero quaternion.
        cloud = create_cloud_xyz32(
            Header(stamp=stamp, frame_id='odin1_base_link'), [(1.0, 1.0, 1.0)])
        referee = RefereeData()
        referee.game_time, referee.sentry_posture = self.game_time, self.posture
        chassis = ChassisData()
        chassis.vx, chassis.vy, chassis.wz = self.vx, self.vy, self.wz
        self.odom_pub.publish(odom)
        self.cloud_pub.publish(cloud)
        self.referee_pub.publish(referee)
        self.chassis_pub.publish(chassis)
        self.count += 1
        self.get_logger().info(
            f'TEST PUB #{self.count}: odom=({self.x}, {self.y}, 0), quaternion=(0,0,0,1), '
            f'cloud_raw=[(1,1,1)], referee(time={self.game_time}, posture={self.posture}), '
            f'chassis=({self.vx}, {self.vy}, {self.wz})', throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TestInputNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
