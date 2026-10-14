import csv, math, os, signal
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import AccelStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rm_msgs.msg import NavigationMcuCmd, DecisionNavCmd, DecisionMcuCmd, RefereeData

class TelemetryNode(Node):
    def __init__(self):
        super().__init__('sentry_telemetry')
        directory = self.declare_parameter('output_directory', 'recordings').value
        os.makedirs(directory, exist_ok=True)
        self.file = open(os.path.join(directory,'telemetry.csv'),'w',newline='',encoding='utf-8')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['stamp_s','frame','x_m','y_m','vx_body_mps','vy_body_mps','speed_mps','ax_world_mps2','ay_world_mps2','wz_body_radps','alpha_z_world_radps2','cmd_vx_mmps','cmd_vy_mmps','cmd_wz_degps','goal_x_m','goal_y_m','requested_mmps','posture','game_time','task'])
        self.nav = NavigationMcuCmd(); self.decision = DecisionNavCmd(); self.mcu = DecisionMcuCmd(); self.referee = RefereeData()
        self.task = ''
        self.last = None
        self.last_flush = 0
        self.accel_puber = self.create_publisher(AccelStamped,'/recorded_acceleration',10)
        self.create_subscription(NavigationMcuCmd,'/navigation2mcu',self.nav_callback,10)
        self.create_subscription(DecisionNavCmd,'/decision2nav',self.decision_callback,10)
        self.create_subscription(DecisionMcuCmd,'/decision2mcu',self.mcu_callback,10)
        self.create_subscription(RefereeData,'/referee_data',self.referee_callback,10)
        self.create_subscription(String,'/decision_state',self.state_callback,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Odometry,'/navigation_odom',self.odom_callback,20)
        self.get_logger().info('RECORD: '+os.path.join(directory,'telemetry.csv'))

    def nav_callback(self, msg): self.nav = msg
    def decision_callback(self, msg): self.decision = msg
    def mcu_callback(self, msg): self.mcu = msg
    def referee_callback(self, msg): self.referee = msg
    def state_callback(self, msg): self.task = msg.data

    def odom_callback(self, msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        v,w = msg.twist.twist.linear,msg.twist.twist.angular
        q = msg.pose.pose.orientation
        norm = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
        if norm < 1e-9 or not math.isfinite(norm): return
        x,y,z,s = q.x/norm,q.y/norm,q.z/norm,q.w/norm
        # 加速度在 odom 世界系计算，不能直接差分自旋中的车体系速度。
        vx = (1-2*(y*y+z*z))*v.x+2*(x*y-s*z)*v.y+2*(x*z+s*y)*v.z
        vy = 2*(x*y+s*z)*v.x+(1-2*(x*x+z*z))*v.y+2*(y*z-s*x)*v.z
        if not all(math.isfinite(a) for a in [stamp,vx,vy,w.z]): return
        world_wz = 2*(x*z-s*y)*w.x+2*(y*z+s*x)*w.y+(1-2*(x*x+y*y))*w.z
        ax = ay = alpha = ''
        if self.last is not None:
            dt = stamp-self.last[0]
            if dt <= 0: return
            if dt < 0.5:
                ax,ay,alpha = (vx-self.last[1])/dt,(vy-self.last[2])/dt,(world_wz-self.last[3])/dt
                accel = AccelStamped(); accel.header = msg.header
                accel.accel.linear.x = ax; accel.accel.linear.y = ay; accel.accel.angular.z = alpha
                self.accel_puber.publish(accel)
        self.last = (stamp,vx,vy,world_wz)
        p = msg.pose.pose.position
        self.writer.writerow([stamp,msg.header.frame_id,p.x,p.y,v.x,v.y,math.hypot(v.x,v.y),ax,ay,w.z,alpha,self.nav.vx,self.nav.vy,self.mcu.wz,self.decision.aim_x,self.decision.aim_y,self.decision.move_v,self.mcu.sentry_posture,self.referee.game_time,self.task])
        if stamp-self.last_flush >= 1:
            self.file.flush(); self.last_flush = stamp

    def destroy_node(self):
        self.file.flush(); self.file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = TelemetryNode()
    stopped = False
    def request_stop(signum, frame):
        nonlocal stopped
        stopped = True
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    executor = rclpy.get_global_executor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not stopped:
            executor.spin_once(timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException): pass
    except RuntimeError:
        # 退出时 context 可能先关闭；运行期间的异常仍正常抛出。
        if rclpy.ok(): raise
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
