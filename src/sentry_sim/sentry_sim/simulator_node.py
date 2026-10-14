import rclpy, math, time, json, os, signal
from rclpy.signals import SignalHandlerOptions
import xml.etree.ElementTree as ET
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.executors import ExternalShutdownException
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from rm_msgs.msg import NavigationMcuCmd, DecisionMcuCmd, RefereeData, ChassisData
from PIL import Image
from .scoring import Scoring
from sentry_decision_sys.terrain import Terrain

def multiply(a, b):
    x,y,z,w = a
    X,Y,Z,W = b
    return (w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X, w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z)

def rotate(q, p):
    value = multiply(multiply(q, (*p,0)), (-q[0],-q[1],-q[2],q[3]))
    return value[:3]

# 仿真传感器安装关系直接读取共用 URDF，不另用近似角度和手写偏移。
origin = ET.parse(os.path.join(get_package_share_directory('sentry_launch'),'urdf','sentry_chassis.xacro')).find('./joint[@name="vehicle_to_odin"]/origin')
roll,pitch,yaw = [float(v)/2 for v in origin.get('rpy').split()]
sr,cr,sp,cp,sy,cy = math.sin(roll),math.cos(roll),math.sin(pitch),math.cos(pitch),math.sin(yaw),math.cos(yaw)
q = (sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy,cr*cp*cy+sr*sp*sy)
FIXED_ROTATION = (-q[0],-q[1],-q[2],q[3])
FIXED_TRANSLATION = tuple(-v for v in rotate(FIXED_ROTATION,tuple(float(v) for v in origin.get('xyz').split())))

class SimulatorNode(Node):
    def __init__(self):
        super().__init__('sentry_simulator')
        self.physics = self.declare_parameter('physics', 'kinematic').value
        self.factor = float(self.declare_parameter('realtime_factor', 1.0).value)
        self.spin_resume_delay = float(self.declare_parameter('spin_resume_delay', 0.5).value)
        self.last_terrain_unsafe = 0
        self.last_terrain_position = None
        self.last_terrain_stamp = 0
        self.output_file = self.declare_parameter('output_file', '').value
        self.x = float(self.declare_parameter('robot_x', 7.25).value)
        self.y = float(self.declare_parameter('robot_y', 2.70).value)
        self.yaw = float(self.declare_parameter('robot_yaw', 0.0).value)
        self.start_delay = float(self.declare_parameter('start_delay', 3.0).value)
        self.elapsed = 0
        self.clock_time = 0
        self.start_time = None
        self.nav = NavigationMcuCmd()
        self.decision = DecisionMcuCmd()
        self.decision.sentry_posture = 3
        self.posture = 3
        self.posture_changed = -5
        self.nav_time = self.decision_time = 0
        self.vx = self.vy = self.wz = 0
        self.gazebo_odom = None
        self.reported = False
        self.judge = Scoring()
        self.distance = 0
        self.collisions = 0
        self.max_actual_speed = self.max_command_speed = 0
        self.aligning = False
        self.terrain_alignment_seconds = 0
        self.terrain_spin_seconds = 0
        self.terrain_spin_events = []
        self.in_terrain_spin = False
        self.last_event_count = 0
        self.trajectory = []
        self.executed_path = Path()
        self.executed_path.header.frame_id = 'map'
        self.last_path_time = -1
        self.last_visual_time = -1
        self.last_wall = time.monotonic()
        self.max_acc = 1.5
        share = get_package_share_directory('sentry_planner')
        self.height = Image.open(os.path.join(share, 'map', '701_model', 'new_701_deep.png')).convert('L')
        self.terrain = Terrain(os.path.join(share, 'map', '701_model', 'new_701_deep.png'))
        self.occupancy = Image.open(os.path.join(share, 'map', '701_model', '701_navigation.png')).convert('L')
        self.nav_suber = self.create_subscription(NavigationMcuCmd, '/navigation2mcu', self.nav_callback, 10)
        self.decision_suber = self.create_subscription(DecisionMcuCmd, '/decision2mcu', self.decision_callback, 10)
        self.state_suber = self.create_subscription(String, '/decision_state', self.state_callback, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.gazebo_suber = self.create_subscription(Odometry, '/sentry_robot/odom', self.gazebo_callback, 10)
        self.clock_puber = self.create_publisher(Clock, '/clock', 10)
        self.odom_puber = self.create_publisher(Odometry, '/odin1/odometry_highfreq', 10)
        self.referee_puber = self.create_publisher(RefereeData, '/referee_data', 10)
        self.chassis_puber = self.create_publisher(ChassisData, '/chassis_data', 10)
        self.gazebo_puber = self.create_publisher(Twist, '/sentry_robot/cmd_vel', 10)
        self.score_puber = self.create_publisher(String, '/simulation_score', 10)
        self.path_puber = self.create_publisher(Path, '/executed_path', 1)
        self.field_puber = self.create_publisher(Marker, '/simulation_field', 1)
        self.joint_puber = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.get_logger().info(f'SIM YAML: physics={self.physics}, realtime_factor={self.factor}, pose=({self.x},{self.y},{self.yaw})')

    def nav_callback(self, msg):
        self.nav = msg
        self.nav_time = time.monotonic()

    def decision_callback(self, msg):
        self.decision = msg
        self.decision_time = time.monotonic()

    def state_callback(self, msg):
        self.aligning = msg.data.endswith('对正')

    def gazebo_callback(self, msg):
        self.gazebo_odom = msg

    def ground(self, x, y):
        i,j = int((x+25)/0.1), int((y+25)/0.1)
        if 0 <= i < self.height.height and 0 <= j < self.height.width:
            return self.height.getpixel((j,i))*0.8/255
        return 0

    def free(self, x, y):
        i,j = int((x+25)/0.1), int((y+25)/0.1)
        return 0 <= i < self.occupancy.height and 0 <= j < self.occupancy.width and self.occupancy.getpixel((j,i)) == 0

    def timer_callback(self):
        wall = time.monotonic()
        dt = min(wall-self.last_wall, 0.05)*self.factor
        self.last_wall = wall
        if self.physics == 'gazebo':
            if self.gazebo_odom is None: return
            stamp = self.gazebo_odom.header.stamp
            stamp_time = stamp.sec+stamp.nanosec/1e9
            if stamp_time <= self.clock_time: return
            dt = stamp_time-self.clock_time
            self.clock_time = stamp_time
            p = self.gazebo_odom.pose.pose.position
            q = self.gazebo_odom.pose.pose.orientation
            self.distance += math.hypot(p.x-self.x,p.y-self.y)
            self.x,self.y = p.x,p.y
            if not self.free(self.x,self.y): self.collisions += 1
            self.yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        else:
            self.clock_time += dt
            clock = Clock()
            clock.clock.sec = int(self.clock_time)
            clock.clock.nanosec = int((self.clock_time%1)*1e9)
            self.clock_puber.publish(clock)
        if self.start_time is None and self.clock_time >= self.start_delay:
            self.start_time = self.clock_time
        self.elapsed = 0 if self.start_time is None else min(300, self.clock_time-self.start_time)
        if self.decision.sentry_posture != self.posture and self.elapsed-self.posture_changed >= 5:
            self.posture = self.decision.sentry_posture
            self.posture_changed = self.elapsed
        active = wall-self.nav_time < 0.5 and wall-self.decision_time < 0.5 and self.start_time is not None and self.elapsed < 300
        bx = self.nav.vx/1000 if active else 0
        by = self.nav.vy/1000 if active else 0
        target_wz = math.radians(self.decision.wz) if active else 0
        c,s = math.cos(self.yaw),math.sin(self.yaw)
        vx,vy = c*bx-s*by,s*bx+c*by
        norm = math.hypot(vx,vy)
        if norm > 4.5: vx,vy = vx*4.5/norm,vy*4.5/norm
        if self.physics == 'gazebo':
            # 与旧 SimulatorBridge 一样：限幅后直接发送 Twist，由 planar_move 执行。
            self.vx,self.vy = vx,vy
        else:
            diff = math.hypot(vx-self.vx,vy-self.vy)
            scale = min(1,self.max_acc*dt/diff) if diff > 0 else 1
            self.vx += (vx-self.vx)*scale
            self.vy += (vy-self.vy)*scale
        self.wz = float(max(-math.pi,min(math.pi,target_wz)))
        cmd = Twist()
        cmd.linear.x = c*self.vx+s*self.vy
        cmd.linear.y = -s*self.vx+c*self.vy
        cmd.angular.z = self.wz
        if self.physics == 'gazebo':
            self.gazebo_puber.publish(cmd)
            speed = math.hypot(self.gazebo_odom.twist.twist.linear.x,self.gazebo_odom.twist.twist.linear.y)
            actual_wz = self.gazebo_odom.twist.twist.angular.z
        else:
            nx,ny = self.x+self.vx*dt,self.y+self.vy*dt
            if self.free(nx,ny):
                self.distance += math.hypot(nx-self.x,ny-self.y)
                self.x,self.y = nx,ny
            else:
                self.collisions += 1
                self.vx = self.vy = 0
            self.yaw += self.wz*dt
            speed = math.hypot(self.vx,self.vy)
            actual_wz = self.wz
        odom = Odometry()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'odin1_base_link'
        odom.header.stamp.sec = int(self.clock_time)
        odom.header.stamp.nanosec = int((self.clock_time%1)*1e9)
        vehicle_q = (0.0,0.0,math.sin(self.yaw/2),math.cos(self.yaw/2))
        # 与旧模型车轮中心和半径对应：0.2242 + 0.0758 m。
        vehicle_z = self.ground(self.x,self.y)+0.3
        if self.physics == 'gazebo':
            actual_q = self.gazebo_odom.pose.pose.orientation
            vehicle_q = (actual_q.x,actual_q.y,actual_q.z,actual_q.w)
            vehicle_z = self.gazebo_odom.pose.pose.position.z
        sensor_q = multiply(vehicle_q,FIXED_ROTATION)
        offset = rotate(vehicle_q,FIXED_TRANSLATION)
        odom.pose.pose.position.x = self.x+offset[0]
        odom.pose.pose.position.y = self.y+offset[1]
        odom.pose.pose.position.z = vehicle_z+offset[2]
        q = odom.pose.pose.orientation
        q.x,q.y,q.z,q.w = sensor_q
        if self.physics == 'gazebo':
            v = self.gazebo_odom.twist.twist.linear
            w = self.gazebo_odom.twist.twist.angular
            # planar_move 给出仅按 yaw 旋转的平面速度；还原 odom 后再按完整姿态转到车体。
            c,s = math.cos(self.yaw),math.sin(self.yaw)
            inverse_vehicle = (-vehicle_q[0],-vehicle_q[1],-vehicle_q[2],vehicle_q[3])
            vehicle_v = rotate(inverse_vehicle,(c*v.x-s*v.y,s*v.x+c*v.y,0.0))
            vehicle_w = rotate(inverse_vehicle,(0.0,0.0,w.z))
        else:
            vehicle_v = (cmd.linear.x,cmd.linear.y,0.0)
            vehicle_w = (0.0,0.0,self.wz)
        wx,wy,wz = vehicle_w
        tx,ty,tz = FIXED_TRANSLATION
        sensor_v = (vehicle_v[0]+wy*tz-wz*ty,vehicle_v[1]+wz*tx-wx*tz,vehicle_v[2]+wx*ty-wy*tx)
        inverse = (-FIXED_ROTATION[0],-FIXED_ROTATION[1],-FIXED_ROTATION[2],FIXED_ROTATION[3])
        odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.linear.z = rotate(inverse,sensor_v)
        odom.twist.twist.angular.x,odom.twist.twist.angular.y,odom.twist.twist.angular.z = rotate(inverse,vehicle_w)
        self.odom_puber.publish(odom)
        if self.clock_time-self.last_path_time >= 0.1 and not self.reported:
            self.last_path_time = self.clock_time
            pose = PoseStamped()
            pose.header = odom.header
            pose.header.frame_id = 'map'
            pose.pose.position.x = self.x
            pose.pose.position.y = self.y
            pose.pose.position.z = vehicle_z
            pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w = vehicle_q
            self.executed_path.poses.append(pose)
            self.trajectory.append([round(self.elapsed,3),self.x,self.y,self.yaw])
        if self.clock_time-self.last_visual_time >= 0.5:
            self.last_visual_time = self.clock_time
            self.executed_path.header.stamp = odom.header.stamp
            self.path_puber.publish(self.executed_path)
            field = Marker()
            field.header = self.executed_path.header
            field.ns = 'field_701'
            field.type = Marker.MESH_RESOURCE
            field.action = Marker.ADD
            field.pose.orientation.w = 1.0
            field.scale.x = field.scale.y = field.scale.z = 1.0
            field.color.r = field.color.g = field.color.b = 0.55
            field.color.a = 1.0
            field.mesh_resource = 'package://sentry_sim/models/rmuc_field/meshes/701.stl'
            self.field_puber.publish(field)
        if self.physics != 'gazebo':
            joints = JointState()
            joints.header = odom.header
            joints.name = ['gimbal_yaw_joint','front_left_wheel_joint','front_right_wheel_joint','rear_left_wheel_joint','rear_right_wheel_joint']
            joints.position = [0.0]*len(joints.name)
            self.joint_puber.publish(joints)
        referee = RefereeData()
        referee.game_time = 0 if self.start_time is None else max(0,int(math.ceil(300-self.elapsed)))
        referee.sentry_posture = self.posture
        self.referee_puber.publish(referee)
        chassis = ChassisData()
        if self.physics == 'gazebo':
            actual = self.gazebo_odom.twist.twist.linear
            chassis.vx,chassis.vy = actual.x*1000,actual.y*1000
        else:
            chassis.vx,chassis.vy = cmd.linear.x*1000,cmd.linear.y*1000
        chassis.wz = math.degrees(actual_wz)
        self.chassis_puber.publish(chassis)
        h = self.ground(self.x,self.y)
        world_v = rotate(vehicle_q,vehicle_v)
        if self.start_time is not None and not self.reported:
            self.max_actual_speed = max(self.max_actual_speed,speed)
            self.max_command_speed = max(self.max_command_speed,math.hypot(self.nav.vx,self.nav.vy)/1000)
            terrain_rotation = not self.terrain.flat(self.x,self.y) and abs(actual_wz) >= math.radians(30)
            # 有界的入口对正允许转向；单独记录，不能当成持续小陀螺。
            alignment_rotation = self.aligning and abs(self.decision.wz) <= 60 and abs(actual_wz) <= math.radians(60)+1e-6
            if terrain_rotation and alignment_rotation: self.terrain_alignment_seconds += dt
            terrain_spin = terrain_rotation and not alignment_rotation
            if terrain_spin:
                self.terrain_spin_seconds += dt
                if not self.in_terrain_spin:
                    self.terrain_spin_events.append({'time':self.elapsed,'x':self.x,'y':self.y,'wz_degps':math.degrees(actual_wz)})
            self.in_terrain_spin = terrain_spin
        # 位姿差分与瞬时速度都来自物理模型；减速时也保留合理的提前禁转。
        pose_v = world_v
        pose_dt = self.clock_time-self.last_terrain_stamp
        if self.last_terrain_position is not None and 0 < pose_dt < 0.5:
            pose_v = ((self.x-self.last_terrain_position[0])/pose_dt,(self.y-self.last_terrain_position[1])/pose_dt)
        if self.clock_time > self.last_terrain_stamp:
            self.last_terrain_position = (self.x,self.y)
            self.last_terrain_stamp = self.clock_time
        if not self.terrain.spin_allowed(self.x,self.y,world_v[0],world_v[1]) or not self.terrain.spin_allowed(self.x,self.y,pose_v[0],pose_v[1]):
            self.last_terrain_unsafe = self.clock_time
        exception = self.clock_time-self.last_terrain_unsafe < self.spin_resume_delay+0.1
        if self.start_time is not None and self.elapsed < 300: self.judge.update(self.elapsed,self.x,self.y,speed,actual_wz,self.posture,exception,self.ground(self.x,self.y))
        if len(self.judge.events) != self.last_event_count:
            self.last_event_count = len(self.judge.events)
            self.get_logger().info('SCORE '+json.dumps(self.judge.events[-1],ensure_ascii=False))
        if self.elapsed >= 300 and not self.reported:
            self.reported = True
            result = {'physics':self.physics,'max_speed_mps':4.5,'max_actual_speed_mps':self.max_actual_speed,'max_command_speed_mps':self.max_command_speed,'terrain_alignment_seconds':self.terrain_alignment_seconds,'terrain_spin_seconds':self.terrain_spin_seconds,'terrain_spin_events':self.terrain_spin_events,'score':self.judge.score,'spin_violations':self.judge.spin_violations,'events':self.judge.events,'distance':round(self.distance,2),'collision_samples':self.collisions,'final_pose':[self.x,self.y],'scoring_posture_seconds':self.judge.scoring_time}
            data = json.dumps(result,ensure_ascii=False,indent=2)
            self.get_logger().info('RESULT '+data)
            msg = String();msg.data = data;self.score_puber.publish(msg)
            if self.output_file:
                result['trajectory_columns'] = ['elapsed_s','x_m','y_m','yaw_rad']
                result['trajectory'] = self.trajectory
                with open(self.output_file,'w') as f: json.dump(result,f,ensure_ascii=False,indent=2);f.write('\n')

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = SimulatorNode()
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
