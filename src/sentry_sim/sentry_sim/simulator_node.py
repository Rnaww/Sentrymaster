import rclpy, math, time, json, os
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from rm_msgs.msg import NavigationMcuCmd, DecisionMcuCmd, RefereeData, ChassisData
from PIL import Image
from .scoring import Scoring

# 与原 URDF 的 odin1_base_link -> vehicle 完全互逆。
FIXED_TRANSLATION = (0.16797, -0.17893, -0.00451)
FIXED_ROTATION = (-math.sqrt(0.5), math.sqrt(0.5), 0, 0)

def multiply(a, b):
    x,y,z,w = a
    X,Y,Z,W = b
    return (w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X, w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z)

def rotate(q, p):
    value = multiply(multiply(q, (*p,0)), (-q[0],-q[1],-q[2],q[3]))
    return value[:3]

class SimulatorNode(Node):
    def __init__(self):
        super().__init__('sentry_simulator')
        self.physics = self.declare_parameter('physics', 'kinematic').value
        self.factor = float(self.declare_parameter('realtime_factor', 1.0).value)
        self.output_file = self.declare_parameter('output_file', '').value
        self.x = float(self.declare_parameter('robot_x', 7.25).value)
        self.y = float(self.declare_parameter('robot_y', 2.05).value)
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
        self.last_event_count = 0
        self.last_wall = time.monotonic()
        self.max_acc = 1.5
        share = get_package_share_directory('sentry_planner')
        self.height = Image.open(os.path.join(share, 'map', '701_model', 'new_701_deep.png')).convert('L')
        self.occupancy = Image.open(os.path.join(share, 'map', '701_model', '701_navigation.png')).convert('L')
        self.nav_suber = self.create_subscription(NavigationMcuCmd, '/navigation2mcu', self.nav_callback, 10)
        self.decision_suber = self.create_subscription(DecisionMcuCmd, '/decision2mcu', self.decision_callback, 10)
        self.gazebo_suber = self.create_subscription(Odometry, '/sentry_robot/odom', self.gazebo_callback, 10)
        self.clock_puber = self.create_publisher(Clock, '/clock', 10)
        self.odom_puber = self.create_publisher(Odometry, '/odin1/odometry_highfreq', 10)
        self.referee_puber = self.create_publisher(RefereeData, '/referee_data', 10)
        self.chassis_puber = self.create_publisher(ChassisData, '/chassis_data', 10)
        self.gazebo_puber = self.create_publisher(Twist, '/sentry_robot/cmd_vel', 10)
        self.score_puber = self.create_publisher(String, '/simulation_score', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.get_logger().info(f'SIM YAML: physics={self.physics}, realtime_factor={self.factor}, pose=({self.x},{self.y},{self.yaw})')

    def nav_callback(self, msg):
        self.nav = msg
        self.nav_time = time.monotonic()

    def decision_callback(self, msg):
        self.decision = msg
        self.decision_time = time.monotonic()

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
        if norm > 3.5: vx,vy = vx*3.5/norm,vy*3.5/norm
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
        vehicle_q = (0,0,math.sin(self.yaw/2),math.cos(self.yaw/2))
        vehicle_z = self.ground(self.x,self.y)
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
        self.odom_puber.publish(odom)
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
        exception = (14.2 < self.x < 16.2 and 1.8 < self.y < 6.8) or any(abs(self.ground(self.x+dx,self.y+dy)-h) > 0.02 for dx,dy in [(0.3,0),(-0.3,0),(0,0.3),(0,-0.3)])
        if self.start_time is not None and self.elapsed < 300: self.judge.update(self.elapsed,self.x,self.y,speed,actual_wz,self.posture,exception)
        if len(self.judge.events) != self.last_event_count:
            self.last_event_count = len(self.judge.events)
            self.get_logger().info('SCORE '+json.dumps(self.judge.events[-1],ensure_ascii=False))
        if self.elapsed >= 300 and not self.reported:
            self.reported = True
            result = {'physics':self.physics,'score':self.judge.score,'events':self.judge.events,'distance':round(self.distance,2),'collision_samples':self.collisions,'final_pose':[self.x,self.y],'scoring_posture_seconds':self.judge.scoring_time}
            data = json.dumps(result,ensure_ascii=False,indent=2)
            self.get_logger().info('RESULT '+data)
            msg = String();msg.data = data;self.score_puber.publish(msg)
            if self.output_file:
                with open(self.output_file,'w') as f: f.write(data+'\n')

def main(args=None):
    rclpy.init(args=args)
    node = SimulatorNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
