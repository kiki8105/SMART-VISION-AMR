import json,os,time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy,DurabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray,String
from rosgraph_msgs.msg import Clock
assert os.environ['ROS_DOMAIN_ID']=='186'
rclpy.init()
n=Node('local05_passive_forensics',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
start=time.monotonic(); last_clock=0.
def emit(kind,**data):
 print(json.dumps({'kind':kind,'wall_monotonic_ns':time.monotonic_ns(),'elapsed':time.monotonic()-start,'ros_ns':n.get_clock().now().nanoseconds,**data},separators=(',',':')),flush=True)
def scan(m,kind):
 emit(kind,stamp_ns=m.header.stamp.sec*1000000000+m.header.stamp.nanosec,frame=m.header.frame_id,count=len(m.ranges))
def clock(m):
 global last_clock
 if time.monotonic()-last_clock>.2:
  emit('clock',stamp_ns=m.clock.sec*1000000000+m.clock.nanosec);last_clock=time.monotonic()
n.create_subscription(LaserScan,'/startup_validation/raw_scan',lambda m:scan(m,'raw_scan'),qos_profile_sensor_data)
n.create_subscription(LaserScan,'/scan',lambda m:scan(m,'forward_scan'),qos_profile_sensor_data)
n.create_subscription(Int32MultiArray,'/smart_vision_amr/observed_obstacles',lambda m:emit('observed',indices=list(m.data)),10)
durable=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(String,'/smart_vision_amr/scan_projection_status',lambda m:emit('projection',payload=json.loads(m.data)),durable)
n.create_subscription(Clock,'/clock',clock,qos_profile_sensor_data)
emit('started',domain=186,duration_limit_seconds=60)
try:
 while time.monotonic()-start<60:
  rclpy.spin_once(n,timeout_sec=.05)
finally:
 emit('finished');n.destroy_node();rclpy.shutdown()
