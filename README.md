# onnxruntime gpu : https://pypi.jetson-ai-lab.io/jp6/cu126/

# regle udev ok pour problemes ttyDmx

# 1. Récupérer le numéro d'interface depuis le parent USB
SUBSYSTEMS=="usb", ACTION=="add", ENV{ID_USB_INTERFACE_NUM}="$attr{bInterfaceNumber}"

# 2. ttyQboard2 → interface 00 du FT4232H (0403:6010)
SUBSYSTEM=="tty", SUBSYSTEMS=="usb", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6010", ENV{ID_USB_INTERFACE_NUM}=="00", SYMLINK+="ttyQboard2", MODE="0666", GROUP="dialout"

# 3. ttyDmx → interface 01 du même FT4232H
SUBSYSTEM=="tty", SUBSYSTEMS=="usb", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6010", ENV{ID_USB_INTERFACE_NUM}=="01", SYMLINK+="ttyDmx", MODE="0666", GROUP="dialout"







## rebuild propre

cd ~/qbo_ws
rm -rf build install log
colcon build
source install/setup.bash


## regler odometrie et imu !

imu/ rotations : 

reinitialiser ecran : ros2 service call /qbo_arduqbo/base_ctrl/set_odometry qbo_msgs/srv/SetOdometry "{x: 0, y: 0, theta: 0}"

reinitialiser imu : ros2 service call /qbo_arduqbo/imu_ctrl/imu_state/calibrate qbo_msgs/srv/CalibrateIMU "{}"


## tester rotations sur 720 degres :

python3 -c "import rclpy,math;from rclpy.node import Node;from geometry_msgs.msg import Twist;from nav_msgs.msg import Odometry; 
class N(Node):
  def __init__(self):
    super().__init__('rot720');self.pub=self.create_publisher(Twist,'/qbo_arduqbo/base_ctrl/cmd_vel',10);self.sub=self.create_subscription(Odometry,'/qbo_arduqbo/base_ctrl/odom',self.cb,10);self.prev=None;self.total=0
  def yaw(self,q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
  def cb(self,m):
    y=self.yaw(m.pose.pose.orientation)
    if self.prev is None:self.prev=y
    d=y-self.prev
    if d>math.pi:d-=2*math.pi
    if d<-math.pi:d+=2*math.pi
    self.total+=abs(d);self.prev=y
    t=Twist()
    if self.total<4*math.pi:t.angular.z=0.8
    else:t.angular.z=0.0;self.pub.publish(t);rclpy.shutdown();return
    self.pub.publish(t)
rclpy.init();n=N();rclpy.spin(n)"


## tester odom sur 3 Mètres:

python3 -c "import rclpy,math;from rclpy.node import Node;from geometry_msgs.msg import Twist;from nav_msgs.msg import Odometry;
class N(Node):
  def __init__(s):
    super().__init__('rot720');s.pub=s.create_publisher(Twist,'/qbo_arduqbo/base_ctrl/cmd_vel',10);s.sub=s.create_subscription(Odometry,'/qbo_arduqbo/base_ctrl/odom',s.cb,10);s.a0=None;s.t=0
  def yaw(s,q):
    import math
    siny=2*(q.w*q.z+q.x*q.y)
    cosy=1-2*(q.y*q.y+q.z*q.z)
    return math.atan2(siny,cosy)
  def cb(s,m):
    y=s.yaw(m.pose.pose.orientation)
    if s.a0 is None: s.a0=y
    d=y-s.a0
    if d>math.pi: d-=2*math.pi
    if d<-math.pi: d+=2*math.pi
    s.t+=abs(d); s.a0=y
    msg=Twist()
    if s.t<4*math.pi: msg.angular.z=0.4
    else: msg.angular.z=0.0; s.pub.publish(msg); rclpy.shutdown(); return
    s.pub.publish(msg)
rclpy.init();n=N();rclpy.spin(n)"
