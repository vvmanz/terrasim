#!/usr/bin/env python3
"""ROS 2 node: streams a running terrasim job to ROS 2 topics.

terrasim writes everything it computes (pose, encoder odometry, IMU, and per wheel: speed set point, encoder speed,
motor current, voltage, torque, normal force, traction, slip, sinkage, surface) to <run>/log.csv. This node follows
that file while the simulation runs and republishes it, so the terrasim-specific signals that native Gazebo sensors do
not have (motor currents, slip, sinkage) are available in ROS 2 for any engine (Gazebo Harmonic or the offline core).
Start the run with --ros (scripts/drive.sh ... --ros): the log is then flushed row by row.

Topics (namespace /terrasim by default):
  ~/clock_source   -- (with clock:=true) /clock from the simulation time of the log (offline core has no gz clock)
  ~/ground_truth   nav_msgs/Odometry   pose and body velocity of the simulated robot (map frame)
  ~/odom           nav_msgs/Odometry   encoder odometry computed in the robot model
  ~/imu            sensor_msgs/Imu     IMU of the model (with its noise model)
  ~/wheels         sensor_msgs/JointState  name = wheel labels, velocity = encoder speed (rad/s), effort = motor torque at the wheel (N m)
  ~/wheel_current  std_msgs/Float64MultiArray  motor current per wheel / side (A)
  ~/wheel_slip     std_msgs/Float64MultiArray  longitudinal slip per wheel (-)
  ~/wheel_sinkage  std_msgs/Float64MultiArray  sinkage per wheel (m)
  ~/wheel_force    std_msgs/Float64MultiArray  [N, traction H, free-rolling force] per wheel (N)
  ~/body_contact   std_msgs/Float64MultiArray  [tilt of the chassis to the ground normal (deg), body normal force (N),
                                                body points touching the ground]

    ros2 run --prefix 'python3' ... or simply:
    python3 ros2/terrasim_bridge.py --ros-args -p log:=/path/to/run/log.csv -p clock:=true
    (scripts/drive.sh --ros starts it automatically next to the simulation)
"""
import math, os, time

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from rosgraph_msgs.msg import Clock

WHEEL_KEYS = ('wset', 'wenc', 'cur', 'vol', 'pow', 'tau', 'N', 'Fx', 'Fy', 'slip', 'slip_lat', 'sink', 'surf')


def stamp(t):
    s = int(math.floor(t)); return Time(sec=s, nanosec=int((t - s) * 1e9))


def quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2); cp, sp = math.cos(pitch / 2), math.sin(pitch / 2); cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


class LogFollower:
    """incremental reader of a CSV that another process appends to (pure Python, testable without ROS)"""
    def __init__(self, path):
        self.path, self.f, self.cols, self.buf = path, None, None, ''

    def poll(self):
        if self.f is None:
            if not os.path.exists(self.path): return []
            self.f = open(self.path, 'r')
        chunk = self.f.read()
        if not chunk: return []
        self.buf += chunk; lines = self.buf.split('\n'); self.buf = lines.pop()       # keep an unfinished line
        rows = []
        for ln in lines:
            if not ln.strip(): continue
            parts = ln.split(',')
            if self.cols is None: self.cols = parts; continue
            if len(parts) != len(self.cols): continue
            row = {}
            for k, v in zip(self.cols, parts):
                try: row[k] = float(v)
                except ValueError: row[k] = v                                        # surface names
            rows.append(row)
        return rows

    def wheels(self):
        n = 0
        while self.cols and f'w{n + 1}_cur' in self.cols: n += 1
        return n


class TerrasimBridge(Node):
    def __init__(self):
        super().__init__('terrasim_bridge')
        self.declare_parameter('log', ''); self.declare_parameter('clock', False); self.declare_parameter('frame', 'map')
        self.declare_parameter('base_frame', 'base_link'); self.declare_parameter('wheel_labels', ''); self.declare_parameter('period', 0.01)
        path = self.get_parameter('log').value
        if not path: raise SystemExit('terrasim_bridge: parameter log:=<run>/log.csv is required')
        self.fol = LogFollower(path); self.frame = self.get_parameter('frame').value; self.base = self.get_parameter('base_frame').value
        labels = self.get_parameter('wheel_labels').value; self.labels = [s for s in labels.split(',') if s] if labels else None
        ns = '/terrasim'
        self.p_gt = self.create_publisher(Odometry, f'{ns}/ground_truth', 10); self.p_od = self.create_publisher(Odometry, f'{ns}/odom', 10)
        self.p_imu = self.create_publisher(Imu, f'{ns}/imu', 10); self.p_wh = self.create_publisher(JointState, f'{ns}/wheels', 10)
        self.p_cur = self.create_publisher(Float64MultiArray, f'{ns}/wheel_current', 10)
        self.p_slip = self.create_publisher(Float64MultiArray, f'{ns}/wheel_slip', 10)
        self.p_sink = self.create_publisher(Float64MultiArray, f'{ns}/wheel_sinkage', 10)
        self.p_force = self.create_publisher(Float64MultiArray, f'{ns}/wheel_force', 10)
        self.p_body = self.create_publisher(Float64MultiArray, f'{ns}/body_contact', 10)
        self.p_clk = self.create_publisher(Clock, '/clock', 10) if self.get_parameter('clock').value else None
        self.create_timer(float(self.get_parameter('period').value), self.tick)
        self.get_logger().info(f'following {path}')

    def arr(self, vals, label):
        m = Float64MultiArray(); m.layout.dim = [MultiArrayDimension(label=label, size=len(vals), stride=len(vals))]; m.data = [float(v) for v in vals]
        return m

    def tick(self):
        for r in self.fol.poll():
            t = r['t']; st = stamp(t); n = self.fol.wheels(); names = self.labels or [f'wheel{i + 1}' for i in range(n)]
            if self.p_clk: self.p_clk.publish(Clock(clock=st))
            gt = Odometry(); gt.header.stamp = st; gt.header.frame_id = self.frame; gt.child_frame_id = self.base
            gt.pose.pose.position.x, gt.pose.pose.position.y, gt.pose.pose.position.z = r['x'], r['y'], r['z']
            q = quat(r['roll'], r['pitch'], r['yaw'])
            gt.pose.pose.orientation.x, gt.pose.pose.orientation.y, gt.pose.pose.orientation.z, gt.pose.pose.orientation.w = q
            gt.twist.twist.linear.x, gt.twist.twist.linear.y, gt.twist.twist.angular.z = r['vx'], r['vy'], r['wz']   # body frame (as logged)
            self.p_gt.publish(gt)
            od = Odometry(); od.header.stamp = st; od.header.frame_id = 'odom'; od.child_frame_id = self.base
            od.pose.pose.position.x, od.pose.pose.position.y = r['odo_x'], r['odo_y']
            q = quat(0.0, 0.0, r['odo_yaw']); od.pose.pose.orientation.x, od.pose.pose.orientation.y, od.pose.pose.orientation.z, od.pose.pose.orientation.w = q
            self.p_od.publish(od)
            im = Imu(); im.header.stamp = st; im.header.frame_id = self.base
            im.angular_velocity.x, im.angular_velocity.y, im.angular_velocity.z = r['imu_gx'], r['imu_gy'], r['imu_gz']
            im.linear_acceleration.x, im.linear_acceleration.y, im.linear_acceleration.z = r['imu_ax'], r['imu_ay'], r['imu_az']
            im.orientation_covariance[0] = -1.0                                   # orientation not provided
            self.p_imu.publish(im)
            js = JointState(); js.header.stamp = st; js.name = names
            js.velocity = [r[f'w{i + 1}_wenc'] for i in range(n)]; js.effort = [r[f'w{i + 1}_tau'] for i in range(n)]
            self.p_wh.publish(js)
            self.p_cur.publish(self.arr([r[f'w{i + 1}_cur'] for i in range(n)], 'current_A'))
            self.p_slip.publish(self.arr([r[f'w{i + 1}_slip'] for i in range(n)], 'slip'))
            self.p_sink.publish(self.arr([r[f'w{i + 1}_sink'] for i in range(n)], 'sinkage_m'))
            self.p_force.publish(self.arr(sum(([r[f'w{i + 1}_N'], r[f'w{i + 1}_Fx'], r[f'w{i + 1}_Fy']] for i in range(n)), []), 'N_Fx_Fy_per_wheel'))
            if 'body_N' in r: self.p_body.publish(self.arr([r['tilt_rel'], r['body_N'], r['body_pts']], 'tilt_deg_bodyN_points'))


def main():
    rclpy.init(); node = TerrasimBridge()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    main()
