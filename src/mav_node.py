#!/usr/bin/env python3
from enum import Enum

import numpy as np
from numpy import deg2rad
from numpy import rad2deg
from numpy import cos
from numpy import sin
import matplotlib.pylab as plt

import proto
from proto import euler2quat
from proto import euler321
from proto import rot2quat
from proto import quat_mul
from proto import quat_normalize
from proto import wrap_pi
from proto import PID

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import DurabilityPolicy
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition as LocalPosition
from px4_msgs.msg import VehicleAttitudeSetpoint
from px4_msgs.msg import VehicleStatus


class MavVelocityControl:
  def __init__(self):
    self.period = 0.0055  # [s]
    self.dt = 0
    self.pid_vx = PID(1.0, 0.0, 0.05)
    self.pid_vy = PID(1.0, 0.0, 0.05)
    self.pid_vz = PID(1.0, 0.0, 0.05)
    self.hover_thrust = 0.7
    self.u = [0.0, 0.0, 0.0, 0.0]  # roll, pitch, yaw, thrust

  def update(self, sp, pv, dt):
    """ Update """
    # Check rate
    self.dt += dt
    if self.dt < self.period:
      return self.u  # Return previous command

    # Transform errors in world frame to mav frame
    errors_W = np.array([sp[0] - pv[0], sp[1] - pv[1], sp[2] - pv[2]])
    C_WB = proto.euler321(pv[3], 0.0, 0.0)
    errors = C_WB.T @ errors_W

    # Roll, pitch, yaw and thrust
    r = -self.pid_vy.update(errors[1], 0.0, dt)
    p = self.pid_vx.update(errors[0], 0.0, dt)
    y = sp[3]
    t = self.hover_thrust + self.pid_vz.update(errors[2], 0.0, dt)

    # Clip values
    self.u[0] = proto.clip_value(r, deg2rad(-20.0), deg2rad(20.0))
    self.u[1] = proto.clip_value(p, deg2rad(-20.0), deg2rad(20.0))
    self.u[2] = y
    self.u[3] = proto.clip_value(t, 0.0, 1.0)

    # Reset dt
    self.dt = 0.0

    return self.u

  def reset(self):
    """ Reset """
    self.dt = 0.0
    self.pid_vx.reset()
    self.pid_vy.reset()
    self.pid_vz.reset()
    self.u = [0.0, 0.0, 0.0, 0.0]


class MavPositionControl:
  def __init__(self):
    self.period = 0.0055
    self.vx_min = -5.0
    self.vx_max = 5.0
    self.vy_min = -5.0
    self.vy_max = 5.0
    self.vz_min = -5.0
    self.vz_max = 5.0

    self.dt = 0
    self.pid_x = PID(0.5, 0.0, 0.05)
    self.pid_y = PID(0.5, 0.0, 0.05)
    self.pid_z = PID(0.5, 0.0, 0.05)
    self.u = [0.0, 0.0, 0.0, 0.0]

  def update(self, sp, pv, dt):
    """ Update """
    # Check rate
    self.dt += dt
    if self.dt < self.period:
      return self.u  # Return previous command

    # Calculate velocity errors in world frame
    errors = np.array([sp[0] - pv[0], sp[1] - pv[1], sp[2] - pv[2]])

    # Velocity commands
    vx = self.pid_x.update(errors[0], 0.0, self.dt)
    vy = self.pid_y.update(errors[1], 0.0, self.dt)
    vz = self.pid_z.update(errors[2], 0.0, self.dt)
    yaw = sp[3]

    self.u[0] = proto.clip_value(vx, self.vx_min, self.vx_max)
    self.u[1] = proto.clip_value(vy, self.vy_min, self.vy_max)
    self.u[2] = proto.clip_value(vz, self.vz_min, self.vz_max)
    self.u[3] = yaw

    # Reset dt
    self.dt = 0.0

    return self.u

  def reset(self):
    """ Reset """
    self.dt = 0.0
    self.pid_vx.reset()
    self.pid_vy.reset()
    self.pid_vz.reset()
    self.u = [0.0, 0.0, 0.0, 0.0]


class MavTrajectoryControl:
  def __init__(self, **kwargs):
    self.A = kwargs.get("A", 2.0)
    self.B = kwargs.get("B", 2.0)
    self.a = kwargs.get("a", 1.0)
    self.b = kwargs.get("b", 2.0)
    self.z = kwargs["z"]
    self.T = kwargs["T"]
    self.f = 1.0 / self.T
    self.delta = kwargs.get("delta", np.pi)

  def get_traj(self):
    """ Return trajectory """
    pos_data = np.zeros((3, 1000))
    time = np.linspace(0.0, self.T, 1000)
    for i, t in enumerate(time):
      pos_data[:, i] = self.get_position(t).T
    return pos_data.T

  def get_position(self, t):
    """ Get position """
    x = self.A * np.sin(self.a * 2.0 * np.pi * self.f * t + self.delta)
    y = self.B * np.sin(self.b * 2.0 * np.pi * self.f * t)
    z = self.z
    return np.array([x, y, z])

  def get_yaw(self, t):
    """ Get yaw """
    p0 = self.get_position(t)
    p1 = self.get_position(t + 0.1)
    dx, dy, dz = p1 - p0

    heading = np.arctan2(dy, dx)
    if heading > np.pi:
      heading -= 2.0 * np.pi
    elif heading < -np.pi:
      heading += 2.0 * np.pi

    return heading

  def get_velocity(self, t):
    ka = 2.0 * np.pi * self.a * self.f
    kb = 2.0 * np.pi * self.b * self.f
    vx = ka * self.A * np.cos(ka * t + self.delta)
    vy = kb * self.B * np.cos(kb * t)
    vz = 0.0
    return np.array([vx, vy, vz])

  def plot_traj(self):
    """ Plot """
    pos_data = np.zeros((3, 1000))
    vel_data = np.zeros((3, 1000))
    time = np.linspace(0.0, self.T, 1000)
    for i, t in enumerate(time):
      pos_data[:, i] = self.get_position(t).T
      vel_data[:, i] = self.get_velocity(t).T

    plt.subplot(311)
    plt.plot(pos_data[0, :], pos_data[1, :])
    plt.axis("equal")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")

    plt.subplot(312)
    plt.plot(time, pos_data[0, :], "r-", label="Position - x")
    plt.plot(time, pos_data[1, :], "g-", label="Position - y")
    plt.plot(time, pos_data[2, :], "b-", label="Position - z")
    plt.xlabel("Time [s]")
    plt.ylabel("Positions [m]")
    plt.legend(loc=0)

    plt.subplot(313)
    plt.plot(time, vel_data[0, :], "r-", label="Velocity - x")
    plt.plot(time, vel_data[1, :], "g-", label="Velocity - y")
    plt.plot(time, vel_data[2, :], "b-", label="Velocity - z")
    plt.xlabel("Time [s]")
    plt.ylabel("Velocity [ms^-1]")
    plt.legend(loc=0)

    plt.show()


class MavMode(Enum):
  GO_TO_START = 1
  EXEC_TRAJECTORY = 2
  HOVER = 3
  LAND = 4


class MavNode(Node):
  """Node for controlling a vehicle in offboard mode."""
  def __init__(self):
    super().__init__('mav_node')
    self.is_running = True
    topic_control = "/fmu/in/offboard_control_mode"
    topic_traj = "/fmu/in/trajectory_setpoint"
    topic_att = "/fmu/in/vehicle_attitude_setpoint"
    topic_cmd = "/fmu/in/vehicle_command"
    topic_pos = "/fmu/out/vehicle_local_position"
    topic_status = "/fmu/out/vehicle_status"
    topic_pos_sp = "/mav/in/position_setpoint"
    topic_yaw_sp = "/mav/in/yaw_setpoint"

    # Configure QoS profile for publishing and subscribing
    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST,
                     depth=1)

    # Create publishers
    pub_init = self.create_publisher
    self.pub_offboard = pub_init(OffboardControlMode, topic_control, qos)
    self.pub_traj = pub_init(TrajectorySetpoint, topic_traj, qos)
    self.pub_att = pub_init(VehicleAttitudeSetpoint, topic_att, qos)
    self.pub_cmd = pub_init(VehicleCommand, topic_cmd, qos)

    # Create subscribers
    sub_init = self.create_subscription
    self.sub_pos = sub_init(LocalPosition, topic_pos, self.pos_cb, qos)
    self.sub_pos_sp = sub_init(Vector3, topic_pos_sp, self.pos_sp_cb, qos)
    self.sub_yaw_sp = sub_init(Float32, topic_yaw_sp, self.yaw_sp_cb, qos)
    self.sub_status = sub_init(VehicleStatus, topic_status, self.status_cb, qos)

    # State
    self.vehicle_local_position = LocalPosition()
    self.vehicle_status = VehicleStatus()
    self.mode = MavMode.GO_TO_START
    self.ts = None
    self.pos = None
    self.vel = None
    self.heading = 0.0
    self.traj_start = None
    self.hover_start = None

    # Settings
    self.takeoff_height = 2.0
    self.traj_period = 15.0
    self.hover_for = 3.0

    # Control
    self.pos_ctrl = MavPositionControl()
    self.vel_ctrl = MavVelocityControl()
    self.traj_ctrl = MavTrajectoryControl(z=self.takeoff_height,
                                          T=self.traj_period)

    x0, y0, z0 = self.traj_ctrl.get_position(0.0)
    self.yaw_sp = self.traj_ctrl.get_yaw(0.0)
    self.pos_sp = [x0, y0, z0, self.yaw_sp]
    # self.yaw_sp = 0.0
    # self.pos_sp = [0.0, 0.0, 2.0, 0.0]

    # Create a timer to publish control commands
    self.dt = 0.005
    self.timer = self.create_timer(self.dt, self.timer_cb)

    # Engage offboard and arm MAV
    self.engage_offboard_mode()
    self.arm()

  def pos_cb(self, msg):
    """Callback function for msg topic subscriber."""
    self.vehicle_local_position = msg

    self.ts = int(msg.timestamp * 1e3)

    pos_x = msg.y
    pos_y = msg.x
    pos_z = -1.0 * msg.z
    self.pos = np.array([pos_x, pos_y, pos_z])

    vel_x = msg.vy
    vel_y = msg.vx
    vel_z = -1.0 * msg.vz
    self.vel = np.array([vel_x, vel_y, vel_z])

    self.heading = -msg.heading + deg2rad(90.0)
    if self.heading > np.pi:
      self.heading -= 2.0 * np.pi
    elif self.heading < -np.pi:
      self.heading += 2.0 * np.pi

  def pos_sp_cb(self, msg):
    """Callback function for position setpoint topic subscriber."""
    self.pos_sp = [msg.x, msg.y, msg.z, self.yaw_sp]

  def yaw_sp_cb(self, msg):
    """Callback function for yaw setpoint topic subscriber."""
    self.yaw_sp = msg.data
    self.pos_sp[3] = self.yaw_sp

  def status_cb(self, vehicle_status):
    """Callback function for vehicle_status topic subscriber."""
    self.vehicle_status = vehicle_status

  def arm(self):
    """Send an arm command to the vehicle."""
    self.cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
    self.get_logger().info('Arm command sent')

  def disarm(self):
    """Send a disarm command to the vehicle."""
    self.cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
    self.get_logger().info('Disarm command sent')

  def engage_offboard_mode(self):
    """Switch to offboard mode."""
    self.cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
    self.get_logger().info("Switching to offboard mode")

  def land(self):
    """Switch to land mode."""
    self.cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
    self.get_logger().info("Landing!")

  def pub_heart_beat(self):
    """Publish the offboard control mode."""
    msg = OffboardControlMode()
    msg.position = False
    msg.velocity = False
    msg.acceleration = False
    msg.attitude = True
    msg.body_rate = False
    msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
    self.pub_offboard.publish(msg)

  def pub_position_sp(self, x: float, y: float, z: float):
    """Publish the trajectory setpoint."""
    msg = TrajectorySetpoint()
    msg.position = [y, x, -z]
    msg.yaw = 1.57079  # (90 degree)
    msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
    self.pub_traj.publish(msg)

  def pub_attitude_sp(self, roll, pitch, yaw, thrust):
    yaw_sp = deg2rad(90.0) - yaw
    if yaw_sp > np.pi:
      yaw_sp -= 2.0 * np.pi
    elif yaw_sp < -np.pi:
      yaw_sp += 2.0 * np.pi

    qw, qx, qy, qz = quat_normalize(euler2quat(yaw_sp, -pitch, roll)).tolist()
    msg = VehicleAttitudeSetpoint()
    msg.q_d = [qw, qx, qy, qz]
    msg.thrust_body = [0.0, 0.0, -thrust]
    self.pub_att.publish(msg)

  def cmd(self, command, **params):
    """Publish a vehicle command."""
    msg = VehicleCommand()
    msg.command = command
    msg.param1 = params.get("param1", 0.0)
    msg.param2 = params.get("param2", 0.0)
    msg.param3 = params.get("param3", 0.0)
    msg.param4 = params.get("param4", 0.0)
    msg.param5 = params.get("param5", 0.0)
    msg.param6 = params.get("param6", 0.0)
    msg.param7 = params.get("param7", 0.0)
    msg.target_system = 1
    msg.target_component = 1
    msg.source_system = 1
    msg.source_component = 1
    msg.from_external = True
    msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
    self.pub_cmd.publish(msg)

  def stop_timer(self):
    self.get_logger().info('Stopping timer!')
    self.timer.cancel()
    self.destroy_timer(self.timer)

  def timer_cb(self):
    """Callback function for the timer."""
    self.pub_heart_beat()
    if self.pos is None or self.vel is None:
      return

    # Process variables
    pos_pv = [self.pos[0], self.pos[1], self.pos[2], self.heading]
    vel_pv = [self.vel[0], self.vel[1], self.vel[2], self.heading]

    # GO TO START
    if self.mode == MavMode.GO_TO_START:
      # Position controller
      vel_sp = self.pos_ctrl.update(self.pos_sp, pos_pv, self.dt)
      u = self.vel_ctrl.update(vel_sp, vel_pv, self.dt)
      self.pub_attitude_sp(u[0], u[1], u[2], u[3])

      # Transition to trajectory mode?
      dx = self.pos[0] - self.pos_sp[0]
      dy = self.pos[1] - self.pos_sp[1]
      dz = self.pos[2] - self.pos_sp[2]
      dpos = np.sqrt(dx * dx + dy * dy + dz * dz)
      dyaw = np.fabs(self.heading - self.yaw_sp)
      if dpos < 0.1 and dyaw < np.deg2rad(10.0):
        self.mode = MavMode.EXEC_TRAJECTORY

    # EXECUTE TRAJECTORY
    elif self.mode == MavMode.EXEC_TRAJECTORY:
      # Run trajectory
      if self.traj_start is None:
        self.traj_start = self.ts

      t = float(self.ts - self.traj_start) * 1e-9
      vel_traj = self.traj_ctrl.get_velocity(t)
      yaw_traj = self.traj_ctrl.get_yaw(t)
      vel_sp = [vel_traj[0], vel_traj[1], vel_traj[2], yaw_traj]
      u = self.vel_ctrl.update(vel_sp, vel_pv, self.dt)
      self.pub_attitude_sp(u[0], u[1], u[2], u[3])
      if t >= self.traj_period:
        self.mode = MavMode.HOVER
        self.traj_start = None

    # HOVER
    elif self.mode == MavMode.HOVER:
      # Position controller
      if self.hover_start is None:
        self.hover_start = self.ts

      pos = self.traj_ctrl.get_position(self.traj_period)
      yaw = self.traj_ctrl.get_yaw(self.traj_period)
      self.pos_sp = [pos[0], pos[1], pos[2], yaw]
      vel_sp = self.pos_ctrl.update(self.pos_sp, pos_pv, self.dt)
      u = self.vel_ctrl.update(vel_sp, vel_pv, self.dt)
      self.pub_attitude_sp(u[0], u[1], u[2], u[3])

      hover_time = float(self.ts - self.hover_start) * 1e-9
      if hover_time > self.hover_for:
        self.hover_start = None
        self.mode = MavMode.LAND

    # LAND
    elif self.mode == MavMode.LAND:
      if self.vehicle_status.arming_state == 1:
        self.stop_node()
        return

      if self.vehicle_status.nav_state != 18:
        self.land()

  def stop_node(self):
    """ Stop Node """
    self.get_logger().info('Stopping the node')
    self.timer.cancel()
    self.destroy_timer(self.timer)

    self.sub_pos.destroy()
    self.sub_pos_sp.destroy()
    self.sub_yaw_sp.destroy()
    self.sub_status.destroy()

    self.get_logger().info('Destroying the node')
    self.destroy_node()
    self.is_running = False


if __name__ == '__main__':
  rclpy.init()
  node = MavNode()
  while node.is_running:
    rclpy.spin_once(node)
  rclpy.shutdown()
