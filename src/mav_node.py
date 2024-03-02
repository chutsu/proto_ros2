#!/usr/bin/env python3
import argparse
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
from proto import KalmanFilter

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import DurabilityPolicy
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition as LocalPosition
from px4_msgs.msg import VehicleAttitudeSetpoint
from px4_msgs.msg import VehicleStatus


class MavVelocityControl:
  def __init__(self):
    self.period = 0.0049  # [s]
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
  def __init__(self, output_mode="VELOCITY"):
    self.output_mode = output_mode
    self.dt = 0
    self.u = [0.0, 0.0, 0.0, 0.0]

    if self.output_mode == "VELOCITY":
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

    elif self.output_mode == "ATTITUDE":
      self.period = 0.0049
      self.roll_min = deg2rad(-35.0)
      self.roll_max = deg2rad(35.0)
      self.pitch_min = deg2rad(-35.0)
      self.pitch_max = deg2rad(35.0)
      self.hover_thrust = 0.3

      self.pid_x = PID(1.0, 0.0, 0.1)
      self.pid_y = PID(1.0, 0.0, 0.1)
      self.pid_z = PID(0.1, 0.0, 0.0)

    else:
      raise NotImplementedError()

  def update(self, sp, pv, dt):
    """ Update """
    # Check rate
    self.dt += dt
    if self.dt < self.period:
      return self.u  # Return previous command

    if self.output_mode == "VELOCITY":
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

    elif self.output_mode == "ATTITUDE":
      # Calculate position errors in mav frame
      errors = euler321(pv[3], 0.0, 0.0).T @ (sp[0:3] - pv[0:3])

      # Attitude commands
      roll = -self.pid_y.update(errors[1], 0.0, dt)
      pitch = self.pid_x.update(errors[0], 0.0, dt)
      thrust = self.pid_z.update(errors[2], 0.0, dt)

      # Attitude command (roll, pitch, yaw, thrust)
      self.u[0] = proto.clip_value(roll, self.roll_min, self.roll_max)
      self.u[1] = proto.clip_value(pitch, self.pitch_min, self.pitch_max)
      self.u[2] = sp[3]
      self.u[3] = proto.clip_value(thrust, 0.0, 1.0)

    else:
      raise NotImplementedError()

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

    # Position and velocity controller
    self.last_ts = None
    self.pos_ctrl = MavPositionControl("ATTITUDE")
    self.vel_ctrl = MavVelocityControl()

  def get_traj(self):
    """ Return trajectory """
    pos_data = np.zeros((3, 1000))
    time = np.linspace(0.0, self.T, 1000)
    for i, t in enumerate(time):
      pos_data[:, i] = self.get_position(t).T
    return pos_data.T

  def get_position(self, t):
    """ Get position """
    w = 2.0 * np.pi * self.f
    theta = np.sin(0.25 * w * t)**2

    ka = 2.0 * np.pi * self.a
    kb = 2.0 * np.pi * self.b

    x = self.A * np.sin(ka * theta + self.delta)
    y = self.B * np.sin(kb * theta)
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
    w = 2.0 * np.pi * self.f
    theta = np.sin(0.25 * w * t)**2

    ka = 2.0 * np.pi * self.a
    kb = 2.0 * np.pi * self.b
    kpift = 0.5 * np.pi * self.f * t
    kx = 2.0 * np.pi**2 * self.A * self.a * self.f
    ky = 2.0 * np.pi**2 * self.B * self.b * self.f
    ksincos = np.sin(kpift) * np.cos(kpift)

    vx = kx * ksincos * np.cos(ka * np.sin(kpift)**2 + self.delta)
    vy = ky * ksincos * np.cos(kb * np.sin(kpift)**2)
    vz = 0.0

    return np.array([vx, vy, vz])

  def update(self, t, pos_pv, vel_pv):
    # Pre-check
    if self.last_ts is None:
      self.last_ts = t
      return np.array([0.0, 0.0, 0.0, 0.0])
    # dt = t - self.last_ts
    dt = 0.005

    # Get trajectory position, velocity and yaw
    traj_pos = self.get_position(t)
    traj_vel = self.get_velocity(t)
    traj_yaw = self.get_yaw(t)

    # Form position and velocity setpoints
    pos_sp = np.array([traj_pos[0], traj_pos[1], traj_pos[2], traj_yaw])
    vel_sp = [traj_vel[0], traj_vel[1], traj_vel[2], traj_yaw]

    # Position control
    att_pos_sp = self.pos_ctrl.update(pos_sp, pos_pv, dt)

    # Velocity control
    att_vel_sp = self.vel_ctrl.update(vel_sp, vel_pv, dt)

    # Mix both position and velocity control into a single attitude setpoint
    att_sp = np.array([0.0, 0.0, 0.0, 0.0])
    att_sp[0] = att_vel_sp[0] + att_pos_sp[0]
    att_sp[1] = att_vel_sp[1] + att_pos_sp[1]
    att_sp[2] = traj_yaw
    att_sp[3] = att_vel_sp[3] + att_pos_sp[3]

    att_sp[0] = proto.clip_value(att_sp[0], deg2rad(-35.0), deg2rad(35.0))
    att_sp[1] = proto.clip_value(att_sp[1], deg2rad(-35.0), deg2rad(35.0))
    att_sp[2] = att_sp[2]
    att_sp[3] = proto.clip_value(att_sp[3], 0.0, 1.0)

    # Update
    self.last_ts = t

    return att_sp

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
  START = 1
  TUNE = 2
  TRAJ = 3
  HOVER = 4
  LAND = 5


class MocapFilter:
  def __init__(self):
    self.initialized = False

    # Initial state x
    self.x = np.zeros(9)

    # Convariance
    self.P = np.eye(9)

    # Measurement Matrix
    self.H = np.block([np.eye(3), np.zeros((3, 6))])

    # Process Noise Matrix
    self.Q = np.eye(9)
    self.Q[0:3, 0:3] = 0.01 * np.eye(3)
    self.Q[3:6, 3:6] = 0.00001**2 * np.eye(3)
    self.Q[6:9, 6:9] = 0.00001**2 * np.eye(3)

    # Measurement Noise Matrix
    self.R = 0.1**2 * np.eye(3)

  def get_position(self):
    """ Get Position """
    return np.array([state[0], state[1], state[2]])

  def get_velocity(self):
    """ Get Velocity """
    return np.array([state[3], state[4], state[5]])

  def update(self, pos, dt):
    """ Update """
    # Initialize
    if self.initialized is False:
      self.x[0] = pos[0]
      self.x[1] = pos[1]
      self.x[2] = pos[2]
      self.initialized = True
      return False

    # Predict
    # -- Transition Matrix
    F = np.eye(9)
    F[0:3, 3:6] = np.eye(3) * dt
    F[0:3, 6:9] = np.eye(3) * dt**2
    F[3:6, 6:9] = np.eye(3) * dt
    # -- Predict
    self.x = F @ self.x
    self.P = F @ self.P @ F.T + self.Q

    # Update
    I = np.eye(9)
    y = z - self.H @ self.x
    S = self.R + self.H @ self.P @ self.H.T
    K = self.P @ self.H.T @ np.linalg.inv(S)
    self.x = self.x + K @ y
    self.P = (I - K @ self.H) @ self.P

    return True


class MavNode(Node):
  """Node for controlling a vehicle in offboard mode."""
  def __init__(self, **kwargs):
    super().__init__('mav_node')
    self.sim_mode = kwargs.get("sim_mode", True)

    self.is_running = True
    topic_control = "/fmu/in/offboard_control_mode"
    topic_traj = "/fmu/in/trajectory_setpoint"
    topic_att = "/fmu/in/vehicle_attitude_setpoint"
    topic_cmd = "/fmu/in/vehicle_command"
    topic_pos = "/fmu/out/vehicle_local_position"
    topic_status = "/fmu/out/vehicle_status"
    topic_pos_sp = "/mav/in/position_setpoint"
    topic_yaw_sp = "/mav/in/yaw_setpoint"
    topic_mocap = "/vicon/srl_mav/srl_mav"

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
    self.sub_mocap = sub_init(PoseStamped, topic_mocap, self.mocap_cb, qos)

    # State
    self.vehicle_local_position = LocalPosition()
    self.status = VehicleStatus()
    self.mode = MavMode.START
    self.ts = None
    self.ts_prev = None
    self.pos = None
    self.vel = None
    self.heading = 0.0
    self.traj_start = None
    self.hover_start = None
    self.tune_start = None

    # Settings
    self.takeoff_height = 2.0
    self.traj_period = 30.0
    self.hover_for = 3.0

    # Filter
    self.filter = MocapFilter()

    # Control
    self.pos_ctrl = MavPositionControl()
    self.vel_ctrl = MavVelocityControl()
    self.traj_ctrl = MavTrajectoryControl(a=1,
                                          b=2,
                                          delta=np.pi / 2,
                                          z=self.takeoff_height,
                                          T=self.traj_period)
    self.yaw_sp = 0.0
    self.pos_sp = [0.0, 0.0, 2.0, 0.0]

    # Create a timer to publish control commands
    self.dt = 0.005
    self.timer = self.create_timer(self.dt, self.timer_cb)

    # Engage offboard and arm MAV if in sim mode
    if self.sim_mode:
      self.arm()
      self.engage_offboard_mode()

    # Position control tuning waypoints
    self.vel_tune_setpoints = [
        # Tune z-axis
        [0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, -0.2, 0.0],
        [0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, +0.2, 0.0],
        [0.0, 0.0, 0.0, 0.0],
        # Tune x-axis
        [0.0, 0.0, 0.0, 0.0],
        [0.5, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0],
        [-0.5, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0],
        # Tune y-axis
        [0.0, 0.0, 0.0, 0.0],
        [0.0, 0.5, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0],
        [0.0, -0.5, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0]
    ]

    # Position control tuning waypoints
    self.pos_tune_setpoints = [
        # Tune x-axis
        [0.0, 0.0, self.takeoff_height, 0.0],
        [1.0, 0.0, self.takeoff_height, 0.0],
        [-1.0, 0.0, self.takeoff_height, 0.0],
        [0.0, 0.0, self.takeoff_height, 0.0],
        # Tune y-axis
        [0.0, 0.0, self.takeoff_height, 0.0],
        [0.0, 0.0, self.takeoff_height, 0.0],
        [0.0, 1.0, self.takeoff_height, 0.0],
        [0.0, -1.0, self.takeoff_height, 0.0],
        [0.0, 0.0, self.takeoff_height, 0.0]
    ]

    # Data
    self.pos_time = []
    self.pos_actual = []
    self.pos_traj = []

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

  def mocap_cb(self, msg):
    """Mocap callback"""
    self.ts = int(msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec)
    dt = float((self.ts - self.ts_prev) * 1e-9) if self.ts_prev else 0.0
    self.ts_prev = self.ts

    rx = msg.pose.position.x
    ry = msg.pose.position.y
    rz = msg.pose.position.z
    pos = np.array([rx, ry, rz])

    if self.filter.update(pos, dt):
      self.pos = self.filter.get_position()
      self.vel = self.filter.get_velocity()

  def pos_sp_cb(self, msg):
    """Callback function for position setpoint topic subscriber."""
    self.pos_sp = [msg.x, msg.y, msg.z, self.yaw_sp]

  def yaw_sp_cb(self, msg):
    """Callback function for yaw setpoint topic subscriber."""
    self.yaw_sp = msg.data
    self.pos_sp[3] = self.yaw_sp

  def status_cb(self, vehicle_status):
    """Callback function for vehicle_status topic subscriber."""
    self.status = vehicle_status

  def is_armed(self):
    """ Is armed? """
    return self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED

  def is_offboard(self):
    """ Is offboard? """
    return self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

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
    if yaw_sp >= np.pi:
      yaw_sp -= 2.0 * np.pi
    elif yaw_sp <= -np.pi:
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

  def execute_velocity_control_test(self):
    # Process variables
    pos_pv = [self.pos[0], self.pos[1], self.pos[2], self.heading]
    vel_pv = [self.vel[0], self.vel[1], self.vel[2], self.heading]

    if self.mode == MavMode.START:
      # Start hover timer
      if self.hover_start is None:
        self.hover_start = self.ts

      # Get hover point
      self.pos_sp = [0.0, 0.0, self.takeoff_height, self.yaw_sp]

      # Update position controller
      vel_sp = self.pos_ctrl.update(self.pos_sp, pos_pv, self.dt)
      u = self.vel_ctrl.update(vel_sp, vel_pv, self.dt)
      self.pub_attitude_sp(u[0], u[1], u[2], u[3])

      # Transition to land?
      hover_time = float(self.ts - self.hover_start) * 1e-9
      dx = self.pos[0] - self.pos_sp[0]
      dy = self.pos[1] - self.pos_sp[1]
      dz = self.pos[2] - self.pos_sp[2]
      dpos = np.sqrt(dx * dx + dy * dy + dz * dz)
      dyaw = np.fabs(self.heading - self.yaw_sp)
      if dpos < 0.2 and dyaw < np.deg2rad(10.0) and hover_time > 3.0:
        self.get_logger().info('TRANSITION TO TUNE!')
        self.mode = MavMode.TUNE
        self.hover_start = None

    elif self.mode == MavMode.TUNE:
      # Start hover timer
      if self.tune_start is None:
        self.tune_start = self.ts

      # Velocity controller
      vel_sp = self.vel_tune_setpoints[0]
      u = self.vel_ctrl.update(vel_sp, vel_pv, self.dt)
      self.pub_attitude_sp(u[0], u[1], u[2], u[3])

      # Check if position setpoint reached
      tune_time = float(self.ts - self.tune_start) * 1e-9
      if tune_time >= 2.0:
        self.get_logger().info('BACK TO START!')
        self.mode = MavMode.START
        self.vel_tune_setpoints.pop(0)
        self.tune_start = None
        self.hover_start = None

      # Land?
      if len(self.vel_tune_setpoints) == 0:
        self.mode = MavMode.LAND
        self.tune_start = None

    elif self.mode == MavMode.LAND:
      # Dis-armed?
      if self.status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
        self.stop_node()
        return

      # Land
      if self.status.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
        self.land()

  def execute_position_control_test(self):
    # Start tune timestamp
    if self.tune_start is None:
      self.tune_start = self.ts

    # Process variables
    pos_pv = [self.pos[0], self.pos[1], self.pos[2], self.heading]
    vel_pv = [self.vel[0], self.vel[1], self.vel[2], self.heading]

    if self.mode == MavMode.START:
      # Start hover timer
      if self.hover_start is None:
        self.hover_start = self.ts

      # Get hover point
      self.pos_sp = [0.0, 0.0, self.takeoff_height, self.yaw_sp]

      # Update position controller
      vel_sp = self.pos_ctrl.update(self.pos_sp, pos_pv, self.dt)
      u = self.vel_ctrl.update(vel_sp, vel_pv, self.dt)
      self.pub_attitude_sp(u[0], u[1], u[2], u[3])

      # Transition to land?
      hover_time = float(self.ts - self.hover_start) * 1e-9
      dx = self.pos[0] - self.pos_sp[0]
      dy = self.pos[1] - self.pos_sp[1]
      dz = self.pos[2] - self.pos_sp[2]
      dpos = np.sqrt(dx * dx + dy * dy + dz * dz)
      dyaw = np.fabs(self.heading - self.yaw_sp)
      if dpos < 0.2 and dyaw < np.deg2rad(10.0) and hover_time > 3.0:
        self.get_logger().info('TRANSITION TO TUNE!')
        self.mode = MavMode.TUNE
        self.hover_start = None

    elif self.mode == MavMode.TUNE:
      # Position controller
      self.pos_sp = self.pos_tune_setpoints[0]
      vel_sp = self.pos_ctrl.update(self.pos_sp, pos_pv, self.dt)
      u = self.vel_ctrl.update(vel_sp, vel_pv, self.dt)
      self.pub_attitude_sp(u[0], u[1], u[2], u[3])

      # Check if position setpoint reached
      tune_time = float(self.ts - self.tune_start) * 1e-9
      dx = self.pos[0] - self.pos_sp[0]
      dy = self.pos[1] - self.pos_sp[1]
      dz = self.pos[2] - self.pos_sp[2]
      dpos = np.sqrt(dx * dx + dy * dy + dz * dz)
      if dpos < 0.1 and tune_time >= self.hover_for:
        self.pos_tune_setpoints.pop(0)
        self.tune_start = None

      # Land?
      if len(self.pos_tune_setpoints) == 0:
        self.mode = MavMode.LAND
        self.tune_start = None

    elif self.mode == MavMode.LAND:
      # Dis-armed?
      if self.status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
        self.stop_node()
        return

      # Land
      if self.status.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
        self.land()

  def execute_trajectory(self):
    # Process variables
    pos_pv = [self.pos[0], self.pos[1], self.pos[2], self.heading]
    vel_pv = [self.vel[0], self.vel[1], self.vel[2], self.heading]

    # GO TO START
    if self.mode == MavMode.START:
      # Set yaw and position setpoint
      x0, y0, z0 = self.traj_ctrl.get_position(0.0)
      self.yaw_sp = self.traj_ctrl.get_yaw(0.0)
      self.pos_sp = [x0, y0, z0, self.yaw_sp]

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
        self.mode = MavMode.TRAJ

    # EXECUTE TRAJECTORY
    elif self.mode == MavMode.TRAJ:
      # Run trajectory
      if self.traj_start is None:
        self.traj_start = self.ts

      # Update trajectory controller
      t = float(self.ts - self.traj_start) * 1e-9
      u = self.traj_ctrl.update(t, pos_pv, vel_pv)
      self.pub_attitude_sp(u[0], u[1], u[2], u[3])

      # Record position
      self.pos_time.append(t)
      self.pos_actual.append([self.pos[0], self.pos[1], self.pos[2]])
      self.pos_traj.append(self.traj_ctrl.get_position(t))

      # Transition to hover?
      if t >= self.traj_period:
        self.mode = MavMode.HOVER
        self.traj_start = None

    # HOVER
    elif self.mode == MavMode.HOVER:
      # Start hover timer
      if self.hover_start is None:
        self.hover_start = self.ts

      # Get hover point
      pos = self.traj_ctrl.get_position(0.0)
      yaw = self.traj_ctrl.get_yaw(0.0)
      self.pos_sp = [pos[0], pos[1], pos[2], yaw]

      # Update position controller
      vel_sp = self.pos_ctrl.update(self.pos_sp, pos_pv, self.dt)
      u = self.vel_ctrl.update(vel_sp, vel_pv, self.dt)
      self.pub_attitude_sp(u[0], u[1], u[2], u[3])

      # Transition to land?
      hover_time = float(self.ts - self.hover_start) * 1e-9
      if hover_time > self.hover_for:
        self.hover_start = None
        self.mode = MavMode.LAND

    # LAND
    elif self.mode == MavMode.LAND:
      # Dis-armed?
      if self.status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
        self.stop_node()
        return

      # Land
      if self.status.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
        self.land()

  def timer_cb(self):
    """Callback function for the timer."""
    self.pub_heart_beat()

    # Check we are receiving position and velocity information
    if self.pos is None or self.vel is None:
      return

    # Check MAV is armed and offboard mode activated
    if self.is_armed() and self.is_offboard():
      return

    # self.execute_velocity_control_test()
    # self.execute_position_control_test()
    self.execute_trajectory()

  def stop_node(self):
    """ Stop Node """
    self.get_logger().info('Stopping the node')
    self.timer.cancel()
    self.destroy_timer(self.timer)

    self.sub_pos.destroy()
    self.sub_pos_sp.destroy()
    self.sub_yaw_sp.destroy()
    self.sub_status.destroy()

    self.pos_time = np.array(self.pos_time)
    self.pos_actual = np.array(self.pos_actual)
    self.pos_traj = np.array(self.pos_traj)

    plt.plot(self.pos_actual[:, 0], self.pos_actual[:, 1], "r-", label="Actual")
    plt.plot(self.pos_traj[:, 0],
             self.pos_traj[:, 1],
             "k--",
             label="Trajectory")
    plt.show()

    self.get_logger().info('Destroying the node')
    self.destroy_node()
    self.is_running = False


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument("--sim_mode", default=True)
  args = parser.parse_args()

  # Run Mav Node
  rclpy.init()
  node = MavNode(sim_mode=args.sim_mode)
  while node.is_running:
    rclpy.spin_once(node)
  rclpy.shutdown()
