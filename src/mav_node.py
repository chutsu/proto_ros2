import numpy as np
from numpy import deg2rad
from numpy import rad2deg
import matplotlib.pylab as plt

import proto
from proto import euler2quat

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import DurabilityPolicy
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleAttitudeSetpoint
from px4_msgs.msg import VehicleStatus


class PID:
  """ PID controller """
  def __init__(self, k_p, k_i, k_d):
    self.k_p = k_p
    self.k_i = k_i
    self.k_d = k_d

    self.error_p = 0.0
    self.error_i = 0.0
    self.error_d = 0.0
    self.error_prev = 0.0
    self.error_sum = 0.0

  def update(self, setpoint, actual, dt):
    """ Update """
    # Calculate errors
    error = setpoint - actual
    self.error_sum += error * dt

    # Calculate output
    self.error_p = self.k_p * error
    self.error_i = self.k_i * self.error_sum
    self.error_d = self.k_d * (error - self.error_prev) / dt
    output = self.error_p + self.error_i + self.error_d

    # Keep track of error
    self.error_prev = error

    return output

  def reset(self):
    """ Reset """
    self.error_prev = 0
    self.error_sum = 0
    self.error_p = 0
    self.error_i = 0
    self.error_d = 0


class MavVelocityControl:
  def __init__(self):
    self.dt = 0
    self.pid_vx = PID(0.5, 0.0, 0.05)
    self.pid_vy = PID(0.5, 0.0, 0.05)
    self.pid_vz = PID(0.5, 0.0, 0.05)
    self.u = [0.0, 0.0, 0.0, 0.0]

  def update(self, sp, pv, dt):
    """ Update """
    # Check rate
    self.dt += dt
    if self.dt < 0.0055:
      return self.u  # Return previous command

    # Calculate RPY errors relative to quadrotor by incorporating yaw
    errors_W = np.array([sp[0] - pv[0], sp[1] - pv[1], sp[2] - pv[2]])
    C_WS = proto.euler321(pv[3], 0.0, 0.0)
    errors = C_WS.T @ errors_W

    # Roll, pitch, yaw and thrust
    r = -self.pid_vy.update(errors[1], 0.0, dt)
    p = self.pid_vx.update(errors[0], 0.0, dt)
    y = sp[3]
    t = 0.7 + self.pid_vz.update(errors[2], 0.0, dt)

    self.u[0] = proto.clip_value(r, deg2rad(-20.0), deg2rad(20.0))
    self.u[1] = proto.clip_value(p, deg2rad(-20.0), deg2rad(20.0))
    self.u[2] = y
    self.u[3] = proto.clip_value(t, 0.0, 1.0)

    # # Yaw first if threshold reached
    # if (fabs(sp[3] - pv[3]) > deg2rad(2)) {
    #   outputs[0] = 0.0;
    #   outputs[1] = 0.0;
    # }

    # Keep track of control action
    self.dt = 0.0  # Reset dt

    return self.u

  def reset(self):
    """ Reset """
    self.dt = 0.0
    self.pid_vx.reset()
    self.pid_vy.reset()
    self.pid_vz.reset()
    self.u = [0.0, 0.0, 0.0, 0.0]


class MavNode(Node):
  """Node for controlling a vehicle in offboard mode."""
  def __init__(self) -> None:
    super().__init__('mav_node')
    topic_control = "/fmu/in/offboard_control_mode"
    topic_traj = "/fmu/in/trajectory_setpoint"
    topic_att = "/fmu/in/vehicle_attitude_setpoint"
    topic_cmd = "/fmu/in/vehicle_command"
    topic_pos = "/fmu/out/vehicle_local_position"
    topic_status = "/fmu/out/vehicle_status"

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
    self.sub_pos = sub_init(VehicleLocalPosition, topic_pos, self.pos_cb, qos)
    self.sub_status = sub_init(VehicleStatus, topic_status, self.status_cb, qos)

    # Initialize variables
    self.offboard_setpoint_counter = 0
    self.vehicle_local_position = VehicleLocalPosition()
    self.vehicle_status = VehicleStatus()
    self.takeoff_height = 1.0

    self.pos = [0.0, 0.0, 0.0]
    self.vel = [0.0, 0.0, 0.0]
    self.heading = 0.0
    self.vel_ctrl = MavVelocityControl()

    # Create a timer to publish control commands
    self.dt =  0.005
    self.timer = self.create_timer(self.dt, self.timer_cb)

  def pos_cb(self, vehicle_local_position):
    """Callback function for vehicle_local_position topic subscriber."""
    self.vehicle_local_position = vehicle_local_position

    pos_x = self.vehicle_local_position.y
    pos_y = self.vehicle_local_position.x
    pos_z = -1.0 * self.vehicle_local_position.z
    self.pos = np.array([pos_x, pos_y, pos_z])

    vel_x = self.vehicle_local_position.vy
    vel_y = self.vehicle_local_position.vx
    vel_z = -1.0 * self.vehicle_local_position.vz
    self.vel = np.array([vel_x, vel_y, vel_z])

    self.heading = self.vehicle_local_position.heading - deg2rad(90.0)
    # self.get_logger().info(f"POS: {[pos_x, pos_y, pos_z, self.heading]}")

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
    self.get_logger().info("Switching to land mode")

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
    # self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

  def pub_attitude_sp(self, roll, pitch, yaw, thrust):
    qw, qx, qy, qz = euler2quat(deg2rad(90.0) + yaw, -pitch, roll)

    msg = VehicleAttitudeSetpoint()
    msg.q_d = [qw, qx, qy, qz]
    msg.thrust_body = [0.0, 0.0, -thrust]
    self.pub_att.publish(msg)

  def cmd(self, command, **params) -> None:
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

  def timer_cb(self) -> None:
    """Callback function for the timer."""
    self.pub_heart_beat()

    if self.offboard_setpoint_counter == 10:
      self.engage_offboard_mode()
      self.arm()

    # self.pub_position_sp(0.0, 5.0, self.takeoff_height)
    # self.pub_attitude_sp(0.0, 0.0, 0.0)

    vel_pv = [self.vel[0], self.vel[1], self.vel[2], self.heading]
    vel_sp = [-0.5, 0.5, 0.2, 0.0]
    u = self.vel_ctrl.update(vel_sp, vel_pv, self.dt)

    self.pub_attitude_sp(u[0], u[1], u[2], u[3])
    self.get_logger().info(f"u: {u}")

    if self.offboard_setpoint_counter < 11:
      self.offboard_setpoint_counter += 1


if __name__ == '__main__':
  rclpy.init()
  offboard_control = MavNode()
  rclpy.spin(offboard_control)
  offboard_control.destroy_node()
  rclpy.shutdown()
