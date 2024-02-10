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


class OffboardControl(Node):
  """Node for controlling a vehicle in offboard mode."""
  def __init__(self) -> None:
    super().__init__('offboard_control_takeoff_and_land')
    topic_control = "/fmu/in/offboard_control_mode"
    topic_traj = "/fmu/in/trajectory_setpoint"
    topic_att = "/fmu/in/vehicle_attitude_setpoint"
    topic_cmd = "/fmu/in/vehicle_command"
    topic_pos = "/fmu/out/pos"
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
    self.takeoff_height = -5.0

    # Create a timer to publish control commands
    self.timer = self.create_timer(0.1, self.timer_cb)

  def pos_cb(self, vehicle_local_position):
    """Callback function for vehicle_local_position topic subscriber."""
    self.vehicle_local_position = vehicle_local_position

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

  def pub_position_sb(self, x: float, y: float, z: float):
    """Publish the trajectory setpoint."""
    msg = TrajectorySetpoint()
    msg.position = [x, y, z]
    msg.yaw = 1.57079  # (90 degree)
    msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
    self.pub_traj.publish(msg)
    self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

  def pub_attitude_sb(self, x: float, y: float, z: float):
    msg = VehicleAttitudeSetpoint()
    msg.q_d = [0.0, 0.0, 0.0, 1.0]
    msg.thrust_body = [0.0, 0.0, -1.0]
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

    self.pub_position_sb(0.0, 0.0, self.takeoff_height)
    # self.pub_attitude_sb(0.0, 0.0, 0.0)

    if self.offboard_setpoint_counter < 11:
      self.offboard_setpoint_counter += 1


if __name__ == '__main__':
  rclpy.init()
  offboard_control = OffboardControl()
  rclpy.spin(offboard_control)
  offboard_control.destroy_node()
  rclpy.shutdown()
