"""Simple control wrapper for piper_sdk."""

import time
from enum import IntEnum
from typing import Literal, Sequence, TypeGuard

import piper_sdk

# Global constants
DEG_TO_RAD = 0.017444
RAD_TO_DEG = 1 / DEG_TO_RAD
GRIPPER_ANGLE_MAX = 0.07  # 70mm
GRIPPER_EFFORT_MAX = 2.0  # 2 Nm


class EmergencyStop(IntEnum):
  INVALID = 0x00
  STOP = 0x01
  RESUME = 0x02


class ControlMode(IntEnum):
  STANDBY = 0x00
  CAN_COMMAND = 0x01
  ETHERNET = 0x03
  WIFI = 0x04
  OFFLINE_TRAJECTORY = 0x07


def validate_control_mode(
    mode: ControlMode,
) -> TypeGuard[Literal[0, 1, 3, 4, 7]]:
  """
  Validate the control mode is one of the allowed values from Piper SDK.

  This function is mainly here for type checking and making linters happy.

  Args:
      mode (ControlMode): The control mode to validate.

  Returns:
      bool: True if the control mode is valid, False otherwise.
  """
  return mode in {0, 1, 3, 4, 7}


class MoveMode(IntEnum):
  POSITION = 0x00
  JOINT = 0x01
  LINEAR = 0x02
  CIRCULAR = 0x03
  MIT = 0x04


def validate_move_mode(mode: MoveMode) -> TypeGuard[Literal[0, 1, 2, 3, 4]]:
  """
  Validate the move mode is one of the allowed values from Piper SDK.

  This function is mainly here for type checking and making linters happy.

  Args:
      mode (MoveMode): The move mode to validate.

  Returns:
      bool: True if the move mode is valid, False otherwise.
  """
  return mode in {0, 1, 2, 3, 4}


class ArmController(IntEnum):
  POSITION_VELOCITY = 0x00
  MIT = 0xAD
  INVALID = 0xFF


def validate_arm_controller(
    controller: ArmController,
) -> TypeGuard[Literal[0, 173, 255]]:
  """
  Validate the arm controller is one of the allowed values from Piper SDK.

  This function is mainly here for type checking and making linters happy.

  Args:
      controller (ArmController): The arm controller to validate.

  Returns:
      bool: True if the arm controller is valid, False otherwise.
  """
  return controller in {0, 173, 255}


class ArmStatus(IntEnum):
  DISABLED = 0x00
  ENABLED = 0x01
  ERROR = 0x02
  UNKNOWN = 0xFF


class GripperCode(IntEnum):
  DISABLE = 0x00
  ENABLE = 0x01
  DISABLE_AND_CLEAR_ERROR = 0x02
  ENABLE_AND_CLEAR_ERROR = 0x03


class ArmInstallationPos(IntEnum):
  UPRIGHT = 0x01  # Horizontal upright

  # Side mount left. In this orientation, the rear cable is facing backward and
  # the green LED is above the cable socket.
  LEFT = 0x02

  # Side mount right. In this orientation, the rear cable is facing backward and
  # the green LED is below the cable socket.
  RIGHT = 0x03


JOINT_LIMITS_RAD = {
    "min": [-2.6179, 0.0, -2.967, -1.745, -1.22, -2.09439],
    "max": [2.6179, 3.14, 0.0, 1.745, 1.22, 2.09439],
}


REST_POSITION = [0.0, 0.0, 0.0, -0.146, 0.65, -0.010]
DOWN_POSITION = [0.0, 1.6, -0.85, 0.0, 0.75, 0.0]


class PiperControl:
  """
  A wrapper around the Piper robot SDK to provide high-level control and state
  functionality.
  """

  def __init__(
      self,
      can_port: str = "can0",
      installation_pos: ArmInstallationPos = ArmInstallationPos.UPRIGHT,
  ) -> None:
    """
    Initializes the PiperControl with a specified CAN port.

    Args:
      can_port (str): The CAN interface port name (e.g., "can0").
      installation_pos: The installation pose of the arm.
    """
    self.can_port = can_port
    self._installation_pos = installation_pos

    self.piper = piper_sdk.C_PiperInterface_V2(can_name=can_port)
    self.piper.ConnectPort()

  def get_status(self) -> piper_sdk.C_PiperInterface_V2.ArmStatus:
    """
    Gets the current control mode of the robot.

    Returns:
      ArmStatus: The current arm status.
    """
    return self.piper.GetArmStatus()

  def reset(
      self,
      enable_arm: bool = True,
      enable_motion: bool = True,
      arm_controller: ArmController = ArmController.POSITION_VELOCITY,
      move_mode: MoveMode = MoveMode.POSITION,
      enable_time_limit: float = 10.0,
  ) -> None:
    """
    Resets the robot controller. Beware it will fall!

    Args:
      enable_arm (bool): Whether to enable the arm.
      enable_motion (bool): Whether to enable motion control.
      arm_controller (ArmController): The arm controller mode to use.
      move_mode (MoveMode): The move mode to use.
      enable_time_limit (float): Maximum number of seconds to retry enabling.
    """

    self.disable()
    self.piper.MotionCtrl_1(EmergencyStop.RESUME, 0, 0)
    self._standby(
        move_mode=move_mode,
        arm_controller=arm_controller,
    )

    if not enable_arm and not enable_motion:
      # Caller requested only disabling the arm.
      print("Robot in standby. Call `enable` to send commands.")
      return

    time.sleep(0.5)

    # Loop until the arm is enabled.
    # The arm can sometimes get disabled while trying to enable it or the motion
    # controller. For this reason, we need to check both, and keep looping until
    # both the arm and motion controller are enabled.

    # TODO(jscholz) This nested while loop of enable calls feels like overkill.
    start_time = time.time()
    finished_enabling = False
    while time.time() - start_time < enable_time_limit:
      if enable_arm:
        self.enable()
        time.sleep(0.5)
      if enable_motion:
        self._enable_motion(
            arm_controller=arm_controller,
            move_mode=move_mode,
        )
        time.sleep(0.5)
      status = self.get_status().arm_status
      arm_enabled = self.is_enabled()
      motion_enabled = (
          status.ctrl_mode == ControlMode.CAN_COMMAND
          and status.mode_feed == move_mode
      )
      if enable_arm:
        print(f"Arm enabled: {arm_enabled}")
      if enable_motion:
        print(f"Motion enabled: {motion_enabled}")
      # Only check the states that are requested to be enabled.
      if ((enable_arm and arm_enabled) or not enable_arm) and (
          (enable_motion and motion_enabled) or not enable_motion
      ):
        print("✅ Finished enabling")
        finished_enabling = True
        break
    if not finished_enabling:
      print("❌ Failed to enable arm and/or motion.")
      raise RuntimeError("Failed to enable Piper.")

  def is_enabled(self) -> bool:
    """
    Checks if the robot arm is enabled.

    Returns:
      bool: True if the arm  and gripper are enabled, False otherwise.
    """
    arm_msgs = self.piper.GetArmLowSpdInfoMsgs()
    arm_enabled = (
        arm_msgs.motor_1.foc_status.driver_enable_status
        and arm_msgs.motor_2.foc_status.driver_enable_status
        and arm_msgs.motor_3.foc_status.driver_enable_status
        and arm_msgs.motor_4.foc_status.driver_enable_status
        and arm_msgs.motor_5.foc_status.driver_enable_status
        and arm_msgs.motor_6.foc_status.driver_enable_status
    )

    gripper_msgs = self.piper.GetArmGripperMsgs()
    gripper_enabled = gripper_msgs.gripper_state.foc_status.driver_enable_status

    return arm_enabled and gripper_enabled

  def enable(self, timeout: float = 5.0) -> None:
    """Attempts to enable the arm and gripper retrying for up to 5 seconds."""
    enable_flag = False
    start_time = time.time()
    elapsed_time_flag = False

    while not enable_flag:
      elapsed_time = time.time() - start_time
      enable_flag = self.is_enabled()
      print("Enable status:", enable_flag)

      self.piper.EnableArm(7)
      self.piper.GripperCtrl(0, 1000, GripperCode.ENABLE, 0)

      if elapsed_time > timeout:
        print("Timeout occurred...")
        elapsed_time_flag = True
        enable_flag = True  # break the loop   # TODO(jscholz) do we need this?
        break

      time.sleep(0.5)  # TODO(jscholz) do we need this?

    if elapsed_time_flag:
      print("Automatic enable timed out.")

  def disable(self) -> None:
    """
    Disables the robot arm.
    """
    self.piper.DisableArm(7)
    self.piper.GripperCtrl(0, 0, GripperCode.DISABLE_AND_CLEAR_ERROR, 0)

  def _standby(
      self,
      move_mode: MoveMode = MoveMode.JOINT,
      arm_controller: ArmController = ArmController.POSITION_VELOCITY,
  ) -> None:
    """
    Puts the robot into standby mode.
    """
    if not validate_move_mode(move_mode):
      raise ValueError(f"Invalid move mode: {move_mode}")
    if not validate_arm_controller(arm_controller):
      raise ValueError(f"Invalid arm controller: {arm_controller}")
    self.piper.MotionCtrl_2(
        ControlMode.STANDBY,
        move_mode,
        0,
        arm_controller,
        installation_pos=self._installation_pos.value,
    )

  def _enable_motion(
      self,
      speed: int = 100,
      move_mode: MoveMode = MoveMode.JOINT,
      ctrl_mode: ControlMode = ControlMode.CAN_COMMAND,
      arm_controller: ArmController = ArmController.POSITION_VELOCITY,
  ) -> None:
    """
    Enables motion control for the arm and gripper.

    Args:
      speed (int): Speed setting for the motion control.
      move_mode (MoveMode): Move mode to use (e.g., POSITION, JOINT).
      ctrl_mode (ControlMode): Control mode to use (e.g., CAN_COMMAND).
      arm_controller (ArmController): MIT mode to use (e.g., POSITION_VELOCITY).
    """
    if not validate_move_mode(move_mode):
      raise ValueError(f"Invalid move mode: {move_mode}")
    if not validate_arm_controller(arm_controller):
      raise ValueError(f"Invalid arm controller: {arm_controller}")
    if not validate_control_mode(ctrl_mode):
      raise ValueError(f"Invalid control mode: {ctrl_mode}")
    self.piper.MotionCtrl_2(
        ctrl_mode,
        move_mode,
        speed,
        arm_controller,
        installation_pos=self._installation_pos.value,
    )

  def get_joint_positions(self) -> list[float]:
    """
    Returns the current joint positions as a sequence of floats (radians).

    Returns:
      Sequence[float]: Joint positions in radians.
    """
    return [
        self.piper.GetArmJointMsgs().joint_state.joint_1 / 1e3 * DEG_TO_RAD,
        self.piper.GetArmJointMsgs().joint_state.joint_2 / 1e3 * DEG_TO_RAD,
        self.piper.GetArmJointMsgs().joint_state.joint_3 / 1e3 * DEG_TO_RAD,
        self.piper.GetArmJointMsgs().joint_state.joint_4 / 1e3 * DEG_TO_RAD,
        self.piper.GetArmJointMsgs().joint_state.joint_5 / 1e3 * DEG_TO_RAD,
        self.piper.GetArmJointMsgs().joint_state.joint_6 / 1e3 * DEG_TO_RAD,
    ]

  def get_joint_velocities(self) -> list[float]:
    """
    Returns the current joint velocities as a sequence of floats.

    Returns:
      Sequence[float]: Joint velocities in radians per second.
    """
    return [
        self.piper.GetArmHighSpdInfoMsgs().motor_1.motor_speed / 1e3,
        self.piper.GetArmHighSpdInfoMsgs().motor_2.motor_speed / 1e3,
        self.piper.GetArmHighSpdInfoMsgs().motor_3.motor_speed / 1e3,
        self.piper.GetArmHighSpdInfoMsgs().motor_4.motor_speed / 1e3,
        self.piper.GetArmHighSpdInfoMsgs().motor_5.motor_speed / 1e3,
        self.piper.GetArmHighSpdInfoMsgs().motor_6.motor_speed / 1e3,
    ]

  def get_joint_efforts(self) -> list[float]:
    """
    Returns the current joint efforts as a sequence of floats.

    Returns:
      Sequence[float]: Joint efforts in Nm.
    """
    return [
        self.piper.GetArmHighSpdInfoMsgs().motor_1.effort / 1e3,
        self.piper.GetArmHighSpdInfoMsgs().motor_2.effort / 1e3,
        self.piper.GetArmHighSpdInfoMsgs().motor_3.effort / 1e3,
        self.piper.GetArmHighSpdInfoMsgs().motor_4.effort / 1e3,
        self.piper.GetArmHighSpdInfoMsgs().motor_5.effort / 1e3,
        self.piper.GetArmHighSpdInfoMsgs().motor_6.effort / 1e3,
    ]

  def get_gripper_state(self) -> tuple[float, float]:
    """
    Returns the current gripper state as a tuple of angle and effort.

    Returns:
      tuple[float, float]: (gripper_angle, gripper_effort)
    """
    raw_angle = self.piper.GetArmGripperMsgs().gripper_state.grippers_angle
    raw_effort = self.piper.GetArmGripperMsgs().gripper_state.grippers_effort

    angle = raw_angle / 1e6
    effort = raw_effort / 1e3

    return angle, effort

  def set_joint_positions(self, positions: Sequence[float]) -> None:
    """
    Sets the joint positions using JointCtrl.

    Args:
      positions (Sequence[float]): A list of joint angles in radians.
    """
    self._enable_motion(
        arm_controller=ArmController.POSITION_VELOCITY, move_mode=MoveMode.JOINT
    )
    joint_angles = []
    for i, pos in enumerate(positions):
      min_rad, max_rad = JOINT_LIMITS_RAD["min"][i], JOINT_LIMITS_RAD["max"][i]
      clipped_pos = min(max(pos, min_rad), max_rad)
      pos_deg = clipped_pos * RAD_TO_DEG
      joint_angle = round(pos_deg * 1e3)  # Convert to millidegrees
      joint_angles.append(joint_angle)
    self.piper.JointCtrl(*joint_angles)  # pylint: disable=no-value-for-parameter

  def set_joint_mit_ctrl(
      self,
      positions: Sequence[float],
      velocities: Sequence[float],
      kps: Sequence[float],
      kds: Sequence[float],
      efforts: Sequence[float],
      motor_idxs: Sequence[int] = range(6),
  ) -> None:
    """
    Sets the MIT control mode for multiple joints.

    Args:
      positions (Sequence[float]): Desired positions in radians.
      velocities (Sequence[float]): Desired velocities in radians per second.
      kps (Sequence[float]): Proportional gains.
      kds (Sequence[float]): Derivative gains.
      efforts (Sequence[float]): Target torques in Nm.
      motor_idxs (Sequence[int]): Motor indices to control (default: range(6)).
    """
    self._enable_motion(arm_controller=ArmController.MIT)
    for idx in motor_idxs:
      pos_deg = positions[idx] * RAD_TO_DEG
      vel_ref = velocities[idx]
      kp = kps[idx]
      kd = kds[idx]
      t_ref = efforts[idx]
      self.piper.JointMitCtrl(idx + 1, pos_deg, vel_ref, kp, kd, t_ref)

  def set_cartesian_position(self, pose: Sequence[float]) -> None:
    """
    Sets the Cartesian position and orientation of the robot end-effector.

    Args:
        pose: [x, y, z, roll, pitch, yaw] in meters and radians.
    """
    self._enable_motion(
        move_mode=MoveMode.POSITION,
        arm_controller=ArmController.POSITION_VELOCITY,
    )
    x_mm = round(pose[0] * 1e6)
    y_mm = round(pose[1] * 1e6)
    z_mm = round(pose[2] * 1e6)
    roll_deg = round(pose[3] * RAD_TO_DEG * 1e3)
    pitch_deg = round(pose[4] * RAD_TO_DEG * 1e3)
    yaw_deg = round(pose[5] * RAD_TO_DEG * 1e3)
    self.piper.EndPoseCtrl(x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg)

  def get_end_effector_pose(self) -> Sequence[float]:
    """
    Returns the current end-effector pose as a sequence of floats.

    Returns:
      Sequence[float]: (x, y, z, roll, pitch, yaw) in meters and radians.
    """
    pose = self.piper.GetArmEndPoseMsgs()
    x = pose.end_pose.X_axis * 1e-6  # Convert from mm to m
    y = pose.end_pose.Y_axis * 1e-6
    z = pose.end_pose.Z_axis * 1e-6
    roll = pose.end_pose.RX_axis * 1e-3 * DEG_TO_RAD
    pitch = pose.end_pose.RY_axis * 1e-3 * DEG_TO_RAD
    yaw = pose.end_pose.RZ_axis * 1e-3 * DEG_TO_RAD
    return [x, y, z, roll, pitch, yaw]

  def set_gripper_ctrl(
      self,
      position: float | None = None,
      effort: float | None = None,
  ) -> None:
    """
    Controls the gripper by setting its position and effort.

    Args:
      position (float | None): The desired position of the gripper in meters.
        Must be between 0.0 and GRIPPER_ANGLE_MAX. If None, the position is not
        updated.
      effort (float | None): The desired effort (force) for the gripper.
        Must be between 0.0 and GRIPPER_EFFORT_MAX. If None, the effort is not
        updated.

    Returns:
      None
    """
    position_int = effort_int = 0
    if position is not None:
      position = min(max(position, 0.0), GRIPPER_ANGLE_MAX)
      position_int = round(position * 1e6)
    if effort is not None:
      effort = min(max(effort, 0.0), GRIPPER_EFFORT_MAX)
      effort_int = round(effort * 1e3)

    print(f"sending {position_int=} {effort_int=}")
    self.piper.GripperCtrl(position_int, effort_int, 0x01, 0)

