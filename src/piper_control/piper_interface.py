"""Simple wrapper for piper_sdk."""

from enum import IntEnum
from typing import Literal, Sequence, TypeGuard

import piper_sdk

# Global constants
DEG_TO_RAD = 3.1415926 / 180.0
RAD_TO_DEG = 1 / DEG_TO_RAD
GRIPPER_ANGLE_MAX = 0.07  # 70mm
GRIPPER_EFFORT_MAX = 2.0  # 2 Nm


class EmergencyStop(IntEnum):
    INVALID = 0x00
    STOP = 0x01
    RESUME = 0x02


def validate_emergency_stop(
    state: EmergencyStop,
) -> TypeGuard[Literal[0, 1, 2]]:
    return state in {0, 1, 2}


# https://github.com/agilexrobotics/piper_sdk/blob/4eddfcf817cd87de9acee316a72cf5b988025378/piper_msgs/msg_v2/feedback/arm_status.py#L108
class ControlMode(IntEnum):
    STANDBY = 0x00
    CAN_COMMAND = 0x01
    TEACH_MODE = 0x02
    ETHERNET = 0x03
    WIFI = 0x04
    REMOTE = 0x05
    LINKAGE_TEACHING = 0x06
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


# The enum values correspond to the piper_sdk arm status codes found here:
# https://github.com/agilexrobotics/piper_sdk/blob/4eddfcf817cd87de9acee316a72cf5b988025378/piper_msgs/msg_v2/feedback/arm_status.py#L117
class ArmStatus(IntEnum):
    NORMAL = 0x00
    EMERGENCY_STOP = 0x01
    NO_SOLUTION = 0x02
    SINGULARITY = 0x03
    TARGET_ANGLE_EXCEEDS_LIMIT = 0x04
    JOINT_COMMUNICATION_EXCEPTION = 0x05
    JOINT_BRAKE_NOT_RELEASED = 0x06
    COLLISION = 0x07
    OVERSPEED_DURING_TEACHING = 0x08
    JOINT_STATUS_ABNORMAL = 0x09
    OTHER_EXCEPTION = 0x0A
    TEACHING_RECORD = 0x0B
    TEACHING_EXECUTION = 0x0C
    TEACHING_PAUSE = 0x0D
    MAIN_CONTROLLER_NTC_OVER_TEMPERATURE = 0x0E
    RELEASE_RESISTOR_NTC_OVER_TEMPERATURE = 0x0F


# The enum values correspond to the piper_sdk teach status codes found here:
# https://github.com/agilexrobotics/piper_sdk/blob/4eddfcf817cd87de9acee316a72cf5b988025378/piper_msgs/msg_v2/feedback/arm_status.py#L140
class TeachStatus(IntEnum):
    OFF = 0x00
    START_RECORD = 0x01
    END_RECORD = 0x02
    EXECUTE = 0x03
    PAUSE = 0x04
    CONTINUE = 0x05
    TERMINATE = 0x06
    MOVE_TO_START = 0x07


# The enum values correspond to the piper_sdk motion status codes found here:
# https://github.com/agilexrobotics/piper_sdk/blob/4eddfcf817cd87de9acee316a72cf5b988025378/piper_msgs/msg_v2/feedback/arm_status.py#L149
class MotionStatus(IntEnum):
    REACHED_TARGET = 0x00
    NOT_YET_REACHED_TARGET = 0x01


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


class PiperInterface:
    """
    A thin wrapper around the Piper robot SDK.

    This class provides a nicer bridge to the underlying API. The specific
    features over the core piper_sdk are:

    1. additional documentation
    2. enums for all message codes to aid readability
    3. hides the internal scaled integer units utilized by C_PiperInterface_V2 and
       using standard list of floats and SI units instead.
    4. get-state methods that return type-annotated primitive python types (list
       of floats) rather than internal piper_sdk types.
    """

    def __init__(
        self,
        can_port: str = "can0",
    ) -> None:
        """
        Initializes the PiperControl with a specified CAN port.

        Args:
          can_port (str): The CAN interface port name (e.g., "can0").
        """
        self.can_port = can_port

        self.piper = piper_sdk.C_PiperInterface_V2(can_name=can_port)
        self.piper.ConnectPort()

    def set_installation_pos(
        self, installation_pos: ArmInstallationPos = ArmInstallationPos.UPRIGHT
    ) -> None:
        """Sets the robot installation pose, call this right after connecting."""
        self.piper.MotionCtrl_2(0x01, 0x01, 0, 0, 0, installation_pos.value)

    def set_joint_zero_positions(self, joints: Sequence[int]) -> None:
        """
        Re-zeros the specified joints at their current positions.

        Args:
          joints (Sequence[int]): The indices of the joints to zero (zero-indexed).
        """

        def _validate_zero_indexed_joint(
            joint: int,
        ) -> TypeGuard[Literal[0, 1, 2, 3, 4, 5]]:
            return 0 <= joint <= 5

        if not all(_validate_zero_indexed_joint(j) for j in joints):
            raise ValueError(f"Invalid joint indices: {joints}")

        for joint in joints:
            self.piper.JointConfig(
                joint_num=joint + 1,  # type: ignore
                set_zero=0xAE,
                acc_param_is_effective=0,
                max_joint_acc=0,
                clear_err=0,
            )

    def set_emergency_stop(self, state: EmergencyStop):
        """Changes the emergency stop state on the arm.

        Depending on the state argument, this would trigger the stop or resume.
        """

        if not validate_emergency_stop(state):
            raise ValueError(f"Invalid emergency stop state {state}.")

        self.piper.MotionCtrl_1(state, 0, 0)

    def get_arm_status(self) -> piper_sdk.C_PiperInterface_V2.ArmStatus:
        """
        Gets the current arm status of the robot.

        Returns:
          ArmStatus: The current arm status.
        """
        return self.piper.GetArmStatus()

    @property
    def arm_status(self) -> ArmStatus:
        return ArmStatus(self.get_arm_status().arm_status.arm_status)

    @property
    def control_mode(self) -> ControlMode:
        return ControlMode(self.get_arm_status().arm_status.ctrl_mode)

    @property
    def motion_status(self) -> MotionStatus:
        return MotionStatus(self.get_arm_status().arm_status.motion_status)

    @property
    def teach_status(self) -> TeachStatus:
        return TeachStatus(self.get_arm_status().arm_status.teach_status)

    def get_gripper_status(
        self,
    ) -> piper_sdk.C_PiperInterface_V2.ArmGripper:
        """
        Gets the current gripper status of the robot.

        Returns:
          ArmGripperStatus: The current gripper status.
        """
        return self.piper.GetArmGripperMsgs()

    def get_end_effector_pose(self) -> list[float]:
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

    def get_joint_positions(self) -> list[float]:
        """
        Returns the current joint positions as a sequence of floats (radians).

        Returns:
          Sequence[float]: Joint positions in radians.
        """
        raw_positions = [
            self.piper.GetArmJointMsgs().joint_state.joint_1,
            self.piper.GetArmJointMsgs().joint_state.joint_2,
            self.piper.GetArmJointMsgs().joint_state.joint_3,
            self.piper.GetArmJointMsgs().joint_state.joint_4,
            self.piper.GetArmJointMsgs().joint_state.joint_5,
            self.piper.GetArmJointMsgs().joint_state.joint_6,
        ]

        # API reports positions in milli-degrees. Convert to radians.
        return [pos / 1e3 * DEG_TO_RAD for pos in raw_positions]

    def get_joint_velocities(self) -> list[float]:
        """
        Returns the current joint velocities as a sequence of floats.

        Returns:
          Sequence[float]: Joint velocities in radians per second.
        """
        raw_speeds = [
            self.piper.GetArmHighSpdInfoMsgs().motor_1.motor_speed,
            self.piper.GetArmHighSpdInfoMsgs().motor_2.motor_speed,
            self.piper.GetArmHighSpdInfoMsgs().motor_3.motor_speed,
            self.piper.GetArmHighSpdInfoMsgs().motor_4.motor_speed,
            self.piper.GetArmHighSpdInfoMsgs().motor_5.motor_speed,
            self.piper.GetArmHighSpdInfoMsgs().motor_6.motor_speed,
        ]

        # API reports speeds in milli-degrees. Convert to radians.
        return [speed / 1e3 * DEG_TO_RAD for speed in raw_speeds]

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
        gripper_status = self.get_gripper_status()

        raw_angle = gripper_status.gripper_state.grippers_angle
        raw_effort = gripper_status.gripper_state.grippers_effort

        angle = raw_angle / 1e6
        effort = raw_effort / 1e3

        return angle, effort

    def get_motor_errors(self) -> list[bool]:
        """Returns whether each of the 6 motors is in error."""
        arm_msgs = self.piper.GetArmLowSpdInfoMsgs()
        return [
            # motor_xx attributes are 1 indexed
            getattr(arm_msgs, f"motor_{i + 1}").foc_status.driver_error_status
            for i in range(6)
        ]

    def show_status(self, arm_status=None) -> None:
        """
        Prints a human friendly status of the arm and gripper.

        Args:
          arm_status: Optional arm status to display. If None, the current status
            will be fetched from the robot.
        """
        error_names = {False: "OK", True: "ERROR"}

        arm_status = self.get_arm_status()
        print("Arm Status:")
        print(f"ctrl_mode: {ControlMode(arm_status.arm_status.ctrl_mode).name}")
        print(f"arm_status: {ArmStatus(arm_status.arm_status.arm_status).name}")
        print(f"mode_feed: {MoveMode(arm_status.arm_status.mode_feed).name}")
        print(f"teach_mode: {TeachStatus(arm_status.arm_status.teach_status).name}")
        print(
            f"motion_status: {MotionStatus(arm_status.arm_status.motion_status).name}"
        )
        print(f"trajectory_num: {arm_status.arm_status.trajectory_num}")
        print(f"err_code: {arm_status.arm_status.err_code}")

        # https://github.com/agilexrobotics/piper_sdk/blob/4eddfcf817cd87de9acee316a72cf5b988025378/piper_msgs/msg_v2/feedback/arm_status.py#L228
        # Decoding err_code manually here because afaict the setter is not used to
        # update the err_status
        err_code = arm_status.arm_status.err_code
        limit_errors = [bool(err_code & (1 << b)) for b in range(6)]
        comms_errors = [bool(err_code & (1 << (b + 9))) for b in range(6)]
        motor_errors = self.get_motor_errors()
        for i in range(6):
            limit = limit_errors[i]
            comms = comms_errors[i]
            motor = motor_errors[i]
            print(
                f"  Joint {i + 1}:"
                f" limit={error_names[limit]}"
                f" comms={error_names[comms]}"
                f" motor={error_names[motor]}"
            )

        gripper_status = self.get_gripper_status()
        print("\nGripper Status:")
        foc_status = gripper_status.gripper_state.foc_status
        print(f"  voltage_too_low     : {error_names[foc_status.voltage_too_low]}")
        print(f"  motor_overheating   : {error_names[foc_status.motor_overheating]}")
        print(f"  driver_overcurrent  : {error_names[foc_status.driver_overcurrent]}")
        print(f"  driver_overheating  : {error_names[foc_status.driver_overheating]}")
        print(f"  sensor_status       : {error_names[foc_status.sensor_status]}")
        print(f"  driver_error_status : {error_names[foc_status.driver_error_status]}")
        print(f"  driver_enable_status: {foc_status.driver_enable_status}")
        print(f"  homing_status       : {error_names[foc_status.homing_status]}")

    def is_gripper_enabled(self) -> bool:
        """
        Checks if the gripper is enabled.
        """
        gripper_msgs = self.piper.GetArmGripperMsgs()
        return gripper_msgs.gripper_state.foc_status.driver_enable_status

    def is_arm_enabled(self) -> bool:
        """
        Checks if the robot arm is enabled.

        Returns:
          bool: True if the arm  and gripper are enabled, False otherwise.
        """
        arm_msgs = self.piper.GetArmLowSpdInfoMsgs()
        return (
            arm_msgs.motor_1.foc_status.driver_enable_status
            and arm_msgs.motor_2.foc_status.driver_enable_status
            and arm_msgs.motor_3.foc_status.driver_enable_status
            and arm_msgs.motor_4.foc_status.driver_enable_status
            and arm_msgs.motor_5.foc_status.driver_enable_status
            and arm_msgs.motor_6.foc_status.driver_enable_status
        )

    def is_enabled(self) -> bool:
        """
        Checks if the robot arm and gripper are enabled.

        Returns:
          bool: True if the arm and gripper are enabled, False otherwise.
        """
        return self.is_arm_enabled() and self.is_gripper_enabled()

    def enable_arm(self) -> None:
        self.piper.EnableArm(7)

    def enable_gripper(self) -> None:
        self.piper.GripperCtrl(0, 0, GripperCode.ENABLE, 0)

    def disable_arm(self) -> None:
        self.piper.DisableArm(7)

    def disable_gripper(self) -> None:
        self.piper.GripperCtrl(0, 0, GripperCode.DISABLE_AND_CLEAR_ERROR, 0)

    def standby(
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
        )

    def set_arm_mode(
        self,
        speed: int = 100,
        move_mode: MoveMode = MoveMode.JOINT,
        ctrl_mode: ControlMode = ControlMode.CAN_COMMAND,
        arm_controller: ArmController = ArmController.POSITION_VELOCITY,
    ) -> None:
        """
        Changes the arm motion control mode.

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
            ctrl_mode,  # type: ignore
            move_mode,
            speed,
            arm_controller,
        )

    def command_joint_positions(self, positions: Sequence[float]) -> None:
        """
        Sets the joint positions using JointCtrl.

        Note: The robot should be using POSITION_VELOCITY controller and JOINT move
        mode for this to work. Use the set_arm_mode() function for this.

        Args:
          positions (Sequence[float]): A list of joint angles in radians.
        """

        joint_angles = []

        for i, pos in enumerate(positions):
            min_rad, max_rad = JOINT_LIMITS_RAD["min"][i], JOINT_LIMITS_RAD["max"][i]
            clipped_pos = min(max(pos, min_rad), max_rad)
            pos_deg = clipped_pos * RAD_TO_DEG
            joint_angle = round(pos_deg * 1e3)  # Convert to millidegrees
            joint_angles.append(joint_angle)

        self.piper.JointCtrl(*joint_angles)  # pylint: disable=no-value-for-parameter

    def command_joint_position_mit(
        self,
        motor_idx: int,
        position: float,
        kp: float,
        kd: float,
    ) -> None:
        """
        Commands a joint via MIT control to move to a given angle.

        Note: The robot should be using MIT controller and MIT move mode for this to
        work. Use the set_arm_mode() function for this.

        Args:
          motor_idx (int): Motor index to control.
          position (float): Desired position in radians.
          kps (float): Proportional gain.
          kds (float): Derivative gain.
        """
        assert motor_idx >= 0 and motor_idx <= 5

        self.piper.JointMitCtrl(motor_idx + 1, position, 0.0, kp, kd, 0.0)

    def command_joint_torque_mit(
        self,
        motor_idx: int,
        position: float,
        torque: float,
    ) -> None:
        """
        Commands a joint via MIT control to move to a given angle.

        Note: The robot should be using MIT controller and MIT move mode for this to
        work. Use the set_arm_mode() function for this.

        Args:
          motor_idx (int): Motor index to control.
          torque (float): The torque command for the motor to execute.
        """
        assert motor_idx >= 0 and motor_idx <= 5
        self.piper.JointMitCtrl(motor_idx + 1, position, 0.0, 0.0, 0.0, torque)

    def command_cartesian_position(self, pose: Sequence[float]) -> None:
        """
        Sets the Cartesian position and orientation of the robot end-effector.

        Note: The robot should be using POSITION_VELOCITY controller and POSITION
        move mode for this to work. Use the set_arm_mode() function for this.

        Args:
            pose: [x, y, z, roll, pitch, yaw] in meters and radians.
        """
        assert len(pose) == 6

        x_mm = round(pose[0] * 1e6)
        y_mm = round(pose[1] * 1e6)
        z_mm = round(pose[2] * 1e6)
        roll_deg = round(pose[3] * RAD_TO_DEG * 1e3)
        pitch_deg = round(pose[4] * RAD_TO_DEG * 1e3)
        yaw_deg = round(pose[5] * RAD_TO_DEG * 1e3)
        self.piper.EndPoseCtrl(x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg)

    def command_gripper(
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

        self.piper.GripperCtrl(position_int, effort_int, GripperCode.ENABLE, 0)
