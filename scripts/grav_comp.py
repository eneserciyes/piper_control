"""Example of using the MitPositionController to move the arm to a fixed pose.

To run this example:
python3 scripts/simple_move.py
"""

import time

import mujoco
import numpy as np

# import pinocchio as pin
from piper_control import piper_connect, piper_control, piper_init, piper_interface


def main():
    print(
        "This script will move the 2nd-to-last joint of the robot arm a small "
        "amount, using position control."
    )
    input("WARNING: the robot will move. Press Enter to continue...")

    # ports = piper_connect.find_ports()
    # print(f"Piper ports: {ports}")

    # piper_connect.activate(ports)
    # ports = piper_connect.active_ports()

    # if not ports:
    #   raise ValueError(
    #       "No ports found. Make sure the Piper is connected and turned on. "
    #       "If you are having issues connecting to the piper, check our "
    #       "troubleshooting guide @ "
    #       "https://github.com/Reimagine-Robotics/piper_control/blob/main/README.md"
    #   )

    model = mujoco.MjModel.from_xml_path(
        "/home/enes/ws/robot/robot/cone-e-description/arm-upright.mjcf"
    )

    data = mujoco.MjData(model)

    robot = piper_interface.PiperInterface(can_port="can_right")
    robot.set_installation_pos(piper_interface.ArmInstallationPos.UPRIGHT)

    print("resetting arm")
    piper_init.reset_arm(
        robot,
        arm_controller=piper_interface.ArmController.MIT,
        move_mode=piper_interface.MoveMode.MIT,
    )

    # print("resetting gripper")
    # piper_init.reset_gripper(robot)

    robot.show_status()

    # First open and close the gripper
    # with piper_control.GripperController(robot) as controller:
    #   controller.command_open()
    #   time.sleep(2.0)
    #   controller.command_close()
    #   time.sleep(2.0)

    # Move the arm joints using Mit mode controller.
    input("Press Enter to move arm...")
    log_tff = []
    log_tau = []
    with piper_control.MitJointPositionController(
        robot,
        kp_gains=5.0,
        kd_gains=0.8,
        rest_position=piper_control.REST_POSITION,
    ) as controller:
        try:
            last_tau = np.zeros(6)
            while True:
                q = robot.get_joint_positions()
                tau = np.array(robot.get_joint_efforts())
                last_tau = last_tau * 0.95 + tau * 0.05

                data.qpos[-6:] = q
                data.qfrc_bias[-6:] = 0.0
                mujoco.mj_forward(model, data)

                tff = (
                    np.array([2.16, 2.16, 2.18, 2, 2, 2])
                    * data.qfrc_bias[-6:].copy()
                    / np.array([4, 3.97, 3.5, 0.6, 0.6, 0.6])
                )
                log_tff.append(tff)
                log_tau.append(last_tau)

                controller.command_torques(tff)
                time.sleep(0.005)
        except KeyboardInterrupt:
            pass
        # print(data.qfrc_actuator)
        finally:
            controller.command_torques(np.zeros(6).tolist())
            print("Stopping controller and disabling arm...")
            np.savez(
                "grav_comp_data_commanded.npz",
                tff=np.array(log_tff),
                tau=np.array(log_tau),
            )

    print("finished, disabling arm.")
    print("WARNING: the arm will power off and drop.")

    input("Press Enter to disable arm...")
    piper_init.disable_arm(robot)


if __name__ == "__main__":
    main()
