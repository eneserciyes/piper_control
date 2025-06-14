"""Example of using the MitPositionController to move the arm to a fixed pose.

To run this example:
python3 scripts/simple_move.py
"""

# import pinocchio as pin
# from piper_control import piper_connect, piper_control, piper_init, piper_interface
import time

import mujoco
import numpy as np
from piper_sdk import C_PiperInterface_V2


def enable_fun(piper: C_PiperInterface_V2, enable: bool):
    """
    使能机械臂并检测使能状态,尝试5s,如果使能超时则退出程序
    """
    enable_flag = False
    loop_flag = False
    # 设置超时时间（秒）
    timeout = 5
    # 记录进入循环前的时间
    start_time = time.time()
    elapsed_time_flag = False
    while not (loop_flag):
        elapsed_time = time.time() - start_time
        print(f"--------------------")
        enable_list = []
        enable_list.append(
            piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status
        )
        enable_list.append(
            piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status
        )
        enable_list.append(
            piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status
        )
        enable_list.append(
            piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status
        )
        enable_list.append(
            piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status
        )
        enable_list.append(
            piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status
        )
        if enable:
            enable_flag = all(enable_list)
            piper.EnableArm(7)
            piper.GripperCtrl(0, 1000, 0x01, 0)
        else:
            enable_flag = any(enable_list)
            piper.DisableArm(7)
            piper.GripperCtrl(0, 1000, 0x02, 0)
        print(f"使能状态: {enable_flag}")
        print(f"--------------------")
        if enable_flag == enable:
            loop_flag = True
            enable_flag = True
        else:
            loop_flag = False
            enable_flag = False
        # 检查是否超过超时时间
        if elapsed_time > timeout:
            print(f"超时....")
            elapsed_time_flag = True
            enable_flag = False
            loop_flag = True
            break
        time.sleep(0.5)
    resp = enable_flag
    print(f"Returning response: {resp}")
    return resp


def main():
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
    piper = C_PiperInterface_V2("can_right")
    piper.ConnectPort()
    piper.EnableArm(7)
    enable_fun(piper=piper, enable=True)

    model = mujoco.MjModel.from_xml_path(
        "/home/enes/ws/robot/robot/cone-e-description/arm-upright.mjcf"
    )

    data = mujoco.MjData(model)

    # robot.set_installation_pos(piper_interface.ArmInstallationPos.UPRIGHT)

    # print("resetting arm")
    # piper_init.reset_arm(
    #     robot,
    #     arm_controller=piper_interface.ArmController.MIT,
    #     move_mode=piper_interface.MoveMode.MIT,
    # )

    # print("resetting gripper")
    # piper_init.reset_gripper(robot)

    # First open and close the gripper
    # with piper_control.GripperController(robot) as controller:
    #   controller.command_open()
    #   time.sleep(2.0)
    #   controller.command_close()
    #   time.sleep(2.0)

    # Move the arm joints using Mit mode controller.
    input("Press Enter to move arm...")
    tau_last = np.zeros(6)
    log_tff = []
    log_tau = []
    piper.MotionCtrl_2(0x01, 0x04, 0, 0xAD, installation_pos=0x00)
    try:
        while True:
            joint1_pos = (piper.GetArmJointMsgs().joint_state.joint_1 / 1000) * (
                np.pi / 180
            )
            joint2_pos = (piper.GetArmJointMsgs().joint_state.joint_2 / 1000) * (
                np.pi / 180
            )
            joint3_pos = (piper.GetArmJointMsgs().joint_state.joint_3 / 1000) * (
                np.pi / 180
            )
            joint4_pos = (piper.GetArmJointMsgs().joint_state.joint_4 / 1000) * (
                np.pi / 180
            )
            joint5_pos = (piper.GetArmJointMsgs().joint_state.joint_5 / 1000) * (
                np.pi / 180
            )
            joint6_pos = (piper.GetArmJointMsgs().joint_state.joint_6 / 1000) * (
                np.pi / 180
            )
            q = np.array(
                [joint1_pos, joint2_pos, joint3_pos, joint4_pos, joint5_pos, joint6_pos]
            )
            print("Current joint positions (rad):", q.round(4))
            joint1_effort = piper.GetArmHighSpdInfoMsgs().motor_1.effort / 1000
            joint2_effort = piper.GetArmHighSpdInfoMsgs().motor_2.effort / 1000
            joint3_effort = piper.GetArmHighSpdInfoMsgs().motor_3.effort / 1000
            joint4_effort = piper.GetArmHighSpdInfoMsgs().motor_4.effort / 1000
            joint5_effort = piper.GetArmHighSpdInfoMsgs().motor_5.effort / 1000
            joint6_effort = piper.GetArmHighSpdInfoMsgs().motor_6.effort / 1000
            tau = np.array(
                [
                    joint1_effort,
                    joint2_effort,
                    joint3_effort,
                    joint4_effort,
                    joint5_effort,
                    joint6_effort,
                ]
            )
            tau_last = tau_last * 0.95 + tau * 0.05
            print("Current joint efforts (Nm):", tau_last.round(4))

            data.qpos[-6:] = q
            data.qfrc_bias[-6:] = 0.0
            mujoco.mj_forward(model, data)

            tff = data.qfrc_bias[-6:].copy()
            print("Torque feedforward (Nm):", tff.round(4))
            log_tff.append(tff)
            log_tau.append(tau_last)

            time.sleep(0.005)
    except KeyboardInterrupt:
        print("Keyboard interrupt received, stopping logging.")
    finally:
        print("Logging complete. Saving data...")
        np.savez("grav_comp_data.npz", log_tff=log_tff, log_tau=log_tau)


if __name__ == "__main__":
    main()
