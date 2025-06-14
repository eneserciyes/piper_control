import time

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


if __name__ == "__main__":
    import numpy as np

    piper = C_PiperInterface_V2("can_right")
    piper.ConnectPort()
    piper.EnableArm(7)
    enable_fun(piper=piper, enable=True)
    time.sleep(0.001)

    # joint1_pos = (piper.GetArmJointMsgs().joint_state.joint_1 / 1000) * (np.pi / 180)
    # print(f"joint1_pos: {joint1_pos}")

    # last_effort = 0.0
    # while True:
    #     effort = piper.GetArmHighSpdInfoMsgs().motor_2.effort / 1000
    #     last_effort = last_effort * 0.99 + (effort) * 0.01
    #     time.sleep(0.01)
    #     print(f"last_effort: {last_effort}")
    piper.MotionCtrl_2(0x01, 0x04, 0, 0xAD, installation_pos=0x00)

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
        piper.JointMitCtrl(1, joint1_pos, 0, 0.01, 0.01, 0.05)
        time.sleep(0.001)
        piper.JointMitCtrl(2, joint2_pos, 0, 0.01, 0.01, 0.7)
        time.sleep(0.001)
        piper.JointMitCtrl(3, joint3_pos, 0, 0.01, 0.01, 0.05)
        time.sleep(0.001)
        piper.JointMitCtrl(4, joint4_pos, 0, 0.01, 0.01, 0.05)
        time.sleep(0.001)
        piper.JointMitCtrl(5, joint5_pos, 0, 0.01, 0.01, 0.05)
        time.sleep(0.001)
        piper.JointMitCtrl(6, joint6_pos, 0, 0.01, 0.01, 0.05)
        time.sleep(0.001)
