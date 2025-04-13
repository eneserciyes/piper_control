from piper_control import piper_control

if __name__ == "__main__":
    robot = piper_control.PiperControl(can_port="can_left")
    robot.disable()
