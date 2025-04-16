"""Sets the zero position of the robot arm."""

import argparse
import time

from piper_control import piper_control


def main():
    print("This script will set the zero position of the robot arm.")
    parser = argparse.ArgumentParser(description="Move the arm.")
    parser.add_argument(
        "--can_port",
        type=str,
        default="can_left",
        help="The CAN port to use (default: can_left).",
    )
    parser.add_argument(
        "--joint_motor_num",
        type=int,
        default=7,
        help="The joint motor number to set the zero position of (default: 7 - all joints).",
    )
    args = parser.parse_args()
    robot = piper_control.PiperControl(can_port=args.can_port)
    robot.reset(enable_arm=False)
    time.sleep(2.0)
    input("Move the joints to the zero position. Press Enter to continue...")
    robot.set_zero_position(args.joint_motor_num)
    print("Done.")


if __name__ == "__main__":
    main()
