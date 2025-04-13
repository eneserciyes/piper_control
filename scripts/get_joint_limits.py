"""Gets the joint limits of the robot arm."""

import argparse
import time

from piper_control import piper_control

DEG2RAD = 0.017453292519943295


def main():
    print("This script will get the joint limits of the robot arm.")
    parser = argparse.ArgumentParser(description="Get the joint limits.")
    parser.add_argument(
        "--can_port",
        type=str,
        default="can_left",
        help="The CAN port to use (default: can_left).",
    )
    args = parser.parse_args()
    robot = piper_control.PiperControl(can_port=args.can_port)
    # robot.reset()
    time.sleep(1.0)
    limits = robot.piper.GetAllMotorAngleLimitMaxSpd()
    print(limits)


if __name__ == "__main__":
    main()
