"""Moves the robot arm to its zero position."""

import argparse
import time

from piper_control import piper_control


def main():
    print("This script will move the robot arm to its zero position.")
    input("Warning: the robot will move. Press Enter to continue...")

    parser = argparse.ArgumentParser(description="Move the arm to zero position.")
    parser.add_argument(
        "--can_port",
        type=str,
        default="can_left",
        help="The CAN port to use (default: can_left).",
    )
    args = parser.parse_args()

    robot = piper_control.PiperControl(can_port=args.can_port)
    robot.reset()
    time.sleep(2.0)

    robot.set_joint_positions([0.0] * 6)
    print("Done.")


if __name__ == "__main__":
    main()
