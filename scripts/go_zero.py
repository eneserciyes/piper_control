"""Moves the robot arm to its zero position."""

import argparse
import time

from loop_rate_limiters import RateLimiter

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

    count = 0
    rate_limiter = RateLimiter(200.0)
    while True:
        robot.set_joint_positions([0.0] * 6)
        if count == 0:
            print("1-----------")
            position = [0, 0, 0, 0, 0, 0]
            # position = [0.2,0.2,-0.2,0.3,-0.2,0.5,0.08]
        elif count == 600:
            print("2-----------")
            position = [0.2, 0.2, -0.2, 0.3, -0.2, 0.5]
            # position = [0,0,0,0,0,0,0]
            # position = [-8524,104705,-78485,-451,-5486,29843,0]
        elif count == 1200:
            print("1-----------")
            position = [0, 0, 0, 0, 0, 0]
            # position = [0.2,0.2,-0.2,0.3,-0.2,0.5,0.08]
            count = 0

        robot.set_joint_positions(position)
        count += 1
        time.sleep(0.005)
        # rate_limiter.sleep()


if __name__ == "__main__":
    main()
