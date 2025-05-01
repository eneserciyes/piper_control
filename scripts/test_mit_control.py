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
    parser.add_argument(
        "--no-gripper",
        help="Whether to use the gripper (default: False).",
        action="store_true",
    )

    args = parser.parse_args()

    robot = piper_control.PiperControl(
        can_port=args.can_port, gripper_on=not args.no_gripper
    )
    robot.reset()
    time.sleep(2.0)

    count = 0
    rate_limiter = RateLimiter(200.0)
    while True:
        robot.set_joint_mit_ctrl(
            positions=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            velocities=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            kps=[10.0, 10.0, 10.0, 10.0, 10.0, 10.0],
            kds=[1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
            efforts=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        rate_limiter.sleep()


if __name__ == "__main__":
    main()
