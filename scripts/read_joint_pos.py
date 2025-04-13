import argparse

from loop_rate_limiters import RateLimiter

from piper_control import piper_control


def main():
    parser = argparse.ArgumentParser(description="Move the arm.")
    parser.add_argument(
        "--can_port",
        type=str,
        default="can_left",
        help="The CAN port to use (default: can_left).",
    )
    args = parser.parse_args()

    rate_limiter = RateLimiter(10)
    robot = piper_control.PiperControl(can_port=args.can_port)
    robot.reset(enable_arm=False)
    while True:
        joint_positions = robot.get_joint_positions()
        print(joint_positions)
        rate_limiter.sleep()


if __name__ == "__main__":
    main()
