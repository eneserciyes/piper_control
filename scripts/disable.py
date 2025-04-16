from piper_control import piper_control
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Move the arm to zero position.")
    parser.add_argument(
        "--can_port",
        type=str,
        default="can_left",
        help="The CAN port to use (default: can_left).",
    )
    args = parser.parse_args()
    robot = piper_control.PiperControl(can_port=args.can_port)
    robot.disable()
