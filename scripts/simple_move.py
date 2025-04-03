"""Moves the 2nd-to-last joint of the robot arm a small amount."""

import argparse
import time

from piper_control import piper_control


def main():
  print(
      "This script will move the 2nd-to-last joint of the robot arm a small "
      "amount, using position control."
  )
  input("Warning: the robot will move. Press Enter to continue...")
  parser = argparse.ArgumentParser(description="Move the arm.")
  parser.add_argument(
      "--can_port",
      type=str,
      default="can0",
      help="The CAN port to use (default: can0).",
  )
  args = parser.parse_args()
  robot = piper_control.PiperControl(can_port=args.can_port)
  robot.reset()
  time.sleep(1.0)
  joint_angles = list(robot.get_joint_positions())
  joint_angles[-2] -= 0.1
  print(f"Setting joint angles to {joint_angles}")
  robot.set_joint_positions(joint_angles)


if __name__ == "__main__":
  main()
