# piper_control - Library for controlling AgileX Pipers

## Overview

This repo provides low-level python modules for connecting and controlling
AgileX Piper robots.

*   `piper_connect`: a python implementation of the CAN setup scripts bundled
    with `piper_sdk`. This are not accessible on pip installs, and we found it
    useful to be able to query and activate CAN ports programmatically.

*   `piper_control`:  our lightweight wrapper of `piper_sdk` for controlling
    AgileX Piper robots.

  The `piper_sdk` API is powerful and quickly maturing, but it's a bit complex
  and under-documented, and we found it helpful to define a simple abstraction
  for basic I/O.

  There are also several sharp bits in `piper_sdk` which can make the robots
  seem tempermental, e.g. becoming unresponsive despite repeated calls to
  `MotionCtrl_2`, `EnableArm`, `GripperCtrl`, etc. We've bundled our solutions
  into `PiperControl` so `reset` and the various move commands perform as one
  would expect.

## Quick start

Install the dependencies and package:

```shell
sudo apt install can-utils
pip install piper_control
```

Set up the CAN connection to the arm(s):

```python
# Set up the connection to the Piper arm.
# These steps require sudo access.
from piper_control import piper_connect

# Print out the CAN ports that are available to connect.
print(piper_connect.find_ports())

# Activate all the ports so that you can connect to any arms connected to your
# machine.
piper_connect.activate()

# Check to see that all the ports are active.
print(piper_connect.active_ports())
```

Then control the robot:

```python
from piper_control import piper_control

robot = piper_control.PiperControl(can_port="can0")

# Resets the robot and enables the motors and motion controller for the arm.
# This call is necessary to be able to both query state and send commands to the
# robot.
robot.reset()

# See where the robot is now.
joint_angles = robot.get_joint_positions()
print(joint_angles)

# Move one joint of the arm.
joint_angles = robot.get_joint_positions()
joint_angles[-2] -= 0.1
print(f"Setting joint angles to {joint_angles}")
robot.set_joint_positions(joint_angles)
```

See the [tutorial.ipynb][tutorial] for a longer walkthrough.

## Troubleshooting / FAQ

### Is my PiperControl working?

This snippet is a good check for whether things are working:

```python
from piper_control import piper_control

robot = piper_control.PiperControl(can_port="can0")

robot.enable()
print(robot.get_joint_positions())
print(robot.get_joint_velocities())
print(robot.get_joint_efforts())
print(robot.get_gripper_state())
```

If you get the following text, then the CAN connection is not working. See this
section on how to debug the CAN connection: TODO(akhil) - add CAN debugging
section.

```text
TODO - copy message not sent output
```

If you get output that looks like this (as in the CAN seems like it is working,
but you get all 0's for the rest of the output), see
[this section](#get-all-zeros-when-calling-get_joint_positions).

```text
can0  is exist
can0  is UP
can0  bitrate is  1000000
can0 bus opened successfully.
[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
(0.0, 0.0)
```

### Get all zeros when calling `get_joint_positions()`

Run through these steps:

```python
# Assume you already have the PiperControl object.
piper = piper_control.PiperControl("can0")

# Enable, reset, then re-enable the robot.
piper.enable()
time.sleep(0.5)
piper.reset()
time.sleep(0.5)
piper.enable()
```

And after that, calling `piper.get_joint_positions()` should return non-zero
values. If it doesn't, then double-check the CAN connection. See TODO -
can debug section link.

[tutorial]: <https://github.com/Reimagine-Robotics/piper_control/blob/main/tutorial.ipynb> "Tutorial"
