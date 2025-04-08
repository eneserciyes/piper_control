# Overview

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
section on how to debug the CAN connection: TODO.

```text
TODO - copy message not sent output
```

If you get output that looks like this (as in the CAN seems like it is working,
but you get all 0's for the rest of the output), see this section: TODO -
link to the Get all zeros section.

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
