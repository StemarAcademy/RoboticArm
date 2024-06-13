# RAConfig: A Python Library for Controlling a Robotic Arm

`RAConfig` is a Python library designed for controlling a robotic arm through the SSC-32U servo controller. This library provides methods to perform inverse kinematics calculations and move the arm to specified coordinates, as well as control the gripper.

## Features

- **Inverse Kinematics:** Compute the angles required for each joint to move the arm to a specified position.
- **Movement Control:** Move the arm to a specified position with customizable speed and effector angle.
- **Gripper Control:** Open or close the gripper, optionally using a force sensor to determine when an object is securely held.

## Requirements

- Python 3.x
- `pyserial` library (for serial communication)

## Installation

Install the required `pyserial` library using pip:

```bash
pip install pyserial
```

## Usage

### Initialization

To initialize the `RAConfig` class, specify the serial port connected to the SSC-32U servo controller:

```python
from ra_config import RAConfig

# Replace 'COM3' with the appropriate serial port for your setup
ra = RAConfig(serial_port='COM3')
```

### Moving the Arm

**Move to a Specific Position**

To move the robotic arm to a specific position `(x, y, z)`, use the `move` method:

```python
# Move the arm to coordinates (x=10, y=20, z=15)
ra.move(x=10, y=20, z=15)
```

Optional parameters:

- `speed` (int): Speed to move from the old position to the new one (default is 700).
- `effector_horizontal_degree` (float): Degree of the effector relative to the ground, ranging from 0 to 180 degrees (default is 90).


**Move from One Position to Another**

To move the arm smoothly from one position to another, use the `move_fromto` method:

```python
# Move the arm from coordinates (x1=0, y1=0, z1=0) to (x2=10, y2=20, z2=15)
ra.move_fromto(x1=0, y1=0, z1=0, x2=10, y2=20, z2=15)
```

Optional parameters:

- `step` (float): Step size for moving from the first coordinates to the second (default is 0.1).
- `speed` (int): Speed to move from the old position to the new one (default is 700).
- `effector_horizontal_degree` (float): Degree of the effector relative to the ground, ranging from 0 to 180 degrees (default is 90).

## Controlling the Gripper

To control the gripper, use the `gripper` method:

```python
# Open the gripper to 180 degrees
ra.gripper(state=True, gripper_degree=180)

# Close the gripper until an object is securely held, with a maximum strength of 20
ra.gripper(state=False, max_strength=20)
```

Optional parameters:

- `state` (bool): Set to `True` if only specifying a degree for the gripper. Set to `False` to close the gripper until an object is held.
- `gripper_degree` (float): Degree to open the gripper (default is 180).
- `max_strength` (int): Maximum force sensor value before stopping the gripper (default is 20).

### Example

Here's a complete example demonstrating how to initialize the `RAConfig` class, move the arm, and control the gripper:

```python
from ra_config import RAConfig

# Initialize the robotic arm
ra = RAConfig(serial_port='COM3')

# Move the arm to a specific position
ra.move(x=10, y=20, z=15, speed=600, effector_horizontal_degree=90)

# Move the arm from one position to another
ra.move_fromto(x1=0, y1=0, z1=0, x2=10, y2=20, z2=15, step=0.2, speed=500)

# Control the gripper
ra.gripper(state=True, gripper_degree=150)
ra.gripper(state=False, max_strength=30)
```

This library provides a straightforward way to control a robotic arm using Python, making it accessible for robotics enthusiasts and developers. Enjoy building and controlling your robotic arm!
