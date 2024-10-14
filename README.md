# Bio IK2 for ROS2

This repository contains a fork of the [Bio IK2 project](https://github.com/PickNikRobotics/bio_ik) by PickNik Robotics, adapted for ROS2.
It contains also definitions of more custom goals (cost functions) for Bio IK2.

## Contents

- [bio_ik2](bio_ik/README.md) - The core library for Bio IK2.
- [bio_ik2_custom_costs](bio_ik_custom_costs/README.md) - Custom goals (cost functions) for Bio IK2.

## Installation

1. Clone this repository into your ROS2 workspace.

```bash
cd /path/to/your/ros2/workspace/src
git clone https://github.com/LeoBoticsHub/bio_ik2_ros2.git
```

2. Build the workspace.

```bash
cd /path/to/your/ros2/workspace
colcon build
```

3. Source the workspace.

```bash
source /path/to/your/ros2/workspace/install/setup.bash
```

## Usage

The Bio IK2 library can be used to solve inverse kinematics problems for robotic manipulators. 
The custom goals provided in `bio_ik2_custom_costs` can be used to define custom cost functions for the optimization problem.
The custom cost functions are useful for redundant manipulators, where the user can define additional constraints to be satisfied by the solution,
in addition to the standard constraints (e.g. joint limits). Several custom cost functions are provided in the `bio_ik2_custom_costs` package,
and the user can define their own custom cost functions by extending the `bio_ik::Goal` class defined in `bio_ik/include/goal_types.h`.