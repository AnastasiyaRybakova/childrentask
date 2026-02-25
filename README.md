# OpenMANIPULATOR-X Pick & Place and Drawing Task Framework (ROS)

Package of childrentask is a ROS-based task execution framework for controlling the OpenMANIPULATOR-X robotic arm in structured manipulation scenarios, including:

- Pick & Place routines

- Predefined drawing trajectories

- Educational interaction tasks

 ## Requirements

- Ubuntu 18.04 (ROS Melodic) or Ubuntu 20.04 (ROS Noetic)
- ROS installed and configured
- MoveIt!
- OpenMANIPULATOR-X ROS packages
For official hardware specifications and platform overview, see the  
OpenMANIPULATOR-X documentation:  
https://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview/

## Workspace Calibration

The drawing surface is aligned with the robot base coordinate frame.

- Workspace origin (0.0, 0.0) corresponds to the lower-left corner of the A4 sheet.
- Drawing surface height: 0.06 m.
- Pen offset from tool center: 0.025 m.
- Robot base is fixed to a modular platform to prevent displacement during execution.

Accurate calibration is required to ensure consistent drawing precision and stable grasp execution.

## Motion Planning Strategy

- Drawing trajectories are defined as Cartesian waypoints.
- Waypoints are converted into joint-space trajectories using MoveIt.
- Linear interpolation is applied between drawing segments.
- Pick & place tasks use predefined grasp and placement poses.
- Gripper control is performed via dedicated ROS service calls.

## Hardware Configuration

- Robot: OpenMANIPULATOR-X (4 DOF + gripper)
- Pen mounted on motor ID14
- External monitor used for MoveIt visualization
- Modular base for vibration reduction
- AR marker-based cube detection (if applicable)
