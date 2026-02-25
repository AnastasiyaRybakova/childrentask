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

## 📸 Experimental Results

### Pick & Place Execution

The OpenMANIPULATOR-X arm performs marker-based cube manipulation during structured interaction tasks.
![IMG_4533](https://github.com/user-attachments/assets/1457a1a7-cb88-456a-a646-35aacae43303)

---

### Drawing Task Output

The robot executes trajectory-based drawing tasks using a mounted marker pen.

![IMG_4532](https://github.com/user-attachments/assets/28310ea0-4eac-45cb-8ff6-567f74f7d9f7)


Drawing trajectories are generated from predefined Cartesian waypoints and converted into joint-space motion using MoveIt planning.

## Workspace Calibration

For consistent drawing and pick & place performance, the robot workspace must be calibrated:

- The drawing surface (e.g., A4 sheet) is aligned with the robot base coordinate frame. Later we replaced the white sheet by whiteboard.
- Surface height is set to 0.06 m relative to the robot base.
- The pen offset from the tool center is 0.025 m.
- Workspace origin (0,0) corresponds to the lower-left corner of the drawing surface.

Accurate calibration ensures reproducible trajectories and minimizes positional error during execution.

## Motion Planning Overview

This project uses MoveIt! for motion planning:

- Drawing tasks are defined as Cartesian waypoints.
- Waypoints are converted into joint-space trajectories using MoveIt.
- Linear interpolation is used between drawing segments for smooth motion.
- Pick & place tasks reference predefined grasp and placement poses.
- Gripper actuation is handled via dedicated ROS service calls.

All motion planning is executed in the ROS ecosystem, enabling visualization and debugging in RViz.

## Limitations & Safety

### Limitations

- Drawing precision depends on accurate surface calibration.
- No dynamic obstacle avoidance is implemented.
- Marker-based object localization can fail in poor lighting.
- Drawing and pick & place repeatability may vary without further tuning.

### Safety

- Ensure the workspace is clear of obstacles before execution.
- Verify joint limits and MoveIt planning scene before running commands.
- For hardware tests, reduce velocity and acceleration limits appropriately.
- Use hardware emergency stop during early testing phases.

## Related Publications

The manipulation and task framework described in this repository is related to the following peer-reviewed works:

- Rybakova, A., & Choi, J. (2025). *Evaluating the Effectiveness of Social Robots in Enhancing English Language Acquisition and Educational Engagement: A Study with Adults and Its Implications for Korean Kindergarten Children.* In Human-Computer Interaction – HCII 2025 (LNCS). Springer.

- Rybakova, A., & Choi, J. (2026). *From Traditional to Robot-Assisted Learning: A Multimodal Robot-Assisted Learning Framework for Enhancing English Acquisition in Korean Preschoolers.* Intelligent Service Robotics, 19, 30. https://doi.org/10.1007/s11370-025-00685-z

- Rybakova, A., & Choi, J. (2025). *A Multi-Modal Embodied Robot Framework for English as a Second Language Learning in Preschoolers: Design and Evaluation.* Robotica, 43(11), 4133–4151. https://doi.org/10.1017/S0263574725102646
