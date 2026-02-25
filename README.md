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

## Related Publications

The manipulation and task framework described in this repository is related to the following peer-reviewed works:

- Rybakova, A., & Choi, J. (2025). *Evaluating the Effectiveness of Social Robots in Enhancing English Language Acquisition and Educational Engagement: A Study with Adults and Its Implications for Korean Kindergarten Children.* In Human-Computer Interaction – HCII 2025 (LNCS). Springer.

- Rybakova, A., & Choi, J. (2026). *From Traditional to Robot-Assisted Learning: A Multimodal Robot-Assisted Learning Framework for Enhancing English Acquisition in Korean Preschoolers.* Intelligent Service Robotics, 19, 30. https://doi.org/10.1007/s11370-025-00685-z

- Rybakova, A., & Choi, J. (2025). *A Multi-Modal Embodied Robot Framework for English as a Second Language Learning in Preschoolers: Design and Evaluation.* Robotica, 43(11), 4133–4151. https://doi.org/10.1017/S0263574725102646
