#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class ManipulatorController(object):
    """
    """
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('manipulator_controller', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        self.arm_move_group = moveit_commander.MoveGroupCommander("arm")
        self.gripper_move_group = moveit_commander.MoveGroupCommander("gripper")

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)

        # We can get the name of the reference frame for this robot:
        planning_frame = self.arm_move_group.get_planning_frame()
        print("============ Reference frame: {}".format(planning_frame))

        # We can also print the name of the end-effector link for this group:
        eef_link = self.arm_move_group.get_end_effector_link()
        print("============ End effector: {}".format(eef_link))

        # We can get a list of all the groups in the robot:
        group_names = self.robot.get_group_names()
        print("============ Robot Groups: {}".format(group_names))

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state: {}".format(self.robot.get_current_state()))

    def interface(self, command):
        if command == "gripper_open":
            self.set_tool_control(0.01)
        elif command == "gripper_close":
            self.set_tool_control(-0.01)
        elif command == "init_pose":
            pose = [0.0, 0.2, 0.0, 0.3]
            self.set_task_space_path(pose)
        elif command == "home_pose":
            pose = [0.0, 0.0, 0.0, 0.35]
            self.set_task_space_path(pose)

    def set_task_space_path(self, pose):
        self.arm_move_group.clear_pose_targets()

        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation.w = pose[0]
        pose_target.position.x = pose[1]
        pose_target.position.y = pose[2]
        pose_target.position.z = pose[3]

        self.arm_move_group.set_pose_target(pose_target)

        plan1 = self.arm_move_group.plan()
        rospy.sleep(1)

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan1)
        self.display_trajectory_publisher.publish(display_trajectory);

        self.arm_move_group.go(wait=True)
        self.arm_move_group.stop()
        self.arm_move_group.clear_pose_targets()

    def set_joint_space_path(self):
        self.arm_move_group.clear_pose_targets()

        joint_goal = self.arm_move_group.get_current_joint_values()
        joint_goal[0] = 1
        joint_goal[1] = 0.087   #0.037
        joint_goal[2] = 0.068   #0.028
        joint_goal[3] = 0.003   #0.003

        self.arm_move_group.go(joint_goal, wait=True)
        self.arm_move_group.stop()
        self.arm_move_group.clear_pose_targets()

    def set_tool_control(self, joint_value):
        self.gripper_move_group.clear_pose_targets()

        joint_goal = self.gripper_move_group.get_current_joint_values()
        joint_goal[0] = joint_value
        joint_goal[1] = joint_value
        self.gripper_move_group.set_joint_value_target(joint_goal)

        plan2 = self.gripper_move_group.plan()

        self.gripper_move_group.go(wait=True)
        self.gripper_move_group.stop()
        self.gripper_move_group.clear_pose_targets()

if __name__ == '__main__':
    manipulator_controller = ManipulatorController()
    # manipulator_controller.interface("gripper_open")
    # manipulator_controller.interface("gripper_close")
    # manipulator_controller.interface("init_pose")
    # manipulator_controller.interface("home_pose")
    manipulator_controller.set_joint_space_path()