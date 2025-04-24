#!/usr/bin/env python 
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import std_msgs.msg
import sensor_msgs.msg
from std_msgs.msg import Header
from tf.transformations import *
import geometry_msgs.msg
import math 
from math import pi
import time


def calculate_orientation(position):
    orientation = geometry_msgs.msg.Quaternion()
    yaw = math.atan2(position.y, position.x + 0.08)
    quat = quaternion_from_euler(0, 0, yaw)
    orientation.x = quat[0]
    orientation.y = quat[1]
    orientation.z = quat[2]
    orientation.w = quat[3]
    return orientation

def circle(radius,res,start):
   waypoints = []
   #waypoints.append(start)
   for i in range(0,res):
      point = copy.deepcopy(start)
      point.position.x += radius*math.cos((2*math.pi/res)*i)
      point.position.z += radius*math.sin((2*math.pi/res)*i)
      #point.position.z =.2
      #point.orientation = calculate_orientation(point.position)
      waypoints.append(copy.deepcopy(point))
   #waypoints.append(start)
   return waypoints                

if __name__ == "__main__":
  
  # Initialize moveit_commander and node
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_circle', anonymous=True)  #initialize moveit_commander

  # Get instance from moveit_commander
  robot = moveit_commander.RobotCommander()    #robot commander to interface to the robot
  scene = moveit_commander.PlanningSceneInterface()
  arm_group = moveit_commander.MoveGroupCommander('arm')      #moving group commander

  
  ## creating a display trajextory publisher which is used later to publish trajectories for Rviz to visualize
  display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

  #------------------------------------------
  # Go to a joint_space target joint angles
  joint_goal = arm_group.get_current_joint_values()
  print(joint_goal)
  joint_goal[0] = pi/4
  joint_goal[1] = -pi/4
  joint_goal[2] = 0
  joint_goal[3] = pi/4
  plan = arm_group.go(joint_goal, wait=True)

  #------------------------------------------
  # Go to a task_space point
  pose_goal = geometry_msgs.msg.Pose()    #pose_goal = arm_group.get_current_pose().pose
  pose_goal.orientation.w = 1.0
  pose_goal.position.x = 0.15
  pose_goal.position.y = 0.00
  pose_goal.position.z = 0.25
  print("Goal pose: ")
  print(pose_goal.position)
  #arm_group.set_goal_orientation_tolerance(0.01)

  arm_group.set_pose_target(pose_goal)
  plan = arm_group.go(wait=True)
  #arm_group.stop()
  #arm_group.clear_pose_targets()

  print("Current pose: ")
  print(arm_group.get_current_pose().pose)


  #------------------------------------------
  # Make a complex motion: Circle
  r = .050
  path_points = circle(r, 10, arm_group.get_current_pose().pose)
  #print(path_points)


  (plan, fraction) = arm_group.compute_cartesian_path(
                                   path_points,   # waypoints to follow
                                   0.001,        # eef_step, 0.001
                                   0.0)         # jump_threshold, 0.0

  #plan = arm_group.retime_trajectory(robot.get_current_state(), plan, .05)  # (0.02) time scaling factor


  display_trajectory = moveit_msgs.msg.DisplayTrajectory()
  display_trajectory.trajectory_start = robot.get_current_state()
  display_trajectory.trajectory.append(plan)
  # Publish
  display_trajectory_publisher.publish(display_trajectory);

  # Excute trajectory
  arm_group.execute(plan, wait=True)


  # Clear
  arm_group.stop()
  #arm_group.clear_pose_targets()

  #moveit_commander.roscpp_shutdown()