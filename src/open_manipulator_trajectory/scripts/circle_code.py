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


def circle(radius,res,start):
   waypoints = []  #planning a Cartesian path directly by a list of waypoints
   waypoints.append(start)
   for i in range(0,res):
      point = copy.deepcopy(start)
      point.position.x += radius*math.cos((2*math.pi/res)*i)
      point.position.z += radius*math.sin((2*math.pi/res)*i)
      #point.position.y =.2
      waypoints.append(copy.deepcopy(point))
   #waypoints.append(start)
   return waypoints     

if __name__ == "__main__":
	#initialize moveit_commander and node
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('move_group_circle', anonymous=True)

	#getting instance from moveit_commander
	robot = moveit_commander.RobotCommander()  #robot commander to interface to the robot
	scene = moveit_commander.PlanningSceneInterface()
	arm_group = moveit_commander.MoveGroupCommander('arm')  #moveing group commander

	#creating a display trajectory publisher which is used later to publish trajectories for Rviz
	display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

	#pose_goal = geometry_msgs.msg.Pose()   
	#pose_goal.orientation.w = 0.90
	#pose_goal.position.x = 0.25
	#pose_goal.position.y = 0.00
	#pose_goal.position.z = 0.15
	#print("Goal pose: ")
	#print(pose_goal.position)

  	#arm_group.set_goal_orientation_tolerance(0.01)
	
	#arm_group.set_pose_target(pose_goal)
	#plan = arm_group.go(wait=True)
	print("current pose: ")
	print(arm_group.get_current_pose().pose)

	# makeing circle motion
	r = .075
	path_points = circle(r, 100, arm_group.get_current_pose().pose)
	
	#for i in path_points:
		##print("x: " + str(i.position.x) + " y: " + str(i.position.y) + " z: " + str(i.position.z))

	(plan, fraction) = arm_group.compute_cartesian_path(
                                   path_points,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold

	plan = arm_group.retime_trajectory(robot.get_current_state(), plan, .05)  # (0.02) time scaling factor

	display_trajectory = moveit_msgs.msg.DisplayTrajectory()
	display_trajectory.trajectory_start = robot.get_current_state()
	display_trajectory.trajectory.append(plan)

	# Publish
	display_trajectory_publisher.publish(display_trajectory);
	print("Waiting while")

	# Excutes trajectory
	arm_group.execute(plan, wait=True)
	print("trajectory finished")
	# Clear
	arm_group.stop()


	#moveit_commander.roscpp_shutdown()
	