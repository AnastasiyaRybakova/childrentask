#!/usr/bin/env python3
import rospy
from open_manipulator_msgs.srv import SetJointPosition, SetJointPositionRequest, SetKinematicsPose, SetKinematicsPoseRequest
import math

# Parameters
PEN_HEIGHT = 0.06     # Height of the pen (meters) from the bottom to the center of the pen holder
SURFACE_Z = 0.06      # Z-coordinate of the horizontal surface (meters)
DRAWING_DEPTH = 0.01  # Depth for proper contact (meters)
Z_OFFSET = 0.015      # Adjust Z-coordinate for pen tip contact


try:
    set_joint_position                                          = rospy.ServiceProxy('/goal_joint_space_path', SetJointPosition)
    goal_joint_space_path_from_present_client_                  = rospy.ServiceProxy("/goal_joint_space_path_from_present", SetJointPosition)
    goal_task_space_path_from_present_position_only_client_     = rospy.ServiceProxy("/goal_task_space_path_from_present_position_only", SetKinematicsPose) 

except rospy.ServiceException as e:
    rospy.logerr("Service call failed: {}".format(e))


def move_to_positions(joint_positions, duration=2.0):
    """Send joint positions to the OpenManipulator using the SetJointPosition service."""
    rospy.wait_for_service('/goal_joint_space_path')
        
    request = SetJointPositionRequest()
    request.planning_group = "arm"
    request.joint_position.joint_name = ["joint1", "joint2", "joint3", "joint4"]
    request.joint_position.position = joint_positions
    request.path_time = duration
    set_joint_position(request)
    rospy.sleep(duration)


# def kinemetics(joint_positions, duration=2.0):
#     """Send joint positions to the OpenManipulator using the SetJointPosition service."""
#     rospy.wait_for_service('/goal_joint_space_path_from_present')

    # request = SetJointPositionRequest()
    # request.planning_group = "arm"
    # request.joint_position.joint_name = ["joint1", "joint2", "joint3", "joint4"]
    # request.joint_position.position = joint_positions
    # request.path_time = duration
    # set_joint_position(request)
    # rospy.sleep(duration)

def goal_task_client(joint_positions, duration=2.0):
    rospy.wait_for_service('/goal_task_space_path_from_present_position_only')
        
    request = SetKinematicsPoseRequest()
    request.planning_group = "arm"
    request.end_effector_name = "gripper"

    request.kinematics_pose.pose.position.x = joint_positions[0]
    request.kinematics_pose.pose.position.y = joint_positions[1]
    request.kinematics_pose.pose.position.z = joint_positions[2]
    
    request.kinematics_pose.pose.orientation.x = 0.0
    request.kinematics_pose.pose.orientation.y = 0.0
    request.kinematics_pose.pose.orientation.z = 0.0
    request.kinematics_pose.pose.orientation.w = 1.0

    request.path_time = duration

    print(request)

    try : 
        goal_task_space_path_from_present_position_only_client_(request)
        rospy.sleep(duration)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))
    return


def calculate_pen_tip_position(x, y):
    """Calculate joint positions needed to position the pen tip at (x, y) on the surface."""
    # Adjust Z-coordinate to make the drawing surface lower by Z_OFFSET
    z = SURFACE_Z + PEN_HEIGHT - DRAWING_DEPTH - Z_OFFSET  # Correct Z-position for the pen to touch the surface

    # Placeholder joint positions (use proper IK service if available)
    joint_positions = [x, y, z, 0.0]  # Example positions; adjust based on IK solution
    return joint_positions

def move_to_start_position():
    """Move the robot to a starting position slightly above the paper."""
    start_position = calculate_pen_tip_position(0.0, 0.0)  # Set coordinates for start position
    move_to_positions(start_position, duration=2.0)

def draw_square():
    """Draw a square on the horizontal surface."""
    move_to_start_position()  # Ensure the robot starts above the paper

    square_corners = [
        (0.0, -0.1),  # Bottom left
        (0.0, 0.1),   # Top left
        (0.1, 0.1),   # Top right
        (0.1, -0.1),  # Bottom right
        (0.0, -0.1)   # Back to bottom left
    ]

    for corner in square_corners:
        x, y = corner
        joint_positions = calculate_pen_tip_position(x, y)
        move_to_positions(joint_positions, duration=2.0)

def draw_circle(radius=0.1, steps=36):
    """Draw a circle on the horizontal surface."""
    for i in range(steps):
        angle = 2 * math.pi * i / steps
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        joint_positions = calculate_pen_tip_position(x, y)
        move_to_positions(joint_positions, duration=0.1)

if __name__ == "__main__":
    rospy.init_node('openmanipulator_draw_shapes')
    # rospy.sleep(30)
    home_positions = [0.05, -0.000, 0.05]  # Example positions; adjust based on IK solution
    goal_task_client(home_positions, duration=2.0)


    # thetas = calculate_motor_angles(0.02, 0.01, 0.015, math.radians(45))
    # print(thetas)
    # move_to_positions(joint_positions, duration=2.0)
    # joint_positions = [0.0, 0, 0, 0.0]  # Example positions; adjust based on IK solution
    # move_to_positions(joint_positions, duration=2.0)
    

    # # Draw shapes
    # rospy.loginfo("Drawing a square...")
    # draw_square()

    # rospy.loginfo("Drawing a circle...")
    # draw_circle()
    rospy.spin()