#!/usr/bin/env python
import rospy
from open_manipulator_msgs.srv import SetJointPosition, SetJointPositionRequest, SetKinematicsPose, SetKinematicsPoseRequest
import math

# Parameters
PEN_HEIGHT = 0.065     # Height of the pen (meters) from the bottom to the center of the pen holder
SURFACE_Z = 0.432      # Z-coordinate of the horizontal surface (meters)
DRAWING_DEPTH = 0.01  # Depth for proper contact (meters)
Z_OFFSET = 0.015      # Adjust Z-coordinate for pen tip contact


try:
    
    goal_joint_space_path_from_present_client_                  = rospy.ServiceProxy("/goal_joint_space_path_from_present", SetJointPosition)
    # goal_task_space_path_from_present_position_only_client_     = rospy.ServiceProxy("/goal_task_space_path_from_present_position_only", SetKinematicsPose) 

except rospy.ServiceException as e:
    rospy.logerr("Service call failed: {}".format(e))


def move_to_positions(joint_positions, duration=2.0):
    """Send joint positions to the OpenManipulator using the SetJointPosition service."""
    rospy.wait_for_service('/goal_joint_space_path')
    try :
        set_joint_position = rospy.ServiceProxy('/goal_joint_space_path', SetJointPosition)    
        request = SetJointPositionRequest()
        request.planning_group = "arm"
        request.joint_position.joint_name = ["joint1", "joint2", "joint3", "joint4"]
        request.joint_position.position = joint_positions
        request.path_time = duration
        set_joint_position(request)
        rospy.sleep(duration)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))

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
    # rospy.wait_for_service('goal_task_space_path_from_present_position_only')
    rospy.wait_for_service('goal_task_space_path')
    
    try :
        # goal_task_space_path_from_present_position_only_client_ = rospy.ServiceProxy("goal_task_space_path_from_present_position_only", SetKinematicsPose) 
        goal_task_space_path_from_present_position_only_client_ = rospy.ServiceProxy("goal_task_space_path", SetKinematicsPose) 
        #print(goal_task_space_path_from_present_position_only_client_.request_class)
        
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

        goal_task_space_path_from_present_position_only_client_(request)
        rospy.sleep(duration)

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))
    return


def calculate_pen_tip_position(x, y, z=None):

    if z is None:
        #z = SURFACE_Z + PEN_HEIGHT - DRAWING_DEPTH - Z_OFFSET  # Default Z-position for the pen to touch the surface
        z = 0.06 
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

    for corner in square_corners:
        x, y = corner
        joint_positions = calculate_pen_tip_position(x, y)
        move_to_positions(joint_positions, duration=2.0)

        
def draw_circle(center=(0.15, 0.1), radius=0.015, steps=50):
    angle = 2 * math.pi
    x = radius * math.cos(angle)
    y = radius * math.sin(angle)
    z=0.06
    joint_positions = calculate_pen_tip_position(x + center[0], y + center[1], z)
    goal_task_client(joint_positions, duration=2)

    for i in range(steps + 1)[1:]:
        angle = 2 * math.pi * i / steps
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)

        joint_positions = calculate_pen_tip_position(x + center[0], y + center[1], z)
        goal_task_client(joint_positions, duration=0.05)


if __name__ == "__main__":
    square_corners = [
        (0.0, -0.015),  # Bottom left
        (0.0, 0.015),   # Top left
        (0.03, 0.015),   # Top right
        (0.03, -0.015),  # Bottom right
        (0.0, -0.015)   # Back to bottom left
    ]
    rospy.init_node('openmanipulator_draw_shapes')
    # rospy.sleep(30)
    home_positions = [0.2, 0.0, 0.2]  # Example positions; adjust based on IK solution
    goal_task_client(home_positions, duration=2.0)
    
    
    for x, y in square_corners :
          #each_time_move = before - [x, y, 0.15]
          goal_positions = [x+0.15, y, 0.06]  # Example positions; adjust based on IK solution
          goal_task_client(goal_positions, duration=5.0)
    
    # # Draw circle
    home_positions = [0.2, 0.0, 0.2]  # Example positions; adjust based on IK solution
    goal_task_client(home_positions, duration=2.0)
    draw_circle(center=(0.15, 0.1), steps=50)  # Adjust radius as needed
    #y = -0.1
    #for i in range(-10, 11) :
    #    x = 0.01*i
    #    print(i, x)
    #    goal_positions = [x+0.3, y, 0.1]  # Example positions; adjust based on IK solution
    #    # print(goal_positions)
    #    goal_task_client(goal_positions, duration=2.0)
    #    for j in range(-10, 11) :
    #        y = 0.01*j
    #       goal_positions = [x+0.3, y, 0.07]  # Example positions; adjust based on IK solution
    #       goal_task_client(goal_positions, duration=1.0)
    #        goal_positions = [x+0.3, y, 0.067]  # Example positions; adjust based on IK solution
    #       goal_task_client(goal_positions, duration=0.4)
    #       goal_positions = [x+0.3, y, 0.07]  # Example positions; adjust based on IK solution
    #       goal_task_client(goal_positions, duration=0.4)
    

        
    # home_positions = [0.2, 0.0, 0.2]  # Example positions; adjust based on IK solution
    # goal_task_client(home_positions, duration=2.0)
        
    # goal_positions = [0.15, 0.0, 0.045]  # Example positions; adjust based on IK solution
    # goal_task_client(goal_positions, duration=2.0)

    # # Draw shapes
    # rospy.loginfo("Drawing a square...")
    # draw_square()

    # rospy.loginfo("Drawing a circle...")
    # draw_circle()
    goal_task_client(home_positions, duration=2.0)
    rospy.spin()
