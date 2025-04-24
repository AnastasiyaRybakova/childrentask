#!/usr/bin/env python
import rospy
from open_manipulator_msgs.srv import (
    SetJointPosition,
    SetJointPositionRequest,
    SetKinematicsPose,
    SetKinematicsPoseRequest
)
import math

# Parameters
PEN_HEIGHT = 0.070     # Height of the pen (meters) from the bottom to the center of the pen holder
SURFACE_Z = 0.432      # Z-coordinate of the horizontal surface (meters)
DRAWING_DEPTH = 0.01   # Depth for proper contact (meters)
Z_OFFSET = 0.015       # Adjust Z-coordinate for pen tip contact

try:
    goal_joint_space_path_from_present_client_ = rospy.ServiceProxy(
        "/goal_joint_space_path_from_present", 
        SetJointPosition
    )
except rospy.ServiceException as e:
    rospy.logerr("Service call failed: {}".format(e))


def move_to_positions(joint_positions, duration=2.0):
    rospy.wait_for_service('/goal_joint_space_path')
    try:
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


def goal_task_client(joint_positions, duration=2.0):
    
    rospy.wait_for_service('goal_task_space_path')  
    try:
        goal_task_space_path_from_present_position_only_client_ = rospy.ServiceProxy(
            "goal_task_space_path",
            SetKinematicsPose
        )

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
        z = 0.095 # Default Z-position for the pen raised
    return [x, y, z, 0.0]

def lift_pen(center):
    x, y = center
    z_lift = 0.110
    joint_positions = calculate_pen_tip_position(x, y, z_lift)
    goal_task_client(joint_positions, duration=1.0)

def smooth_start(points, z):
    start_x, start_y = points[0]
    goal_task_client([start_x, start_y, z + 0.01], duration=0.5)
    goal_task_client([start_x, start_y, z], duration=0.5)


def draw_square(center=(0.15, 0.0), size=0.03):
    rospy.loginfo("Drawing Square...")
    z = 0.082
    x, y = center
    corners = [
        (x, y),
        (x + size, y),
        (x + size, y + size),
        (x, y + size),
        (x, y)
    ]
    smooth_start(corners, z)
    for px, py in corners[1:]:
        goal_task_client([px, py, z], duration=0.5)

def draw_circle(center=(0.21, 0.0), radius=0.015, steps=50):
    rospy.loginfo("Drawing Circle...")
    z = 0.088
    points = []
    for i in range(steps + 1):
        angle = 2 * math.pi * i / steps
        x = center[0] + radius * math.cos(angle)
        y = center[1] + radius * math.sin(angle)
        points.append((x, y))
    smooth_start(points, z)
    for x, y in points[1:]:
        goal_task_client([x, y, z], duration=0.1)

def draw_triangle(center=(0.25, 0.05), side_length=0.03):
    rospy.loginfo("Drawing Triangle...")
    z = 0.089
    half_s = side_length / 2.0
    half_height = (math.sqrt(3) / 2.0) * side_length / 2.0
    corners = [
        (center[0], center[1] + half_height),
        (center[0] - half_s, center[1] - half_height),
        (center[0] + half_s, center[1] - half_height),
        (center[0], center[1] + half_height)
    ]
    smooth_start(corners, z)
    for x, y in corners[1:]:
        goal_task_client([x, y, z], duration=0.5)

def draw_star(center=(0.18, 0.08), radius=0.02):
    rospy.loginfo("Drawing Star...")
    z = 0.085
    vertices = []
    for i in range(5):
        angle = 2.0 * math.pi * i / 5.0
        x = center[0] + radius * math.cos(angle)
        y = center[1] + radius * math.sin(angle)
        vertices.append((x, y))
    star_order = [0, 2, 4, 1, 3, 0]
    path = [vertices[i] for i in star_order]
    smooth_start(path, z)
    for x, y in path[1:]:
        goal_task_client([x, y, z], duration=0.4)

def draw_heart(center=(0.18, -0.10), scale=0.0015, steps=120):
    rospy.loginfo("Drawing Heart...")
    z = 0.085
    points = []
    for i in range(steps + 1):
        t = 2.0 * math.pi * i / steps
        x = 16 * (math.sin(t) ** 3) * scale + center[0]
        y = (13 * math.cos(t) - 5 * math.cos(2 * t) - 2 * math.cos(3 * t) - math.cos(4 * t)) * scale + center[1]
        points.append((x, y))
    smooth_start(points, z)
    for x, y in points[1:]:
        goal_task_client([x, y, z], duration=0.1)

def draw_pentagon(center=(0.24, -0.08), side_length=0.04):
    rospy.loginfo("Drawing Pentagon...")
    z = 0.079
    angle = 2 * math.pi / 5
    radius = side_length / (2 * math.sin(math.pi / 5))
    vertices = [(center[0] + radius * math.cos(i * angle), center[1] + radius * math.sin(i * angle)) for i in range(5)]
    vertices.append(vertices[0])
    smooth_start(vertices, z)
    for x, y in vertices[1:]:
        goal_task_client([x, y, z], duration=0.5)

def draw_sun(center=(0.16, 0.20), radius=0.015):
    rospy.loginfo("Drawing Sun...")
    z = 0.087
    steps = 20
    circle_points = []
    for i in range(steps + 1):
        angle = 2.0 * math.pi * i / steps
        x = center[0] + radius * math.cos(angle)
        y = center[1] + radius * math.sin(angle)
        circle_points.append((x, y))
    smooth_start(circle_points, z)
    for x, y in circle_points[1:]:
        goal_task_client([x, y, z], duration=0.1)
    for i in range(8):
        angle = 2 * math.pi * i / 8
        goal_task_client([center[0] + radius * math.cos(angle), center[1] + radius * math.sin(angle), z], duration=0.2)
        goal_task_client([center[0] + (radius + 0.01) * math.cos(angle), center[1] + (radius + 0.01) * math.sin(angle), z], duration=0.2)

def draw_house(center=(0.15, -0.06), width=0.025, height=0.025):
    rospy.loginfo("Drawing House...")
    z = 0.082
    w = width / 2
    h = height / 2
    base = [
        (center[0] - w, center[1] - h),
        (center[0] + w, center[1] - h),
        (center[0] + w, center[1] + h),
        (center[0] - w, center[1] + h),
        (center[0] - w, center[1] - h)
    ]
    smooth_start(base, z)
    for x, y in base[1:]:
        goal_task_client([x, y, z], duration=0.5)
    roof = [
        (center[0] - w, center[1] + h),
        (center[0], center[1] + h + 0.025),
        (center[0] + w, center[1] + h)
    ]
    smooth_start(roof, z)
    for x, y in roof[1:]:
        goal_task_client([x, y, z], duration=0.5)

def draw_cloud(center=(0.18, 0.15), radius=0.008, steps_per_bump=15):
    rospy.loginfo("Drawing Cloud...")
    z = 0.084
    bumps = 3
    spacing = radius * 2.0

    points = []

    # First bump
    cx = center[0] - spacing
    for j in range(steps_per_bump + 1):
        t = float(j) / steps_per_bump
        angle = math.pi * (1 - t)
        x = cx + radius * math.cos(angle)
        y = center[1] + radius * math.sin(angle)
        points.append((x, y))

    # Middle larger bump
    cx = center[0]
    large_radius = radius * 1.5
    for j in range(steps_per_bump + 1):
        t = float(j) / steps_per_bump
        angle = math.pi * (1 - t)
        x = cx + large_radius * math.cos(angle)
        y = center[1] + large_radius * math.sin(angle)
        points.append((x, y))

    # Third bump
    cx = center[0] + spacing
    for j in range(steps_per_bump + 1):
        t = float(j) / steps_per_bump
        angle = math.pi * (1 - t)
        x = cx + radius * math.cos(angle)
        y = center[1] + radius * math.sin(angle)
        points.append((x, y))

    # Draw bumps
    for idx, (x, y) in enumerate(points):
        goal_task_client([x, y, z], duration=0.1 if idx > 0 else 1.0)

    # Draw simple curved line back to start
    end_x, end_y = points[-1]
    start_x, start_y = points[0]
    link_steps = 10
    for i in range(1, link_steps + 1):
        t = float(i) / link_steps
        x = end_x + t * (start_x - end_x)
        y = end_y - radius * 0.3 * math.sin(math.pi * t)
        goal_task_client([x, y, z], duration=0.1)

    lift_pen(center)


def draw_fish(center=(0.24, 0.13), body_length=0.035, body_width=0.02):
    rospy.loginfo("Drawing Fish...")
    z = 0.085
    steps = 40

    # Correct elliptical body points
    body_points = []
    for i in range(steps + 1):
        angle = 2 * math.pi * i / steps
        x = center[0] + (body_length / 2) * math.cos(angle)
        y = center[1] + (body_width / 2) * math.sin(angle)
        body_points.append((x, y))

    smooth_start(body_points, z)

    for x, y in body_points[1:]:
        goal_task_client([x, y, z], duration=0.1)

    lift_pen(center)

    # Tail drawing
    tail = [
        (center[0] - body_length / 2, center[1]),
        (center[0] - body_length * 0.7, center[1] + body_width * 0.8),
        (center[0] - body_length * 0.7, center[1] - body_width * 0.8),
        (center[0] - body_length / 2, center[1])
    ]

    smooth_start(tail, z)

    for x, y in tail[1:]:
        goal_task_client([x, y, z], duration=0.2)

    lift_pen(center)

    

def draw_house(center=(0.15, -0.06), width=0.025, height=0.025):
    rospy.loginfo("Drawing House...")
    z = 0.082
    w = width / 2
    h = height / 2
    base = [
        (center[0] - w, center[1] - h),
        (center[0] + w, center[1] - h),
        (center[0] + w, center[1] + h),
        (center[0] - w, center[1] + h),
        (center[0] - w, center[1] - h)
    ]
    for i, (x, y) in enumerate(base):
        goal_task_client([x, y, z], duration=0.5 if i > 0 else 1.0)
    roof = [
        (center[0] - w, center[1] + h),
        (center[0], center[1] + h + 0.025),
        (center[0] + w, center[1] + h)
    ]
    for x, y in roof:
        goal_task_client([x, y, z], duration=0.5)


def draw_flower(center=(0.20, 0.10), petal_radius=0.01, num_petals=6, center_radius=0.006):
    rospy.loginfo("Drawing Cartoon Flower...")
    z = 0.084
    steps = 20

    # Draw flower center (full circle)
    center_points = []
    for i in range(steps + 1):
        angle = 2 * math.pi * i / steps
        x = center[0] + center_radius * math.cos(angle)
        y = center[1] + center_radius * math.sin(angle)
        center_points.append((x, y))

    smooth_start(center_points, z)
    for x, y in center_points[1:]:
        goal_task_client([x, y, z], duration=0.08)

    lift_pen(center)

    # Draw each petal as an unfinished circle (270 degrees)
    for p in range(num_petals):
        angle_offset = 2 * math.pi * p / num_petals
        petal_center_x = center[0] + (center_radius + petal_radius) * math.cos(angle_offset)
        petal_center_y = center[1] + (center_radius + petal_radius) * math.sin(angle_offset)

        petal_points = []
        start_angle = math.pi / 4
        end_angle = start_angle + 1.5 * math.pi
        for i in range(steps + 1):
            angle = start_angle + (end_angle - start_angle) * i / steps
            x = petal_center_x + petal_radius * math.cos(angle)
            y = petal_center_y + petal_radius * math.sin(angle)
            petal_points.append((x, y))

        smooth_start(petal_points, z)
        for x, y in petal_points[1:]:
            goal_task_client([x, y, z], duration=0.06)

        lift_pen(center)

    # Lift pen before drawing stem
    stem_length = 0.03
    stem_steps = 10
    stem_path = []
    for i in range(stem_steps + 1):
        x = center[0]
        y = center[1] - center_radius - i * (stem_length / stem_steps)
        stem_path.append((x, y))

    smooth_start(stem_path, z)
    for x, y in stem_path[1:]:
        goal_task_client([x, y, z], duration=0.1)

    lift_pen(center)


if __name__ == "__main__":
    square_corners = [
        (0.0, -0.015),
        (0.0,  0.015),
        (0.03, 0.015),
        (0.03, -0.015),
        (0.0, -0.015)
    ]

    rospy.init_node('openmanipulator_draw')

    home_positions = [0.2, 0.0, 0.2]  
    goal_task_client(home_positions, duration=2.0)

    rospy.loginfo("Drawing a square...")
    draw_square(center=(0.15, 0.0), size=0.03)
    goal_task_client(home_positions, duration=2.0)

    rospy.loginfo("Drawing a circle...")
    draw_circle(center=(0.21, 0.0), radius=0.015)
    goal_task_client(home_positions, duration=2.0)

    rospy.loginfo("Drawing a triangle...")
    draw_triangle(center=(0.25, 0.05), side_length=0.03)
    goal_task_client(home_positions, duration=2.0)

    rospy.loginfo("Drawing a star...")
    draw_star(center=(0.18, 0.08), radius=0.02)
    goal_task_client(home_positions, duration=2.0)

    rospy.loginfo("Drawing a heart shape...")
    draw_heart(center=(0.18, -0.10), scale=0.0015, steps=120)
    goal_task_client(home_positions, duration=2.0)

    rospy.loginfo("Drawing a pentagon...")
    draw_pentagon(center=(0.24, -0.08), side_length=0.04)
    goal_task_client(home_positions, duration=2.0)

    rospy.loginfo("Drawing Sun...")
    draw_sun(center=(0.16, 0.20), radius=0.015)
    goal_task_client(home_positions, duration=2.0)

    rospy.loginfo("Drawing Cloud...")
    draw_cloud(center=(0.18, 0.15), radius=0.008)
    goal_task_client(home_positions, duration=2.0)

    rospy.loginfo("Drawing Fish...")
    draw_fish(center=(0.24, 0.13), body_length=0.035, body_width=0.02)
    goal_task_client(home_positions, duration=2.0)

    rospy.loginfo("Drawing House...")
    raw_house(center=(0.15, -0.05), width=0.025, height=0.025)
    goal_task_client(home_positions, duration=2.0)
    
    #rospy.loginfo("Drawing flower...")
    #draw_flower(center=(0.27, 0.10), petal_radius=0.01)  
    #goal_task_client(home_positions, duration=2.0)


  
    # returning to home position after finishing the performance
    rospy.loginfo("Returning to home position...")
    goal_task_client(home_positions, duration=2.0)

    rospy.spin()
