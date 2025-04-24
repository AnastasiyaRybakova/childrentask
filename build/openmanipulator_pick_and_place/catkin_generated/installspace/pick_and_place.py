import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from ar_track_alvar_msgs.msg import AlvarMarkers
from open_manipulator_msgs.srv import SetJointPosition, SetKinematicsPose
from sensor_msgs.msg import JointState
import sys

class OpenManipulatorPickAndPlace:
    def __init__(self):
        # Initialize the node
        rospy.init_node("openmanipulator_pick_and_place")

        # MoveIt! initialization
        moveit_commander.roscpp_initialize(sys.argv)
        self.move_group = moveit_commander.MoveGroupCommander("openmanipulator_x")

        # Initialize variables
        self.mode_state_ = 0
        self.demo_count_ = 0
        self.pick_ar_id_ = 0
        self.present_joint_angle_ = [0.0] * 5  # 4 joints + gripper
        self.present_kinematic_position_ = [0.0] * 3

        self.ar_marker_pose = []
        self.open_manipulator_is_moving_ = False

        # Setup service clients for joint control and tool control
        self.goal_joint_space_path_client = rospy.ServiceProxy('goal_joint_space_path', SetJointPosition)
        self.goal_tool_control_client = rospy.ServiceProxy('goal_tool_control', SetJointPosition)
        self.goal_task_space_path_client = rospy.ServiceProxy('goal_task_space_path', SetKinematicsPose)

        # Subscribe to necessary topics
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.arPoseMarkerCallback)
        rospy.Subscriber("joint_states", JointState, self.jointStatesCallback)
        rospy.Subscriber("gripper/kinematics_pose", Pose, self.kinematicsPoseCallback)

        # Timer for periodic callback
        rospy.Timer(rospy.Duration(0.1), self.publishCallback)

    def setJointSpacePath(self, joint_angle, path_time):
        try:
            response = self.goal_joint_space_path_client(joint_position=joint_angle, path_time=path_time)
            return response.is_planned
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

    def setToolControl(self, joint_angle):
        try:
            response = self.goal_tool_control_client(joint_position=joint_angle)
            return response.is_planned
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

    def setTaskSpacePath(self, kinematics_pose, kinematics_orientation, path_time):
        try:
            response = self.goal_task_space_path_client(
                kinematics_pose=kinematics_pose,
                kinematics_orientation=kinematics_orientation,
                path_time=path_time
            )
            return response.is_planned
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

    def arPoseMarkerCallback(self, msg):
        """Callback for AR marker detection"""
        self.ar_marker_pose = []
        for marker in msg.markers:
            marker_data = {
                "id": marker.id,
                "position": [marker.pose.pose.position.x, marker.pose.pose.position.y, marker.pose.pose.position.z]
            }
            self.ar_marker_pose.append(marker_data)

    def jointStatesCallback(self, msg):
        """Callback for joint states"""
        self.present_joint_angle_ = [msg.position[i] for i in range(len(msg.name))]

    def kinematicsPoseCallback(self, msg):
        """Callback for kinematics pose"""
        self.present_kinematic_position_ = [msg.position.x, msg.position.y, msg.position.z]

    def printText(self):
        """Print status to the console"""
        print("\n-----------------------------")
        print("Pick and Place demonstration!")
        print("-----------------------------")
        print("1 : Home pose")
        print("2 : Pick and Place demo. start")
        print("3 : Pick and Place demo. Stop")
        print("-----------------------------")

        if self.mode_state_ == 1:
            print(f"Current state: {self.demo_count_}")

        print(f"Joint Angles: {self.present_joint_angle_}")
        print(f"Kinematic Position: {self.present_kinematic_position_}")

        if self.ar_marker_pose:
            print("AR marker detected.")
            for marker in self.ar_marker_pose:
                print(f"ID: {marker['id']} --> Position: {marker['position']}")

    def demoSequence(self):
        """Execute the pick-and-place demo sequence"""
        joint_angle = []
        kinematics_position = []
        kinematics_orientation = []
        gripper_value = []

        if self.demo_count_ == 0:  # Home pose
            joint_angle = [0.00, -1.05, 0.35, 0.70]
            self.setJointSpacePath(joint_angle, 1.5)
            self.demo_count_ += 1
        elif self.demo_count_ == 1:  # Initial pose
            joint_angle = [0.01, -0.80, 0.01, 1.90]
            self.setJointSpacePath(joint_angle, 1.0)
            self.demo_count_ += 1
        elif self.demo_count_ == 2:  # Wait & open the gripper
            self.setJointSpacePath(self.present_joint_angle_, 3.0)
            gripper_value = [0.010]
            self.setToolControl(gripper_value)
            self.demo_count_ += 1
        elif self.demo_count_ == 3:  # Pick the box
            for marker in self.ar_marker_pose:
                if marker["id"] == self.pick_ar_id_:
                    kinematics_position = marker["position"]
                    kinematics_position[2] = 0.05  # Set Z position
                    kinematics_orientation = [0.74, 0.00, 0.66, 0.00]
                    self.setTaskSpacePath(kinematics_position, kinematics_orientation, 2.0)
                    self.demo_count_ += 1
                    return
            self.demo_count_ = 2  # Retry pick if no marker detected
        elif self.demo_count_ == 4:  # Wait & grip
            self.setJointSpacePath(self.present_joint_angle_, 1.0)
            gripper_value = [-0.002]
            self.setToolControl(gripper_value)
            self.demo_count_ += 1
        elif self.demo_count_ == 5:  # Initial pose again
            joint_angle = [0.01, -0.80, 0.00, 1.90]
            self.setJointSpacePath(joint_angle, 1.0)
            self.demo_count_ += 1
        elif self.demo_count_ == 6:  # Place pose
            joint_angle = [1.57, -0.21, -0.15, 1.89]
            self.setJointSpacePath(joint_angle, 1.0)
            self.demo_count_ += 1
        elif self.demo_count_ == 7:  # Place the box
            kinematics_position = self.present_kinematic_position_
            kinematics_position[2] -= 0.076  # Adjust based on pick id
            kinematics_orientation = [0.74, 0.00, 0.66, 0.00]
            self.setTaskSpacePath(kinematics_position, kinematics_orientation, 2.0)
            self.demo_count_ += 1
        elif self.demo_count_ == 8:  # Wait & place
            gripper_value = [0.010]
            self.setToolControl(gripper_value)
            self.demo_count_ += 1
        elif self.demo_count_ == 9:  # Move up after place
            kinematics_position = self.present_kinematic_position_
            kinematics_position[2] = 0.135
            kinematics_orientation = [0.74, 0.00, 0.66, 0.00]
            self.setTaskSpacePath(kinematics_position, kinematics_orientation, 2.0)
            self.demo_count_ += 1
        elif self.demo_count_ == 10:  # Home pose again
            joint_angle = [0.00, -1.05, 0.35, 0.70]
            self.setJointSpacePath(joint_angle, 1.5)
            self.demo_count_ = 1
            if self.pick_ar_id_ == 0:
                self.pick_ar_id_ = 1
            elif self.pick_ar_id_ == 1:
                self.pick_ar_id_ = 2
            else:
                self.pick_ar_id_ = 0
                self.demo_count_ = 0
                self.mode_state_ = 0

    def publishCallback(self, event):
        """Callback for timer event to periodically check mode and perform actions"""
        self.printText()
        if self.mode_state_ == 1:
            if not self.open_manipulator_is_moving_:
                self.demoSequence()

    def run(self):
        """Start the process and keep spinning ROS"""
        rospy.spin()

if __name__ == '__main__':
    try:
        manipulator = OpenManipulatorPickAndPlace()
        manipulator.run()
    except rospy.ROSInterruptException:
        pass
