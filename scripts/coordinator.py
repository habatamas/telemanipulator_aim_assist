#!/usr/bin/env python
import rospy
from open_manipulator_msgs.srv import SetKinematicsPose, SetKinematicsPoseRequest

class Coordinator:
    def __init__(self):
        # initialize ros node
        rospy.init_node('telemanipulator_coordinator')
        print("node initialized")

        # initialize calls to robot pose request service
        rospy.wait_for_service("/open_manipulator/goal_task_space_path_position_only")
        self.goal_task_space_service = rospy.ServiceProxy("/open_manipulator/goal_task_space_path_position_only", SetKinematicsPose)

    def request_pose(self, x, y, z, time=2.0):
        # create request and set data
        request = SetKinematicsPoseRequest()
        request.end_effector_name = "gripper"
        request.path_time = time
        request.kinematics_pose.pose.position.x = x
        request.kinematics_pose.pose.position.y = y
        request.kinematics_pose.pose.position.z = z

        # call service
        response = self.goal_task_space_service(request)
        print(response)

    def request_home_pose(self):
        self.request_pose(0.134,0,0.241)

    def request_init_pose(self):
        self.request_pose(0.286,0,0.204)

    def run(self):
        while True:
            print("home")
            self.request_home_pose()
            rospy.sleep(3)
            print("init")
            self.request_init_pose()
            rospy.sleep(3)

if __name__=="__main__":
    coordinator_node = Coordinator()
    coordinator_node.run()