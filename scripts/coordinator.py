#!/usr/bin/env python
import rospy
from open_manipulator_msgs.srv import SetKinematicsPose, SetKinematicsPoseRequest
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

class Coordinator:
    def __init__(self):
        # initialize ros node
        rospy.init_node('telemanipulator_coordinator')
        print("node initialized")

        # initialize calls to robot pose request service
        rospy.wait_for_service("/open_manipulator/goal_task_space_path_position_only")
        self.goal_task_space_service = rospy.ServiceProxy("/open_manipulator/goal_task_space_path_position_only", SetKinematicsPose)

        # subscribe to desired position topic
        rospy.Subscriber("telemanipulator_aim_assist/desired_position", Point, self.desired_position_callback)

        # create marker publisher
        self.marker_publisher = rospy.Publisher("/telemanipulator_aim_assist/markers", Marker, queue_size=10)

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

    def desired_position_callback(self, desired_position):
        self.request_pose(desired_position.x,desired_position.y,desired_position.z,0.5)

        # publish desired position marker
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time()
        marker.ns = "telemanipulator_aim_assist"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = desired_position.x
        marker.pose.position.y = desired_position.y
        marker.pose.position.z = desired_position.z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.marker_publisher.publish(marker)

    def run(self):
        rospy.spin()

if __name__=="__main__":
    coordinator_node = Coordinator()
    coordinator_node.run()