#!/usr/bin/env python
import rospy
from open_manipulator_msgs.srv import SetKinematicsPose, SetKinematicsPoseRequest

# initialize ros node
rospy.init_node('telemanipulator_coordinator')

# connect to openmanipulator controller service
rospy.wait_for_service("/open_manipulator/goal_task_space_path_position_only")

service = rospy.ServiceProxy("/open_manipulator/goal_task_space_path_position_only", SetKinematicsPose)


while True:
    # go to home position
    request = SetKinematicsPoseRequest()
    request.end_effector_name = "gripper"
    request.path_time = 2.0
    request.kinematics_pose.pose.position.x = 0.134
    request.kinematics_pose.pose.position.y = 0
    request.kinematics_pose.pose.position.z = 0.241
    response = service(request)
    rospy.sleep(3)

    # go to init position
    request = SetKinematicsPoseRequest()
    request.end_effector_name = "gripper"
    request.path_time = 2.0
    request.kinematics_pose.pose.position.x = 0.286
    request.kinematics_pose.pose.position.y = 0
    request.kinematics_pose.pose.position.z = 0.204
    response = service(request)
    rospy.sleep(3)