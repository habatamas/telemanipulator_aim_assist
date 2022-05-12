#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String, UInt8, UInt8MultiArray, Float32, Float32MultiArray
from open_manipulator_msgs.srv import SetJointPositionRequest, SetJointPosition
import math

# forward kinematics calculation
# source: https://github.com/MOGI-ROS/open_manipulator_tools/blob/main/scripts/forward_kinematics.py
def forward_kinematics(joint_angles):
    '''
    Calculates the TCP coordinates from the joint angles
    :param joint_angles: list, joint angles [j0, j1, j2, j3]
    :return: list, the list of TCP coordinates
    '''
    # link lengths
    l1 = 0.128
    l2 = 0.024
    l3 = 0.124
    l4 = 0.126

    # offsets
    x_offset = 0.012
    z_offset = 0.0595 + 0.017

    x = x_offset + (l1 * math.sin(joint_angles[1]) + l2 * math.cos(joint_angles[1]) + l3 * math.cos(joint_angles[1] + joint_angles[2]) + l4 * math.cos(joint_angles[1] + joint_angles[2] + joint_angles[3])) * math.cos(joint_angles[0])
    y = (l1 * math.sin(joint_angles[1]) + l2 * math.cos(joint_angles[1]) + l3 * math.cos(joint_angles[1] + joint_angles[2]) + l4 * math.cos(joint_angles[1] + joint_angles[2] + joint_angles[3])) * math.sin(joint_angles[0])
    z = z_offset + l1 * math.cos(joint_angles[1]) - l2 * math.sin(joint_angles[1]) - l3 * math.sin(joint_angles[1] + joint_angles[2]) - l4 * math.sin(joint_angles[1] + joint_angles[2] + joint_angles[3])

    return [x,y,z]

class ControllerNode:
    def __init__(self):
        # initialize ros node
        rospy.init_node('controller_haptic_device')

        # initialize subscribers and publishers
        self.desired_position_publisher = rospy.Publisher('/telemanipulator_aim_assist/desired_position', Point, queue_size=1)
        self.angle_subscriber  = rospy.Subscriber('/mogi_haptic/joint_angles', Float32MultiArray, self.joint_angle_callback, queue_size=10)
        self.button_subscriber = rospy.Subscriber('/mogi_haptic/button_states', UInt8MultiArray, self.button_callback, queue_size=10)

        # previous button state
        self.prev_button_state = "\x00\x00\x00\x00\x00\x00"

    # joint angle subscriber callback
    def joint_angle_callback(self, joint_angles):
        x,y,z = forward_kinematics(joint_angles.data)
        desired_position = Point(x,y,z)
        self.desired_position_publisher.publish(desired_position)

    # button press callback
    def button_callback(self, button_data):
        # check for keypress
        if(button_data.data[0]=='\x01' and self.prev_button_state[0]=='\x00'):
            self.set_gripper_client(-0.01,1)

        # check for keypress
        if(button_data.data[0]=='\x00' and self.prev_button_state[0]=='\x01'):
            self.set_gripper_client(0.01,0)

        self.prev_button_state = button_data.data

    def set_gripper_client(self, gripper, time):
        service_name = '/open_manipulator/goal_tool_control'

        rospy.wait_for_service(service_name,1)

        try:
            set_joint_angles = rospy.ServiceProxy(service_name, SetJointPosition)

            arg = SetJointPositionRequest()
            arg.joint_position.joint_name = ["gripper"]
            arg.joint_position.position = [gripper]
            arg.path_time = time
            resp1 = set_joint_angles(arg)
            print('Service done!')
            return resp1
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return False

    # run main loop
    def run(self):
        rospy.spin()


if __name__=="__main__":
    node = ControllerNode()
    node.run()
