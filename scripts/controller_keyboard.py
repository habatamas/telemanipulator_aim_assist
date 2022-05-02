#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point

# initialize ros node
rospy.init_node('controller_keyboard')
publisher = rospy.Publisher('desired_position', Point, queue_size=10)

print("-= Telemanipulator Aim Assist =-")

while not rospy.is_shutdown(): # run the node until Ctrl-C is pressed

    # read desired coordinates from terminal
    print("")
    print("Please enter the desired TCP coordinates:")
    x = float(input("    x: "))
    y = float(input("    y: "))
    z = float(input("    z: "))

    # construct message
    desired_position = Point()
    desired_position.x = x
    desired_position.y = y
    desired_position.z = z

    # publish message
    publisher.publish(desired_position)
