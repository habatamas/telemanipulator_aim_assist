#!/usr/bin/env python

import rospy

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point

class Controller:
    def __init__(self):
        # initialize node
        rospy.init_node("controller_interactive_marker")

        # create desired position piblisher
        self.publisher = rospy.Publisher('telemanipulator_aim_assist/desired_position', Point, queue_size=10)
    
        # create interactive marker server
        self.server = InteractiveMarkerServer("telemanipulator_controller_marker")

        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "world"
        int_marker.name = "telemanipulator_controller_marker"
        int_marker.description = "Telemanipulator control"
        int_marker.scale = 0.1

        # create a box marker
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.scale.x = 0.015
        box_marker.scale.y = 0.015
        box_marker.scale.z = 0.015
        box_marker.color.r = 0.0
        box_marker.color.g = 0.5
        box_marker.color.b = 0.5
        box_marker.color.a = 1.0

        # create a non-interactive control which contains the box
        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.markers.append( box_marker )

        # add the control to the interactive marker
        int_marker.controls.append( box_control )

        # create x y z axis controllers
        control = InteractiveMarkerControl()
        control.name = "move_x"
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.name = "move_y"
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.name = "move_z"
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        # add the interactive to the server
        self.server.insert(int_marker, self.marker_callback)
    
    def marker_callback(self, feedback):
        p = feedback.pose.position

        print(feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z))

        desired_position = Point()
        desired_position.x = p.x
        desired_position.y = p.y
        desired_position.z = p.z
        self.publisher.publish(desired_position)

    def run(self):
        self.server.applyChanges()
        rospy.spin()

if __name__=="__main__":
    controller = Controller()
    controller.run()