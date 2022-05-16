#!/usr/bin/env python
from pydoc import describe
from socket import timeout
import rospy
import threading
import Queue as queue
from open_manipulator_msgs.srv import SetKinematicsPose, SetKinematicsPoseRequest
from open_manipulator_msgs.msg import KinematicsPose, OpenManipulatorState
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from math import sqrt, atan2, sin, cos, pi

class Target:
    def __init__(self, x, y, z, h_low, h_high, r):
        self.x = x
        self.y = y
        self.z = z
        self.h_low = h_low
        self.h_high = h_high
        self.r = r
    
    def is_active(self, desired):
        r = ((desired.x-self.x)**2+(desired.y-self.y)**2)**0.5
        return self.z <= desired.z and desired.z <= self.h_high and r <= self.r

    def constrain(self, desired):
        # calculate xy offset
        dx = desired.x - self.x
        dy = desired.y - self.y
        r = sqrt(dx**2 + dy**2)
        phi = atan2(dy,dx)

        # check if desired position is in range
        if(r <= self.r):
            # conic mode
            if(self.h_low <= desired.z and desired.z <= self.h_high):
                ratio = (desired.z - self.h_low)/(self.h_high - self.h_low)
                r = min(r, ratio * self.r)

            # precise mode
            elif(self.z <= desired.z and desired.z <= self.h_low):
                r = 0

        # calculate admitted coordinates
        admitted = Point()
        admitted.x = self.x + r*cos(phi)
        admitted.y = self.y + r*sin(phi)
        admitted.z = desired.z

        return admitted
    
    def create_marker(self):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time()
        marker.ns = "telemanipulator_aim_assist"
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        
        # common properties
        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        marker.pose.position.z = self.h_low
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        marker.color.a = 0.3
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        # draw cone
        n = 100
        dphi = 2*pi / n
        for i in range(n):
            marker.points.append(Point(0,0,0))
            marker.points.append(Point(self.r*cos(i*dphi),self.r*sin(i*dphi),self.h_high-self.h_low))
            marker.points.append(Point(self.r*cos((i+1)*dphi),self.r*sin((i+1)*dphi),self.h_high-self.h_low))

        return marker

class Coordinator:
    targets = [Target(0.2, 0.1, 0, 0.1, 0.15, 0.05),
               Target(0.2, -0.1, 0, 0.1, 0.15, 0.05),
               Target(0.259, 0, 0, 0.1, 0.15, 0.05),
               Target(0.259, -0.1, 0, 0.1, 0.15, 0.05)]
    #targets = []

    def __init__(self):
        # initialize ros node
        rospy.init_node('telemanipulator_coordinator')
        print("node initialized")

        # create robot mover thread and queue
        self.speed = 1 # 1m/s
        self.move_queue = queue.Queue()
        move_thread = threading.Thread(target=self.move_thread)
        move_thread.daemon =True
        move_thread.start()

        # create event queue
        self.event_queue = queue.Queue()

        # subscribe to actual TCP position topic and wait for initial pose
        rospy.Subscriber("open_manipulator/gripper/kinematics_pose", KinematicsPose, self._tcp_pose_callback)
        self.actual_x = None
        self.actual_y = None
        self.actual_z = None

        # subscribe to desired position topic
        rospy.Subscriber("telemanipulator_aim_assist/desired_position", Point, self._desired_position_callback)

        # create marker publisher
        self.marker_publisher = rospy.Publisher("/telemanipulator_aim_assist/markers", Marker, queue_size=10)

    def move_thread(self):
        # task loop
        while True:
            # wait for target position
            x,y,z,time = self.move_queue.get()

            # initialize calls to robot pose request service
            rospy.wait_for_service("/open_manipulator/goal_task_space_path_position_only")
            goal_task_space_service = rospy.ServiceProxy("/open_manipulator/goal_task_space_path_position_only", SetKinematicsPose)

            # create request and set data
            request = SetKinematicsPoseRequest()
            request.end_effector_name = "gripper"
            request.path_time = time
            request.kinematics_pose.pose.position.x = x
            request.kinematics_pose.pose.position.y = y
            request.kinematics_pose.pose.position.z = z

            # call service
            print("New TCP position %g %g %g (time=%g) requested"%(x,y,z,time))
            response = goal_task_space_service(request)
            print(response)

            # wait for robot to stop
            state = OpenManipulatorState()
            state.open_manipulator_moving_state = state.IS_MOVING
            while(state.open_manipulator_moving_state!=state.STOPPED):
                state = rospy.wait_for_message("/open_manipulator/states", OpenManipulatorState) 
    
    # request a TCP position
    def request_pose(self, x, y, z, time=2.0):
        # clear previous elements
        if(not self.move_queue.empty()):
            with self.move_queue.mutex:
                self.move_queue.queue.clear()
        
        # queue new task
        self.move_queue.put((x,y,z,time))

    # request home TCP position
    def request_home_pose(self):
        self.request_pose(0.134,0,0.241)

    # request init TCP position
    def request_init_pose(self):
        self.request_pose(0.286,0,0.204)

    # update TCP  coordinates
    def _tcp_pose_callback(self, pose):
        self.event_queue.put((self.tcp_pose_callback, pose))
    def tcp_pose_callback(self, pose):
        self.actual_x = pose.pose.position.x
        self.actual_y = pose.pose.position.y
        self.actual_z = pose.pose.position.z

    # handle new desired postion request
    def _desired_position_callback(self, desired_position):
        self.event_queue.put((self.desired_position_callback, desired_position))
    def desired_position_callback(self, desired_position):
        # draw target markers
        for i in range(len(self.targets)):
            marker = self.targets[i].create_marker()
            marker.id = 100+i
            self.marker_publisher.publish(marker)

        # calculate admitted position
        admitted_position = desired_position
        for target in self.targets:
            if(target.is_active(desired_position)):
                admitted_position = target.constrain(desired_position)

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
        
        # publish admitted position marker
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time()
        marker.ns = "telemanipulator_aim_assist"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = admitted_position.x
        marker.pose.position.y = admitted_position.y
        marker.pose.position.z = admitted_position.z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.marker_publisher.publish(marker)


        # calculate distance from current position
        dx = admitted_position.x-self.actual_x
        dy = admitted_position.y-self.actual_y
        dz = admitted_position.z-self.actual_z
        d = (dx**2+dy**2+dz**2)**0.5
        time = d/self.speed

        self.request_pose(admitted_position.x,admitted_position.y,admitted_position.z,time)

    # run event handler task
    def run(self):
        # event loop
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            # handle events
            while(not self.event_queue.empty()):
                callback, data = self.event_queue.get()
                callback(data)
            
            # sleep
            rate.sleep()
            


if __name__=="__main__":
    coordinator_node = Coordinator()
    coordinator_node.run()