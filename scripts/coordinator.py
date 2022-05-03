#!/usr/bin/env python
import rospy
import threading
import Queue as queue
from open_manipulator_msgs.srv import SetKinematicsPose, SetKinematicsPoseRequest
from open_manipulator_msgs.msg import KinematicsPose
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

class Coordinator:
    def __init__(self):
        # initialize ros node
        rospy.init_node('telemanipulator_coordinator')
        print("node initialized")

        # create robot mover thread and queue
        self.speed = 0.1 # 10cm/s
        self.move_queue = queue.Queue()
        move_thread = threading.Thread(target=self.move_thread)
        move_thread.daemon = daemon=True
        move_thread.start()

        # create event queue
        self.event_queue = queue.Queue()

        # initialize calls to robot pose request service
        rospy.wait_for_service("/open_manipulator/goal_task_space_path_position_only")
        self.goal_task_space_service = rospy.ServiceProxy("/open_manipulator/goal_task_space_path_position_only", SetKinematicsPose)

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

            # create request and set data
            request = SetKinematicsPoseRequest()
            request.end_effector_name = "gripper"
            request.path_time = time
            request.kinematics_pose.pose.position.x = x
            request.kinematics_pose.pose.position.y = y
            request.kinematics_pose.pose.position.z = z

            # call service
            print("New TCP position %g %g %g (time=%g) requested"%(x,y,z,time))
            response = self.goal_task_space_service(request)
            print(response)

            rospy.sleep(time)
    
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
        # calculate distance from current position
        dx = desired_position.x-self.actual_x
        dy = desired_position.y-self.actual_y
        dz = desired_position.z-self.actual_z
        d = (dx**2+dy**2+dz**2)**0.5
        time = d/self.speed

        self.request_pose(desired_position.x,desired_position.y,desired_position.z,time)

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

    # run event handler task
    def run(self):
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