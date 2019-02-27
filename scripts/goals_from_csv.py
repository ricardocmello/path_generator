#!/usr/bin/python
import rospy
import numpy as np
import csv
import math

from tf.transformations import euler_from_quaternion as efq
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from std_srvs.srv import EmptyResponse, Empty
from threading import Lock

class GoalsFromCsv():
    def __init__(self, name):
        self.name = name
        self.rospy = rospy
        self.rospy.init_node(self.name, anonymous = True)
        self.rospy.loginfo("[%s] Starting Node", self.name)
		self.initParameters()
		self.initSubscribers()
		self.initPublishers()
		self.initServiceClients()
		self.initVariables()
        self.main()

    def initParameters(self):
		self.path_topic = self.rospy.get_param("~path_topic","/path")
        self.path_loop = self.rospy.get_param("~path_loop", False)
        self.goal_topic = self.rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.goal_rate = self.rospy.get_param("~path_rate", 10)
        self.goal_tolerances = {"xy": self.rospy.get_param("~goal_tolerances/xy", 0.3),
                                "theta": self.ropsy.get_param("~goal_tolerances/theta", 0.35)}
		self.csv_file = self.rospy.get_param("~csv_file","~/catkin_ws/src/path_generator/paths/path_points.csv")
        self.csv_header = self.rospy.get_param("~csv_header",True)
        self.odom_topic = self.rospy.get_param("~odom_topic", "/RosAria/pose")
		self.frame_id = self.rospy.get_param("~frame_id","odom")
		self.updateParamsService = self.name + self.rospy.get_param("~update_params_service", "/update_parameters")
		self.param_lock = Lock()
        return

    def initSubscribers(self):
        self.sub_odom = self.rospy.Publishers(self.odom_topic, Odometry, self.callback_odom)
        return

    def initPublishers(self):
        self.pub_goal = self.rospy.initPublishers(self.goal_topic, PoseStamped, queue_size = 5)
        return

    def initServiceClients(self):
        self.service = self.rospy.Service(self.updateParamsService, Empty, self.callback_update_params)
        return

    def initVariables(self):
        self.rate = self.ropsy.Rate(self.goal_rate)
        self.msg_goal = PoseStamped()
        self.msg_header = Header()
        self.msg_pose = Pose()
        self.change = False
        self.read_flag = False
        self.goal_reached = False
        self.goals = np.array()
        self.goal_id = 0
        self.seq = 0
        return

    def callback_update_params(self, req):
		with self.param_lock:
			self.initParameters()
			self.rospy.loginfo("[%s] Parameter update after request", self.name)
        return EmptyResponse()

    def callback_odom(self, msg):
        self.x_bot = msg.pose.pose.position.x
        self.y_bot = msg.pose.pose.position.y
        self.qz_bot = msg.pose.pose.orientation.z
        self.qw_bot = msg.pose.pose.orientation.w
        self.theta_bot = efq([0, 0, self.qz_bot, self.qw_bot])
        self.change = True
        return

    def read_file(self):
        try:
            with open(self.csv_file) as csv_file:
                csv_reader = csv.reader(csv_file, delimiter=',')
                for line_counter, row in enumerate(csv_reader):
                    if line_counter > 0:
                        self.goals = np.array(np.array(row))
            self.read_flag = True
        except:
            self.read_flag = False
        return read_flag

    def get_current_goal(self):
        self.x_goal, self.y_goal = self.current_goal[0], self.current_goal[1]
        self.qz_goal, self.qw_goal = self.current_goal[2], self.current_goal[3]
        self.theta_goal = efq([0, 0, self.goal_qz, self.goal_qw])
        return

    def make_msg_header(self):
        self.msg_header.seq = self.seq
        self.msg_header.stamp.secs = self.rospy.get_rostime().secs
        self.msg_header.stamp.nsecs = self.rospy.get_rostime().nsecs
        self.msg_header.frame_id = self.frame_id
        self.seq += 1
        return

    def make_msg_pose(self):
        self.msg_pose.position.x = self.x_goal
        self.msg_pose.position.y = self.y_goal
        self.msg_pose.orientation.x = 0
        self.msg_pose.orientation.y = 0
        self.msg_pose.orientation.qz = self.qz_goal
        self.msg_pose.orientation.qw = self.qw_goal
        return

    def publish_goal(self):
        self.current_goal = self.goals(self.goal_id)
        self.make_msg_header()
        self.make_msg_pose()
        self.msg_goal.header = self.msg_header
        self.msg_goal.pose = self.msg_pose
        self.pub_goal.publish(self.msg_goal)
        return

    def check_goal(self):
        dist_to_goal = sqrt((self.x_goal - self.x_bot)**2 + (self.y_goal - self.y_bot)**2)
        angle_to_goal = abs(self.goal_theta - self.theta_bot)
        if dist_to_goal <= self.goal_tolerances["xy"]
            self.rospy.loginfo("[%s] Reached Goal %d XY position", self.name, self.goal_id)
            if angle_to_goal <= self.goal_tolerances["theta"]:
                self.rospy.loginfo("[%s] Reached Goal %d orientation", self.name, self.goal_id)
                self.goal_reached = True
        return

    def main(self):
        self.rospy.loginfo("[%s] Configuration OK", self.name)
        self.read_file()
        if self.read_flag:
            self.rospy.loginfo("[%s] Reading CSV file OK", self.name)
            while not self.rospy.is_shutdown():
                if self.change:
                    self.publish_goal()
                self.rate.sleep()
        else:
            self.rospy.logerr("[%s] Reading CSV failed", self.name)



if __name__ == '__main__':
	try:
		pg = PathGen('GoalsFromCsv')
	except rospy.ROSInterruptException:
        print("")
        pass
