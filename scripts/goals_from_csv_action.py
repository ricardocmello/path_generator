#!/usr/bin/python
import rospy
import numpy as np
import csv
import math

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
#from nav_msgs.msg import Odometry
#from std_msgs.msg import Header
#from std_msgs.msg import Float32
from geometry_msgs.msg import Pose, Point, PoseStamped, Quaternion
from std_srvs.srv import EmptyResponse, Empty
from tf.transformations import euler_from_quaternion as efq
from tf.transformations import quaternion_from_euler as qfe
from threading import Lock

class GoalsFromCsv():
	def __init__(self, name):
		self.name = name
		self.rospy = rospy
		self.rospy.init_node(self.name, anonymous = True)
		self.rospy.loginfo("[%s] Starting Node", self.name)
		self.actionlib = actionlib
		self.initParameters()
		self.initSubscribers()
		self.initPublishers()
		self.initServiceClients()
		self.initActionClients()
		self.initVariables()
		self.main()

	def initParameters(self):
		self.goals_loop = self.rospy.get_param("~goals_loop", False)
		self.goal_topic = self.rospy.get_param("~goal_topic", "/move_base_simple/goal")
		self.goal_rate = self.rospy.get_param("~path_rate", 10)
		self.goal_tolerances = {"xy": self.rospy.get_param("~goal_tolerances/xy", 0.3),
								"theta": self.rospy.get_param("~goal_tolerances/theta", 0.45)}
		self.csv_file = self.rospy.get_param("~csv_file","/home/walker/catkin_ws/src/path_generator/paths/path_points.csv")
		self.csv_header = self.rospy.get_param("~csv_header",True)
		self.odom_topic = self.rospy.get_param("~odom_topic", "/odometry/filtered")
		self.frame_id = self.rospy.get_param("~frame_id","odom")
		self.updateParamsService = self.name + self.rospy.get_param("~update_params_service", "/update_parameters")
		self.wait_time = self.rospy.get_param("~wait_time", 5.0)
		self.param_lock = Lock()
		return

	def initSubscribers(self):
		#self.sub_odom = self.rospy.Subscriber(self.odom_topic, Odometry, self.callback_odom)
		return

	def initPublishers(self):
		self.pub_goal = self.rospy.Publisher(self.goal_topic, PoseStamped, queue_size = 5)
		return

	def initServiceClients(self):
		self.service = self.rospy.Service(self.updateParamsService, Empty, self.callback_update_params)
		return

	def initActionClients(self):
		self.action_client = self.actionlib.SimpleActionClient('move_base', MoveBaseAction)
		self.wait = self.action_client.wait_for_server(self.rospy.Duration(self.wait_time))
		return

	def initVariables(self):
		self.rate = self.rospy.Rate(self.goal_rate)
		self.msg_goal = MoveBaseGoal()
		self.msg_pose = Pose()
		self.change = False
		self.read_flag = False
		self.final_goal_reached = False
		self.goal_reached = False
		self.goal_published = False
		self.goals = np.array([])
		self.goal_id = 0
		self.goals_number = 0
		self.seq = 0
		self.x_bot, self.y_bot = 0, 0
		self.qz_bot, self.qw_bot = 0, 0
		self.theta_bot = 0
		self.x_goal, self.y_goal = 0, 0
		self.qz_goal, self.qw_goal = 0, 0
		self.theta_goal = 0
		self.exit = False
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
		self.theta_bot = self.theta_bot[2]
		self.change = True
		return

	def read_file(self):
		try:
			with open(self.csv_file) as csv_file:
				csv_reader = csv.reader(csv_file, delimiter=',')
				for line_counter, row in enumerate(csv_reader):
					if line_counter > 0:
						row_i = np.reshape(map(float, np.array(row)), (1, 4))
						if line_counter == 1:
							self.goals = row_i
						else:
							self.goals = np.concatenate((self.goals, row_i))
				self.goals_number = self.goals.size
			self.read_flag = True
		except Exception as e:
			print(e)
			self.read_flag = False
		return

	def get_current_goal(self):
		self.current_goal = self.goals[self.goal_id]
		self.x_goal, self.y_goal = self.current_goal[0], self.current_goal[1]
		self.qz_goal, self.qw_goal = self.current_goal[2], self.current_goal[3]
		self.theta_goal = efq([0, 0, self.qz_goal, self.qw_goal])
		self.theta_goal = self.theta_goal[2]
		return

	def make_msg_header(self):
		self.msg_goal.target_pose.header.frame_id = self.frame_id
		self.msg_goal.target_pose.header.stamp = self.rospy.Time.now()
		return

	def make_msg_pose(self):
		self.msg_pose.position.x = self.x_goal
		self.msg_pose.position.y = self.y_goal
		self.msg_pose.orientation.x = 0
		self.msg_pose.orientation.y = 0
		self.msg_pose.orientation.z = self.qz_goal
		self.msg_pose.orientation.w = self.qw_goal
		self.msg_goal.target_pose.pose = self.msg_pose
		return

	def publish_goal(self):
		self.get_current_goal()
		self.make_msg_header()
		self.make_msg_pose()
		self.action_client.send_goal(self.msg_goal, self.callback_done, self.callback_active, self.callback_feedback)
		self.rospy.loginfo("[%s] Sending Goal with ID %d to Action Server", self.name, self.goal_id)
		self.goal_published = True
		self.goal_reached = False
		return

	def callback_active(self):
		self.rospy.loginfo("[%s] The goal with ID %d is now being processed by the Action Server...", self.name, self.goal_id)
		return

	def callback_feedback(self, feedback):
		#self.rospy.loginfo("[%s] Feedback for goal with ID %d received", self.name, self.goal_id)
		return

	def callback_done(self, status, result):
		if status == 2:
			self.rospy.loginfo("[%s] The goal with ID %d received a cancel request after it started executing", self.name, self.goal_id)
		elif status == 3:
			self.rospy.loginfo("[%s] Reached Goal %d successfully", self.name, self.goal_id)
			self.goal_id += 1
			self.goal_published = False
			if self.goal_id > self.goals_number - 1:
				self.rospy.loginfo("[%s] Reached final goal", self.name)
				self.final_goal_reached = True
				return
		elif status == 4:
			self.rospy.loginfo("[%s] The goal with ID %d was aborted by the Action Server", self.name, self.goal_id)
			self.rospy.signal_shutdown("Goal with ID "+str(self.goal_id)+" aborted, shutting down!")
			return
		elif status == 5:
			self.rospy.loginfo("[%s] The goal with ID %d has been rejected by the Action Server", self.name, self.goal_id)
 			self.rospy.signal_shutdown("Goal with ID "+str(self.goal_id)+" rejected, shutting down!")
 			return
		elif status == 8:
			self.rospy.loginfo("[%s] The goal with ID %d received a cancel request before it started executing, successfully cancelled!", self.name, self.goal_id)
		else:
			pass
		return


	def main(self):
		self.rospy.loginfo("[%s] Configuration OK", self.name)
		if self.wait:
			self.rospy.loginfo("[%s] Connected to move base server", self.name)
			self.read_file()
			if self.read_flag:
				self.rospy.loginfo("[%s] Reading CSV file OK", self.name)
				while not self.rospy.is_shutdown() and not self.exit:
					if not self.final_goal_reached:
						if not self.goal_published:
							self.publish_goal()
					else:
						if self.goals_loop:
							self.goal_id = 0
							self.final_goal_reached = False
						else:
							self.exit = True
					self.rate.sleep()
			else:
				self.rospy.logerr("[%s] Reading CSV failed", self.name)
				self.rospy.logwarn("[%s] Exiting due to CSV reading error", self.name)
		else:
			self.rospy.logerr("Action server not available!")


if __name__ == '__main__':
	try:
		pg = GoalsFromCsv('GoalsFromCsv')
	except rospy.ROSInterruptException:
		pass
