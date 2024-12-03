# Copyright 2024 NXP

# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
import time
import math
import numpy as np

from rclpy.node import Node
from collections import deque
from sensor_msgs.msg import Joy
from synapse_msgs.msg import EdgeVectors
from synapse_msgs.msg import TrafficStatus
from sensor_msgs.msg import LaserScan

QOS_PROFILE_DEFAULT = 10

PI = math.pi

LEFT_TURN = +1.0
RIGHT_TURN = -1.0

TURN_MIN = 0.0
TURN_MAX = 1.0
SPEED_MIN = 0.0
SPEED_MAX = 1.0
# SPEED_MAX = 0.25
SPEED_25_PERCENT = SPEED_MAX / 4
SPEED_50_PERCENT = SPEED_25_PERCENT * 2
SPEED_75_PERCENT = SPEED_25_PERCENT * 3

THRESHOLD_OBSTACLE_VERTICAL = 1.0
THRESHOLD_OBSTACLE_HORIZONTAL = 0.25



class LineFollower(Node):
	""" Initializes line follower node with the required publishers and subscriptions.

		Returns:
			None
	"""
	def __init__(self):
		super().__init__('line_follower')

		# Subscription for edge vectors.
		#self.list_dist = []
		self.subscription_vectors = self.create_subscription(
			EdgeVectors,
			'/edge_vectors',
			self.edge_vectors_callback,
			QOS_PROFILE_DEFAULT)
		
		self.subscription_full_vectors = self.create_subscription(
			EdgeVectors,
			'/full_edge_vectors',
			self.full_edge_vectors_callback,
			QOS_PROFILE_DEFAULT)
		

		# Publisher for joy (for moving the rover in manual mode).
		self.publisher_joy = self.create_publisher(
			Joy,
			'/cerebri/in/joy',
			QOS_PROFILE_DEFAULT)

		# Subscription for traffic status.
		self.subscription_traffic = self.create_subscription(
			TrafficStatus,
			'/traffic_status',
			self.traffic_status_callback,
			QOS_PROFILE_DEFAULT)

		# Subscription for LIDAR data.
		self.subscription_lidar = self.create_subscription(
			LaserScan,
			'/scan',
			self.lidar_callback,
			QOS_PROFILE_DEFAULT)

		self.traffic_status = TrafficStatus()

		self.obstacle_detected = False

		self.turn = TURN_MIN
		self.buggy_state = 'LANE_FOLLOWING'
		self.ramp_min_speed = 0.5
		self.ramp_max_speed = 1.0
		self.ramp_detected = False
		self.ramp_detection_threshold = 1.0
		self.ramp_progress = 0.0
		self.lidar_max_range = 10.0
		self.lidar_angle_increment = 0.0175
		self.lidar_msg = None
		self.full_edgeVectors_msg = None
		self.edgeVectors_msg = None


	""" Operates the rover in manual mode by publishing on /cerebri/in/joy.

		Args:
			speed: the speed of the car in float. Range = [-1.0, +1.0];
				Direction: forward for positive, reverse for negative.
			turn: steer value of the car in float. Range = [-1.0, +1.0];
				Direction: left turn for positive, right turn for negative.

		Returns:
			None
	"""
	def rover_move_manual_mode(self, speed, turn):
		msg = Joy()

		msg.buttons = [1, 0, 0, 0, 0, 0, 0, 1]

		msg.axes = [0.0, speed, 0.0, turn]

		self.publisher_joy.publish(msg)


	""" Updates instance member with traffic status message received from /traffic_status.

		Args:
			message: "~/cognipilot/cranium/src/synapse_msgs/msg/TrafficStatus.msg"

		Returns:
			None
	"""
	def traffic_status_callback(self, message):
		self.traffic_status = message



	""" Analyzes edge vectors received from /edge_vectors to achieve line follower application.
		It checks for existence of ramps & obstacles on the track through instance members.
			These instance members are updated by the lidar_callback using LIDAR data.
		The speed and turn are calculated to move the rover using rover_move_manual_mode.

		Args:
			message: "~/cognipilot/cranium/src/synapse_msgs/msg/EdgeVectors.msg"

		Returns:
			None
	"""
	def edge_vectors_callback(self, message):
		# speed = SPEED_MAX
		self.turn = TURN_MIN
		self.edgeVectors_msg = message
		vectors = message
		half_width = vectors.image_width / 2

		# NOTE: participants may improve algorithm for line follower.
		if (vectors.vector_count == 0):  # none.
			pass

		if (vectors.vector_count == 1):  # curve.
			# Calculate the magnitude of the x-component of the vector.
			deviation = vectors.vector_1[1].x - vectors.vector_1[0].x
			turn = deviation / vectors.image_width

		if (vectors.vector_count == 2):  # straight.
			# Calculate the middle point of the x-components of the vectors.
			middle_x_left = (vectors.vector_1[0].x + vectors.vector_1[1].x) / 2
			middle_x_right = (vectors.vector_2[0].x + vectors.vector_2[1].x) / 2
			middle_x = (middle_x_left + middle_x_right) / 2
			deviation = half_width - middle_x
			turn = deviation / half_width
			self.turn = turn


	""" Analyzes LIDAR data received from /scan topic for detecting ramps/bridges & obstacles.

		Args:
			message: "docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html"

		Returns:
			None
	"""
	def lidar_callback(self, msg):
		# TODO: participants need to implement logic for detection of ramps and obstacles.
		speed = SPEED_MAX
		turn = TURN_MIN
		self.ramp_detected = self.detect_ramp()
		print('ramp detected\n')
		
		if self.ramp_detected:
			self.update_ramp_progress(msg.ranges)


	# this functions is defined just to update the full_edgeVectors's msg variable
	def full_edge_vectors_callback(self, message):
		self.full_edgeVectors_msg = message
		pass


	# function to detect ramp using callback messages of lidar & edge vectors
	def detect_ramp(self):
		vectors = self.full_edgeVectors_msg
		lidar_msg = self.lidar_msg
		half_width = vectors.image_width
		if (vectors.count == 2):
			# Calculate the middle point of the left vector
			middle_x_left = (vectors.vector_1[0].x + vectors.vector_1[1].x) / 2
			middle_y_left = (vectors.vector_1[0].y + vectors.vector_1[1].y) / 2
			
			# Calculate the middle point of the right vector
			middle_x_right = (vectors.vector_2[0].x + vectors.vector_1[1].x) / 2
			middle_y_right = (vectors.vector_2[0].y + vectors.vector_1[1].y) / 2
			
			# calculate the bottom middle point of full image(as the relative buggy position)
			image_middle_x = half_width
			image_middle_y = vectors.image_height
			
			# calculate the range of angles the ramp will be visible in (using mid points of ramp vectors)
			theta_left = math.atan((image_middle_x - middle_x_left) / (image_middle_y - middle_y_left))
			theta_right = math.atan((image_middle_x - middle_x_right) / (image_middle_y - middle_y_right))
			
			# calculate the corresponding ranges of indices of lidar data
			lidar_left_index_range = int(theta_left/self.lidar_angle_increment)
			lidar_right_index_range = int(theta_right/self.lidar_angle_increment)
			
			lidar_ramp_range_indices = []
			min_disance = self.lidar_max_range
			continuous_data = False
			
			# check if the (lidar data meets the ramp threshold & the data is continous) in this range or not
			for i in range(lidar_left_index_range, lidar_right_index_range - 1, -1):
				continuous_data = True
				if (min_disance < lidar_msg.ranges[i]):
					min_disance = lidar_msg.ranges[i]
					
				if (lidar_msg.ranges[i] > self.lidar_max_range):
					continuous_data = False
					break
					
			if (min_disance < self.ramp_detection_threshold & continuous_data is True):
				self.ramp_detected = True
				
		return self.ramp_detected


	def update_ramp_progress(self, lidar_ranges):
		# Convert LIDAR ranges to a numpy array for easier processing
		ranges = np.array(lidar_ranges)

		# Define the expected ramp profile
		ramp_horizontal_length = 1.5  # meters
		ramp_height = 0.45  # meters
		
		# Calculate the angle of the ramp
		ramp_angle = np.arctan2(ramp_height, ramp_horizontal_length)

		# Find the index of the minimum range (closest point)
		min_range_index = np.argmin(ranges)

		# Get the angle of this point
		angle_increment = self.get_parameter('angle_increment').value
		min_range_angle = min_range_index * angle_increment

		# Calculate the vertical component of this range
		vertical_component = ranges[min_range_index] * np.sin(min_range_angle)

		# Estimate progress based on vertical position
		self.ramp_progress = np.clip(vertical_component / ramp_height, 0.0, 1.0)

		# Update the state based on progress
		if self.state == 'RAMP_APPROACH' and self.ramp_progress > 0.1:
			self.state = 'RAMP_ASCENT'
		elif self.state == 'RAMP_ASCENT' and self.ramp_progress > 0.9:
			self.state = 'RAMP_PLATEAU'
		elif self.state == 'RAMP_PLATEAU' and vertical_component < ramp_height * 0.9:
			self.state = 'RAMP_DESCENT'
		elif self.state == 'RAMP_DESCENT' and self.ramp_progress < 0.1:
			self.state = 'RAMP_END'

		self.get_logger().info(f'Ramp progress: {self.ramp_progress:.2f}, State: {self.state}')



	def calculate_speed(self):
		if self.state == 'LANE_FOLLOWING' and self.ramp_detected:
			self.state = 'RAMP_APPROACH'
			
		if self.state == 'RAMP_APPROACH':
			# Gradually decrease speed
			self.current_speed = self.max_speed - (self.max_speed - self.min_speed) * (1 - self.ramp_progress)
			if self.ramp_progress >= 1.0:
				self.state = 'RAMP_ASCENT'
				
		elif self.state == 'RAMP_ASCENT':
			# Gradually increase speed
			self.current_speed = self.min_speed + (self.max_speed - self.min_speed) * self.ramp_progress
			if self.ramp_progress >= 1.0:
				self.state = 'RAMP_PLATEAU'
				
		elif self.state == 'RAMP_PLATEAU':
			# Maintain minimum speed
			self.current_speed = self.min_speed
			if self.ramp_progress >= 1.0:
				self.state = 'RAMP_DESCENT'
				
		elif self.state == 'RAMP_DESCENT':
			# Gradually increase speed
			self.current_speed = self.min_speed + (self.max_speed - self.min_speed) * self.ramp_progress
			if self.ramp_progress >= 1.0:
				self.state = 'RAMP_END'
				
		elif self.state == 'RAMP_END':
			# Gradually decrease speed
			self.current_speed = self.max_speed - (self.max_speed - self.min_speed) * (1 - self.ramp_progress)
			if self.ramp_progress >= 1.0:
				self.state = 'LANE_FOLLOWING'
				self.ramp_detected = False
				
		elif self.state == 'LANE_FOLLOWING':
			# Full speed
			self.current_speed = self.max_speed

		return self.current_speed



	def rover_move_manual_mode(self, speed, turn):
		msg = Joy()
		msg.buttons = [1, 0, 0, 0, 0, 0, 0, 1]
		msg.axes = [0.0, speed, 0.0, turn]
		self.publisher_joy.publish(msg)



	def run_buggy(self):
		rate = self.create_rate(10)  # 10 Hz
		while rclpy.ok():
			speed = self.calculate_speed()
			self.rover_move_manual_mode(speed, self.turn)
			rclpy.spin_once(self)
			rate.sleep()



def main(args=None):

	rclpy.init(args=args)

	line_follower = LineFollower()

	rclpy.spin(line_follower)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	line_follower.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
