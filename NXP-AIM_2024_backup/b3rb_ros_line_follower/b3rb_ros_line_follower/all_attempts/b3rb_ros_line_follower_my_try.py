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
from rclpy.node import Node
from collections import deque
from sensor_msgs.msg import Joy
import time
import math

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

		self.ramp_started = False
		self.ramp_top = False
		self.ramp_finished = True
		self.ramp_upLength = deque(maxlen=2)
		self.ramp_slant_height = None
		self.ramp_slant_covered = 0.0
		self.ramp_threshold_distance = 1.5
		self.curr_speed = 0.0
		self.curr_turn = 0.0
		self.ramp_close_threshold = 1.0
		self.ramp_about_to_finish = False
		# self.current_accel = 0.0

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
		turn = TURN_MIN

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

		if (self.traffic_status.stop_sign is True):
			# speed = SPEED_MIN
			pass
			# print("stop sign detected")

		if self.ramp_started is True:
			# TODO: participants need to decide action on detection of ramp/bridge.
			# print("ramp/bridge detected\n")
			pass
			# Changes
			# speed = SPEED_50_PERCENT
			# self.rover_move_manual_mode(speed, turn)

		if self.obstacle_detected is True:
			# TODO: participants need to decide action on detection of obstacle.
			# print("obstacle detected")
			pass
			# Changes
			# speed = 0.25
			# self.rover_move_manual_mode(speed, turn)
		# self.curr_speed = speed
		self.curr_turn = turn
		self.rover_move_manual_mode(self.curr_speed, turn)

	""" Updates instance member with traffic status message received from /traffic_status.

		Args:
			message: "~/cognipilot/cranium/src/synapse_msgs/msg/TrafficStatus.msg"

		Returns:
			None
	"""
	def traffic_status_callback(self, message):
		self.traffic_status = message

	""" Analyzes LIDAR data received from /scan topic for detecting ramps/bridges & obstacles.

		Args:
			message: "docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html"

		Returns:
			None
	"""
	def lidar_callback(self, message):
		# TODO: participants need to implement logic for detection of ramps and obstacles.
		speed = SPEED_MAX
		turn = TURN_MIN

		# index_mid = 180 
		index_left = 187
		index_right = 173
		distance = min(message.ranges[index_right:index_left])

		# if (distance < self.ramp_threshold_distance):
		# 	print(f'Ramp detected\t Distance={distance}\n')
		# 	self.ramp_started = True
		# 	self.ramp_upLength.append(distance)
		# 	# speed = SPEED_25_PERCENT
		# else :
		# 	self.ramp_started = False
		
		if (not self.ramp_started) and  (self.ramp_finished) and (distance < self.ramp_threshold_distance):
			# # check ramp started
			# if (distance < self.ramp_threshold_distance):
			# print(f'Ramp detected\t Distance={distance}\n')
			self.ramp_started = True
			print(f"Actually Ramp started, speed={self.curr_speed}\n")
			self.ramp_finished = False

		elif self.ramp_started and self.ramp_slant_height == None:
			self.ramp_upLength.append(distance)
			if distance > 50:
				self.ramp_slant_height = self.ramp_upLength.popleft()
				print(f"climbing started, speed={self.curr_speed}\n")

		elif self.ramp_started and self.ramp_slant_height is not None:
			# check for ramp mid acheived
			self.ramp_slant_covered += self.curr_speed/9.9
			if (abs(self.ramp_slant_covered - self.ramp_slant_height)/2.0 < 0.1):
				self.ramp_top = True
				self.ramp_started = False
				self.ramp_slant_height = None
				self.ramp_slant_covered = 0.0
				print(f"reached top, speed={self.curr_speed}\n")
				self.smooth_target_vel(SPEED_50_PERCENT, -5.0)


		elif self.ramp_top:
			# slow down
			# check for road
			if (distance < 10.0):
				self.smooth_target_vel(SPEED_MAX, 0.5)
				print(f"started going down, speed={self.curr_speed}\n")
			
				if (distance < self.ramp_close_threshold):
					self.ramp_about_to_finish = True
					self.smooth_target_vel(SPEED_50_PERCENT, -1.0)
					self.ramp_top = False
					print(f"finished ramp, speed={self.curr_speed}\n")

		elif self.ramp_about_to_finish:
			if (distance > 10.0):
				self.smooth_target_vel(SPEED_MAX, 1.0)
				self.ramp_started = False
				self.ramp_finished = True
				print(f"back to road, speed={self.curr_speed}\n")
				# self.ramp_finished = False
			# check for inf
				# increase
		elif self.ramp_finished:		
			self.curr_speed = speed
			self.curr_turn = turn
			self.rover_move_manual_mode(speed, turn)

		# for i in range(index_left, index_right+1,):
		# 	print(f'range[{i}]={message.ranges[i]}')

		# print("\n################\n")

		# self.obstacle_detected = False
		# self.rover_move_manual_mode(speed, turn)

	def smooth_target_vel(self, speed, accel):
		target_speed = self.curr_speed + accel * 0.050
		if (target_speed > SPEED_MAX):
			target_speed = SPEED_MAX
		if (target_speed < SPEED_50_PERCENT):
			target_speed = SPEED_50_PERCENT
		self.curr_speed = target_speed
		self.rover_move_manual_mode(target_speed, self.curr_turn)



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
