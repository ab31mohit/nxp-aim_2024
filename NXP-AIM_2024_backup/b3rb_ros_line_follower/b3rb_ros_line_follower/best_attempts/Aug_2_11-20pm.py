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


'''
We figure out the reason for turn going from beyond 1.0 as we were using tan theta to get the slope value
and from there, the value of slope angle so it was exceeding the range. Here we used sin theta to get the slope angle
and now the turn was always in the range of (0,1). Now the buggy is behaving similar to the earlier best, but this time
buggy was always using the turn within its range.
currently the buggy is behaving less accurately on turn (1 vectors cases).
'''

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
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion, quaternion_from_euler

QOS_PROFILE_DEFAULT = 10

PI = math.pi

LEFT_TURN = +1.0
RIGHT_TURN = -1.0

TURN_MIN = 0.0
TURN_MAX = 1.0

SPEED_MIN = 0.0
SPEED_MAX = 1.0

SPEED_25_PERCENT = SPEED_MAX / 4
SPEED_50_PERCENT = SPEED_25_PERCENT * 2
SPEED_75_PERCENT = SPEED_25_PERCENT * 3

THRESHOLD_OBSTACLE_VERTICAL = 1.0
THRESHOLD_OBSTACLE_HORIZONTAL = 0.25


class LineFollower(Node):

    def __init__(self):
        super().__init__('line_follower')

        # Subscription for edge vectors.
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

        # Subscriber for joy (for getting velocity of buggy).
        self.subscriber_joy = self.create_subscription(
            Joy,
            '/cerebri/in/joy',
            self.vel_callback,
            QOS_PROFILE_DEFAULT)

        # Subscription for /scan (for getting LIDAR data).
        self.subscription_lidar = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            QOS_PROFILE_DEFAULT)
        
        # Subscription for odometry (for geting odometry data).
        self.subscription_odom = self.create_subscription(
            Odometry,
            '/cerebri/out/odometry',
            self.odom_callback,
            QOS_PROFILE_DEFAULT)
        
        self.curr_speed = SPEED_MIN
        self.curr_turn = TURN_MIN
        self.curr_z = 0.0
        self.prev_z = 0.0
        self.diff_z = 0.0

        self.edgeVectors_topic_pub_count = 0
        
        self.prev_error_speed = 0.0
        self.prev_error_turn = 0.0

    ################################################
    def odom_callback(self, msg):
        self.curr_z = msg.pose.pose.position.z
        self.diff_z = self.curr_z - self.prev_z
        self.prev_z = self.curr_z
    
        quat = msg.pose.pose.orientation
        quat_list = [quat.x, quat.y, quat.z, quat.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)

    
    #################################################
    def rover_move_manual_mode(self, speed, turn):
        msg = Joy()
        msg.buttons = [1, 0, 0, 0, 0, 0, 0, 1]
        msg.axes = [0.0, speed, 0.0, turn]
        self.publisher_joy.publish(msg)
    
    
    #################################################
    def vel_callback(self, msg):
        self.curr_speed = msg.axes[1]
        self.curr_turn = msg.axes[3]
        # print(f"Currenlty: speed= {self.curr_speed:.2f}\t turn= {self.curr_turn:.2f}\n")
        
    
    ##################################################
    def edge_vectors_callback(self, msg):
        
        # this callback function will be called with a freq of 30hz(30 times in every second),
        # but we will update the code using this callback functions after every 3rd calling of this functions,
        # means we will use this function to update the code with a freq of 10hz (to match lidar data freq)
        
        if self.edgeVectors_topic_pub_count % 3 == 0:
            
            speed = SPEED_MAX
            turn = TURN_MIN

            vectors = msg
            half_width = vectors.image_width / 2

            # NOTE: participants may improve algorithm for line follower.
            if (vectors.vector_count == 0):  # none.
                speed = SPEED_50_PERCENT
                # turn = TURN_MIN

            if (vectors.vector_count == 1):  # curve.
                # Calculate the magnitude of the x-component of the vector.
                
                x2 = vectors.vector_1[0].x
                x1 = vectors.vector_1[1].x
                y2 = vectors.vector_1[0].y
                y1 = vectors.vector_1[1].y
                
                len_vector = math.sqrt((x2-x1)**2.0 + (y2-y1)**2)
                
                if (x2 - x1) != 0:
                    slope_angle = math.asin(abs(y2-y1) / len_vector)
                    del_theta = PI/2.0 - slope_angle
                else:
                    slope_angle = PI/2.0
                    del_theta = 0.0
                    
                del_theta = np.interp(del_theta, [0.0, PI/2.0], [0.0, 1.0])
                
                if (x2 - x1) > 0:
                    turn = -del_theta*1.3
                    
                else: turn = del_theta*1.3
                
                # steer = np.interp(del_theta, [0, PI/2.0], [0, 1])
                
                # speed = SPEED_75_PERCENT
                
            if (vectors.vector_count == 2):  # straight.
                # Calculate the middle point of the x-components of the vectors.
                middle_x_left = (vectors.vector_1[0].x + vectors.vector_1[1].x) / 2
                middle_x_right = (vectors.vector_2[0].x + vectors.vector_2[1].x) / 2
                middle_x = (middle_x_left + middle_x_right) / 2
                deviation = half_width - middle_x
                turn = deviation / half_width

            # if (self.traffic_status.stop_sign is True):
            #     speed = SPEED_MIN
            #     print("stop sign detected")

            # if self.ramp_detected is True:
            #     # TODO: participants need to decide action on detection of ramp/bridge.
            #     print("ramp/bridge detected")

            # if self.obstacle_detected is True:
            #     # TODO: participants need to decide action on detection of obstacle.
            #     print("obstacle detected")

            self.rover_move_manual_mode(speed, turn)
            print(f"Currenlty: speed= {self.curr_speed:.2f}\t turn= {self.curr_turn:.2f}\n")
        
            
            self.edgeVectors_topic_pub_count %= 3 
        
        self.edgeVectors_topic_pub_count += 1
        
    
    ####################################################
    def pid_speed(self, target_speed):
        kp = 0.10
        kd = 0.05
        dt = 0.099

        error = (target_speed - self.curr_speed)
        d_error = (error - self.prev_error_speed) / dt
        speed = self.curr_speed + kp*error + kd*d_error
        self.prev_error_speed = error
        
        # if speed > SPEED_MAX:
        #     speed = SPEED_MAX
        # elif speed < -SPEED_MAX:
        #     speed = -SPEED_MAX
        # else: speed = speed
         
        return speed
    
    
    ####################################################
    def pid_turn_1vec(self, target_turn):
        kp = 0.2
        kd = 0.05
        dt = 0.099
        
        error = target_turn - self.curr_turn
        d_error = (error - self.prev_error_turn) / dt
        turn = self.curr_turn + kp*error + kd*d_error
        
        self.prev_error_turn = error
        return turn
    

    ####################################################
    def pid_turn_2vec(self, target_turn):
        kp = 0.1
        kd = 0.05
        dt = 0.099
        
        error = target_turn - self.curr_turn
        d_error = (error - self.prev_error_turn) / dt
        turn = self.curr_turn + kp*error + kd*d_error
        
        self.prev_error_turn = error
        return turn

    
    ####################################################
    def lidar_callback(self, msg):
        # TODO: participants need to implement logic for detection of ramps and obstacles.
        pass
    


##################################################################################
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