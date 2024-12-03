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
This code ran the buggy thoughout the track(after first ramp to second ramp). Most of the time buggy was on the track.
But at some sharp curves, the buggy was on the edge line of sometimes goes out of track but maintains the track
for most of the time.
'''

import rclpy
import time
import math
import numpy as np
import cv2

from rclpy.node import Node
from collections import deque
from sensor_msgs.msg import Joy
from synapse_msgs.msg import EdgeVectors
from synapse_msgs.msg import TrafficStatus
from sensor_msgs.msg import CompressedImage
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


RAMP_DETECTION_DISTANCE = 1.5  # meters
RAMP_ANGLE_THRESHOLD = 10  # degrees
RAMP_SAFE_SPEED = 0.5
RAMP_HEIGHT_THRESHOLD = 0.05  # meters

LIDAR_THRESHOLD = 10.0
FLAT_TRACK_THRESHOLD = 0.78


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
        
        
        self.raw_image = self.create_subscription(
           CompressedImage,
           '/camera/image_raw/compressed',
           self.image_callback,
           QOS_PROFILE_DEFAULT)
        
        self.curr_speed = SPEED_MIN
        self.curr_turn = TURN_MIN
        self.curr_z = 0.0
        self.prev_z = 0.0
        self.diff_z = 0.0
        
        self.curr_edgeVec_count = 0
        self.ramp_detected = False
        # self.on_ramp = False
        # self.ramp_start_height = 0.0
        self.ramp_stage = "none"

        self.edgeVectors_topic_pub_count = 0
        
        self.edgeVectors = None
        self.trajectory = None
        
        self.prev_error_speed = 0.0
        self.prev_error_turn = 0.0
        
        self.exp_func = 0.0
        
    ###############################################
    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        # Display image
        if self.edgeVectors.vector_count != 0:
            if self.edgeVectors.vector_count == 1:
                point_1 = (int(self.edgeVectors.vector_1[1].x), int(self.edgeVectors.vector_1[1].y))
                point_2 = (int(self.edgeVectors.vector_1[0].x), int(self.edgeVectors.vector_1[0].y))
                cv2.line(image, point_1, point_2, (0, 255, 0), 2)
            else:
                left_point_1 = (int(self.edgeVectors.vector_1[1].x), int(self.edgeVectors.vector_1[1].y))
                left_point_2 = (int(self.edgeVectors.vector_1[0].x), int(self.edgeVectors.vector_1[0].y)) 
                cv2.line(image, left_point_1, left_point_2, (0, 255, 0), 2)
                
                right_point_1 = (int(self.edgeVectors.vector_2[1].x), int(self.edgeVectors.vector_2[1].y))
                right_point_2 = (int(self.edgeVectors.vector_2[0].x), int(self.edgeVectors.vector_2[0].y))
                cv2.line(image, right_point_1, right_point_2, (0, 255, 0), 2)
    
        if self.trajectory is not None:
            point_1 = self.trajectory[0]
            point_2 = self.trajectory[1]
            cv2.line(image, point_1, point_2, (0, 0, 255), 2)
        
        cv2.imshow("Image", image)
        cv2.waitKey(1)



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
            self.edgeVectors = msg
            half_width = vectors.image_width / 2
            
            self.curr_edgeVec_count = vectors.vector_count

            # NOTE: participants may improve algorithm for line follower.
            if (vectors.vector_count == 0):  # none.
                speed = SPEED_50_PERCENT
                # turn = TURN_MIN
                self.trajectory = ((int(half_width), int(vectors.image_height)), (int(half_width), int(vectors.image_height/2)))
                
            # Adjust speed for ramps
            if self.ramp_stage == "flat" or self.ramp_stage == "none":
                speed = SPEED_MAX
                # self.get_logger().info("Ramp completed, resuming normal speed")
            elif self.ramp_detected or self.ramp_stage != "none":
                speed = RAMP_SAFE_SPEED
                # self.get_logger().info(f"On or approaching ramp, maintaining speed at {speed}")



            if (vectors.vector_count == 1):  # curve.
                
                if (vectors.vector_1[1].x - vectors.vector_1[0].x) != 0:
                    slope = (vectors.vector_1[0].y - vectors.vector_1[1].y
                            ) / (vectors.vector_1[0].x - vectors.vector_1[1].x)
                    len_vec = math.sqrt((vectors.vector_1[1].y - vectors.vector_1[0].y)**2 + (vectors.vector_1[1].x - vectors.vector_1[0].x)**2 )
                    slope_angle = abs(math.atan(slope))
                    del_theta = PI/2.0 - slope_angle
                else:
                    slope = float('inf')
                    slope_angle = PI/2.0
                    del_theta = 0.0
                
                # steer = np.interp(del_theta, [0, PI/2.0], [0, 1])
                
                # if (slope < 0):
                #     turn = -del_theta + self.exp_func
                # else: turn = del_theta + self.exp_func
                turn = (slope/abs(slope))*slope_angle + self.exp_func
                
                if not math.isinf(slope):
                    top_x = half_width + len_vec*math.cos(slope_angle)
                    top_y = vectors.image_height - len_vec*math.sin(slope_angle)
                else:
                    top_x = half_width
                    top_y = vectors.vector_1[0].y
                    
                self.trajectory = ((int(half_width), int(vectors.image_height)), (int(top_x), int(top_y)))
                

            if (vectors.vector_count == 2):  # straight.
                # Calculate the middle point of the x-components of the vectors.
                middle_x_left = (vectors.vector_1[0].x + vectors.vector_1[1].x) / 2
                middle_y_left = (vectors.vector_1[0].y + vectors.vector_1[1].y) / 2
                middle_x_right = (vectors.vector_2[0].x + vectors.vector_2[1].x) / 2
                middle_y_right = (vectors.vector_2[0].y + vectors.vector_2[1].y) / 2
                middle_x = (middle_x_left + middle_x_right) / 2
                middle_y = (middle_y_left + middle_y_right) / 2

                deviation = half_width - middle_x
                turn = deviation / half_width
                turn = turn + self.exp_func
                
                # Use PID for smoother speed control
                # if abs(self.exp_func) > 0.4:
                #     target_speed = speed - 0.3
                # else:target_speed = speed
                
                target_speed = speed
                speed = self.pid_speed(target_speed)
                
                
                self.trajectory = ((int(half_width), int(vectors.image_height)), (int(middle_x), int(middle_y)))
                


            self.rover_move_manual_mode(speed, turn)
            print(f"Currenlty: speed= {self.curr_speed:.2f} turn= {self.curr_turn:.2f} height= {self.curr_z:.2f}\n")
        
            
            self.edgeVectors_topic_pub_count %= 3 
        
        self.edgeVectors_topic_pub_count += 1
        
    
    ####################################################
    def pid_speed(self, target_speed):
        kp = 0.3
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
            # Detect ramp using LIDAR data
        front_ranges = msg.ranges[178:184]  # Adjust indices based on your LIDAR setup (for ramp condition only)
        front_obst_ranges = np.array(msg.ranges[165:195])  # for detecting obstacles
        print('###### arry', front_obst_ranges)
        k1 = 0.3
        self.exp_func = k1 * self.calculate_exponential_sum(front_obst_ranges)
        
        # print(front_ranges)
        avg_front_distance = sum(front_ranges) / len(front_ranges)
        valid_ranges = [r for r in front_ranges if r != float('inf') and r != 0]
            
        if self.ramp_detected:
            
            if valid_ranges and self.ramp_stage == "approaching":
                self.ramp_stage = "approaching"
            elif not valid_ranges and self.ramp_stage == "descending":
                self.ramp_stage = "flat"
                print("######### Back on flat track #######")
                self.ramp_detected = False
            elif not valid_ranges:    # All infinity, we're on the ascending part
                self.ramp_stage = "ascending"
            elif min(valid_ranges) < LIDAR_THRESHOLD:    # We're on the descending part
                self.ramp_stage = "descending"
            # elif all(r < FLAT_TRACK_THRESHOLD for r in valid_ranges):  # Back on flat ground
            #     self.ramp_stage = "flat"
            #     print("######### Back on flat track #######")
            #     self.ramp_detected = False
                
            print(f"\n{valid_ranges}\n")
            print(f"Ramp stage: {self.ramp_stage}")
        else:
            if avg_front_distance < RAMP_DETECTION_DISTANCE and self.curr_edgeVec_count == 2 and valid_ranges:
                self.ramp_detected = True
                self.ramp_stage = "approaching"
                print("Ramp detected!")
                
    
    def calculate_exponential_sum(self,numbers):

        # Calculate the negative exponential of each number
        beta = 2
        negative_exponentials = np.exp(-numbers*beta)

        # Subtract exponentials from 1 to 5
        subtract_part = np.sum(negative_exponentials[:5])

        # Add exponentials from 6 to 10
        add_part = np.sum(-negative_exponentials[5:])

        # Total value
        total_value = subtract_part + add_part

        return total_value

            



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