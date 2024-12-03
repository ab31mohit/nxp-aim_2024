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
MAX_RAMP_SPEED = SPEED_50_PERCENT

PUB_SPEED_HZ = 9.9
PUB_EDGE_VECTORS_HZ= 30
PUB_SCAN_HZ = 9.9

class LineFollower(Node):

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

        # Publisher for joy (for moving the rover in manual mode).
        self.subscriber_joy = self.create_subscription(
            Joy,
            '/cerebri/in/joy',
            self.vel_callback,
            QOS_PROFILE_DEFAULT)

        # Subscription for LIDAR data.
        self.subscription_lidar = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            QOS_PROFILE_DEFAULT)
        
        self.curr_speed = SPEED_MIN
        self.curr_turn = TURN_MIN
        self.ramp_detected_threshold = 2.5
        self.ramp_ended_threshold = 1.5
        self.target_turn = TURN_MIN
        self.target_speed = SPEED_MIN
        self.distance = 0.0
        self.ramp_ended = False
        self.last_turn = TURN_MIN
        self.prev_error_speed = 0.0
        self.prev_error_turn = 0.0
        self.edgeVectors_topic_pub_count = 0
        # self.lidar_msg = None
        self.edgeVector_msg = None
        self.lidar_front_ranges = np.zeros(40)
        self.lidar_front_min_distance_index = None
        self.lidar_front_min_distance_value = None


    def rover_move_manual_mode(self, speed, turn):
        msg = Joy()

        msg.buttons = [1, 0, 0, 0, 0, 0, 0, 1]

        msg.axes = [0.0, speed, 0.0, turn]

        self.publisher_joy.publish(msg)

    def vel_callback(self, msg):
        self.curr_speed = msg.axes[1]
        self.curr_turn = msg.axes[3]
        print(f"Currenlty : speed ={self.curr_speed}\t turn ={self.curr_turn}\n")

    def edge_vectors_callback(self, message):
        # NOTE: participants may improve algorithm for line follower.
        
        # this callback function will be called with a freq of 30hz(30 times in every second),
        # but we will update the code using this callback functions after every 3rd calling of this functions,
        # means we will use this function to update the code with a freq of 10hz (to match lidar data freq)
        if self.edgeVectors_topic_pub_count % 3 == 0:

            # speed = SPEED_MAX
            # turn = TURN_MIN
            vectors = message
            self.edgeVector_msg = message
            half_width = vectors.image_width / 2
            # speed = self.curr_speed
            ideal_slope_left = 0.4381      # slope in Right hand cartesian frame of image
            ideal_slope_left_inclination = math.atan(ideal_slope_left)
            ideal_slope_right = -0.4086    # slope in Right hand cartesian frame of image
            ideal_slope_right_inclination = math.atan(ideal_slope_right)
            
            if vectors.vector_count == 2:
                if (vectors.vector_1[0].x - vectors.vector_1[1].x) != 0.0:
                    curr_slope_left = -(vectors.vector_1[0].y - vectors.vector_1[1].y
                                        ) / (vectors.vector_1[0].x - vectors.vector_1[1].x)
                else: curr_slope_left = ideal_slope_left
                
                if ((vectors.vector_2[0].x - vectors.vector_2[1].x)) != 0.0:
                    curr_slope_right = -(vectors.vector_2[0].y - vectors.vector_2[1].y
                                        ) / (vectors.vector_2[0].x - vectors.vector_2[1].x)
                else: curr_slope_right = ideal_slope_right
                
                curr_slope_left_inclination = math.atan(abs(curr_slope_left))
                curr_slope_right_inclination = math.atan(abs(curr_slope_right))
            
    
            if (vectors.vector_count == 0):  # none.
                self.target_speed = SPEED_50_PERCENT
                self.target_turn = TURN_MIN

            if (vectors.vector_count == 1):  # curve.
                
                if (vectors.vector_1[1].x - vectors.vector_1[0].x) != 0.0:
                    slope = -(vectors.vector_1[1].y - vectors.vector_1[0].y
                            ) / (vectors.vector_1[1].x - vectors.vector_1[0].x)
                else:
                    if (vectors.vector_1[0].x > half_width):
                        slope = -10
                    else: slope = 10
                
                if (slope > 0):
                    print(f"only left lane, slope ={slope}")
                    # error = slope_ideal_left - slope
                    # d_error = (error - self.prev_error_turn) / dt
                    # turn = self.curr_turn + kp*error + kd*d_error
                    # self.prev_error_turn = error
                    self.target_turn = -TURN_MAX
                
                else:
                    print(f"only right lane, slope ={slope}")
                    # error = slope_ideal_right - slope
                    # d_error = (error - self.prev_error_turn) / dt
                    # turn = self.curr_turn + kp*error + kd*d_error
                    # self.prev_error_turn = error
                    self.target_turn = TURN_MAX
                
                self.target_speed = SPEED_50_PERCENT
            
            if (vectors.vector_count == 2):  # straight.
                # deviation = half_width - middle_x
                # turn = deviation / half_width
                
                del_left_inclination = abs(ideal_slope_left_inclination) - curr_slope_left_inclination
                del_right_inclination = abs(ideal_slope_right_inclination) - curr_slope_right_inclination
                
                del_inclination = del_left_inclination - del_right_inclination
                
                self.target_turn = del_inclination
                self.target_speed = SPEED_75_PERCENT

            # in the end, we'll update the count variable for this function (re-initialize count)
            self.edgeVectors_topic_pub_count %= 3
        
        self.edgeVectors_topic_pub_count += 1
            
        

    def lidar_callback(self, msg):
        # TODO: participants need to implement logic for detection of ramps and obstacles.
        self.distance = msg.ranges[180]
        self.lidar_msg =msg.ranges
        self.lidar_front_ranges = msg.ranges[200 : 159, -1]
        self.lidar_front_min_distance_value = np.min(self.lidar_front_ranges)
        self.lidar_front_min_distance_index = np.argmin(self.lidar_front_ranges)
        # print(self.distance)
        self.calculate_speed()

    def detect_ramp(self):
        # define front lidar data range of buggy
        # lidar_front_ranges = np.arange(200, 159,-1)    # saved indices from 200 to 160 (size = 40)
        
        # min_distance_value = np.min(lidar_front_ranges)
        # min_distance_index = np.argmin(lidar_front_ranges)
        vector_count = self.edgeVector_msg.vector_count
        min_dist = self.lidar_front_min_distance_value
        min_dist_index = self.lidar_front_min_distance_index
        # wide_view_range = self.lidar_msg.ranges[]
        
        
        if vector_count == 2 and min_dist < self.ramp_detected_threshold:
            print(f"Ramped detected, Min dist= {min_dist:.2f}, at index= {min_dist_index}")
            return [True, min_dist, min_dist_index]
        
        else: return [False, min_dist, min_dist_index]
        
    def handle_ramp(self, min_dist, min_dist_index):
        dt_speed = 0.1 
        kp_speed = 0.1
        kd_speed = 0.05
    
        
        frequency = 10  # Hz
        period = 1 / frequency  # Period in seconds
        
        while (True):
            start_time = time.time()
            p_error = (MAX_RAMP_SPEED - self.curr_speed)
            d_error = (p_error - self.prev_error_speed) / dt_speed
            
            speed = self.curr_speed + kp_speed*p_error + kd_speed*d_error
            self.prev_error_speed = p_error
            self.rover_move_manual_mode(speed, self.target_turn)
            self.ramp_ended = True

            # Function logic goes here
            print("Ramp Function logic executing")

            # Calculate how long the function took to execute
            elapsed_time = time.time() - start_time

            # Sleep for the remaining time in the period, if any
            time_to_sleep = period - elapsed_time
            if time_to_sleep > 0:
                time.sleep(time_to_sleep)
            else:
                print("Warning: Ramp Function execution took longer than the period")
            
        self.prev_error_speed = 0.0
        self.ramp_ended = True
        
    
    def calculate_speed(self):
        dt_speed = 0.1 
        kp_speed = 0.1
        kd_speed = 0.05
        
        kp_turn = 0.2
        kd_turn = 0.05
        dt_turn = 0.1
        
        if (self.distance <= self.ramp_detected_threshold):
            print('##########  Ramp detected  ##########\n')
            p_error = (MAX_RAMP_SPEED - self.curr_speed)
            d_error = (p_error - self.prev_error_speed) / dt_speed
            speed = self.curr_speed + kp_speed*p_error + kd_speed*d_error
            self.prev_error_speed = p_error
            self.rover_move_manual_mode(speed, self.target_turn)
            self.ramp_ended = True
            # self.rover_move_manual_mode(speed, self.current_turn)
        else:
            self.prev_error_speed = 0.0
            self.prev_error_turn = 0.0
            # print('lane following\n')
            if self.ramp_ended is True:
                start_time = time.time()
                elasped_time = 0.0
                # print("****waiting****")
                while (elasped_time <= 0.6):
                    self.rover_move_manual_mode(self.curr_speed, self.target_turn)
                    elasped_time = time.time() - start_time
                self.ramp_ended = False
            else:
                print("Entering PID")
                # speed pid
                p_error = (self.target_speed - self.curr_speed)
                d_error = (p_error - self.prev_error_speed) / dt_speed
                speed = self.curr_speed + kp_speed*p_error + kd_speed*d_error
                self.prev_error_speed = p_error

                # turn pid
                error_turn = self.target_turn - self.curr_turn
                d_error_turn = (error_turn - self.prev_error_turn) / dt_turn
                turn = self.curr_turn + kp_turn*error_turn + kd_turn*d_error_turn
                self.prev_error_turn = error_turn
                self.rover_move_manual_mode(speed, turn)


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

