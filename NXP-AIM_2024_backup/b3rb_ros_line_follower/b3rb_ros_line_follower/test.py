import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import LaserScan
import numpy as np
import cv2
import math
from synapse_msgs.msg import EdgeVectors
from functools import partial


QOS_PROFILE_DEFAULT = 10

class Test(Node):

    def __init__(self):
        super().__init__('my_test_node')
        print("My test node is running!")

        # self.thresh_image = self.create_subscription(
        #    CompressedImage,
        #    '/debug_images/thresh_image',
        #    partial(self.image_callback,image_title='thresh_image'),
        #    QOS_PROFILE_DEFAULT)
        
        # self.vector_image = self.create_subscription(
        #    CompressedImage,
        #    '/debug_images/vector_image',
        #    partial(self.image_callback,image_title='vector_image'),
        #    QOS_PROFILE_DEFAULT)

        # self.raw_image = self.create_subscription(
        #    CompressedImage,
        #    '/camera/image_raw/compressed',
        #    partial(self.image_callback,image_title='raw_image'),
        #    QOS_PROFILE_DEFAULT)
        
                # Subscription for edge vectors.
        self.subscription_vectors = self.create_subscription(
            EdgeVectors,
            '/edge_vectors',
            self.edge_vectors_callback,
            QOS_PROFILE_DEFAULT)
        
        self.raw_image = self.create_subscription(
           CompressedImage,
           '/camera/image_raw/compressed',
           self.image_callback,
           QOS_PROFILE_DEFAULT)
        
        # Subscription for odometry (for geting odometry data).
        # self.subscription_odom = self.create_subscription(
        #     Odometry,
        #     '/cerebri/out/odometry',
        #     self.odom_callback,
        #     QOS_PROFILE_DEFAULT)
        
        self.sub_scan = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.lidar_callback, 
            QOS_PROFILE_DEFAULT)
        
        # self.ramp_threshold_distance = 1.5
        self.edgeVectors_topic_pub_count = 0
        self.edgeVectors = None
        self.trajectory = None
        self.curr_edgeVec_count = 0



    ################################################
    def edge_vectors_callback(self, msg):

        if self.edgeVectors_topic_pub_count % 3 == 0:

            vectors = msg
            self.edgeVectors = msg
            half_width = vectors.image_width / 2
            
            self.curr_edgeVec_count = vectors.vector_count

            # NOTE: participants may improve algorithm for line follower.
            if (vectors.vector_count == 0):  # none.
                pass

            if (vectors.vector_count == 1):  # curve.
                if (vectors.vector_1[1].x - vectors.vector_1[0].x) != 0:
                    slope = (vectors.vector_1[1].y - vectors.vector_1[0].y
                            ) / (vectors.vector_1[1].x - vectors.vector_1[0].x)
                    slope_angle = abs(math.atan(slope))
                    del_theta = math.pi/2.0 - slope_angle
                else:
                    slope = float('inf')
                    slope_angle = math.pi/2.0
                    del_theta = 0.0
                
                # steer = np.interp(del_theta, [0, PI/2.0], [0, 1])
                
                if (slope < 0):
                    turn = -del_theta
                else: turn = del_theta
                
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
                
                self.trajectory = ((int(half_width), int(vectors.image_height)), (int(middle_x), int(middle_y)))
    
            self.edgeVectors_topic_pub_count %= 3 
        
        self.edgeVectors_topic_pub_count += 1
    
    
    def odom_callback(self, msg):    
        quat = msg.pose.pose.orientation
        quat_list = [quat.x, quat.y, quat.z, quat.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        
        # print(f"roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}\n")


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

    def lidar_callback(self, msg):
        # Parameters
        forward_angle_range = 10  # Degrees for forward-facing range
        min_distance = float('inf')

        # Calculate the index range for the forward direction
        total_ranges = len(msg.ranges)

        # Note : Actually nxp organizers put lidar facing behind the buggy, so index positions
        # 3,2,1,0,359,358,357 corresponds to back of buggy(instead of front) and the ranges array's
        # index positions increase anticlockwise

        # index position of lidar ranges array facing forward to buggy
        index_mid = 180 
        index_left = 195
        index_right = 165

        # distance = min(msg.ranges[index_right:index_left])

        # if (distance < self.ramp_threshold_distance):
        #     print(f'\nRamp detected\n')
        #     # self.ramp_detected = True

        for i in range(index_left, index_right-1, -1):
            # if distance == msg.ranges[i]:
            #     print(f'Min dist={distance} at index={i}\n')
            print(f'ranges[{i}]={msg.ranges[i]}')

        print("\n################\n")



        
def main(args=None):
    rclpy.init(args=args)

    test_node = Test()

    rclpy.spin(test_node)

    # Destroy the node explicitly
    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()