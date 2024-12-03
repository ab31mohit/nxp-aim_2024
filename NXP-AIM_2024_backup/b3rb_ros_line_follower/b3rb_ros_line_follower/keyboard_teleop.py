import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import sys, select, os
if os.name == 'nt':
    import msvcrt, time
else:
    import tty, termios
# from sensor_msgs.msg import CompressedImage
# import numpy as np
# import cv2
# import math
# from synapse_msgs.msg import EdgeVectors

QOS_PROFILE_DEFAULT = 10

MAX_LIN_SPEED = 2.0
MIN_LIN_SPEED = 0.0
LIN_SPEED_STEP_SIZE = 0.1
MAX_TURN_SPEED = 1.0
MIN_TURN_SPEED = 0.0
TURN_SPEED_STEP_SIZE = 0.1

msg = """
Control your b3rb mobile robot your keyboard!
---------------------------------------------------------------------------
Movement in horizontal plane:
        w                     
    a       d                
        x                    

keys  : corresponding action on by the buggy     
------ ---------------------------------------------------------------------
w/x   : move buggy forward/backward linearly (change linear vel in x)
a/d   : take left/right turn (change angular velocity along z or yaw motion)
s     : stop turning/steering but keep moving linearly (stop yaw motion)
space : stop buggy completely :)

CTRL-C to quit
"""

e = """
Unable to connect to b3rb robot :(!
"""

######### Assist functions for teleop and system events #############

# function to get the key feedback of user from keyboard
def getKey(settings):
    if os.name == 'nt':
        timeout = 0.1
        startTime = time.time()
        while(1):
            if msvcrt.kbhit():
                if sys.version_info[0] >= 3:
                    return msvcrt.getch().decode()
                else:
                    return msvcrt.getch()
            elif time.time() - startTime > timeout:
                return ''
    
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


# Displays current velocity of b3rb in 2d plane (velocity in x and steer along z)
def vels(lin_vel, turn):
    return "currently:\tLinear speed={:.2f}\tTurning speed={:.2f}".format(lin_vel, turn)

# smoothing the calculated output velocity of robot 
def smooth_velocity(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input

    return output

# select the optimal option to assist the limiting velocity condition
def constrain(input, low, high):
    if input < low:
        input = low
    elif input > high:
        input = high
    else:
        input = input

    return input

# check linear speed limit
def checkLinearSpeedLimit(vel):
    vel = constrain(vel, -MAX_LIN_SPEED, MAX_LIN_SPEED)
    return vel

# check turnning speed limit
def checkTurningSpeedLimit(vel):
    vel = constrain(vel, -MAX_TURN_SPEED, MAX_TURN_SPEED)
    return vel

# Class for Teleop node
class KeyboardTeleop(Node):

    def __init__(self):
        super().__init__('b3rb_keyboard_teleop_node')
        print("Keyboard teleop node is running!")

        if os.name != 'nt':
            self.settings = termios.tcgetattr(sys.stdin)
        else:
            self.settings = None

        # Publisher for joy (for moving the rover in manual mode).
        self.publisher_joy = self.create_publisher(Joy,
            '/cerebri/in/joy',QOS_PROFILE_DEFAULT)
        
        self.status = 0
        self.target_lin_vel = 0.0
        self.control_lin_vel = 0.0
        self.target_turn_vel = 0.0
        self.control_turn_vel = 0.0

        print(msg)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        key = getKey(self.settings)

        if key == 'w':
            self.target_lin_vel = checkLinearSpeedLimit(self.target_lin_vel + LIN_SPEED_STEP_SIZE)
            self.status += 1
            print(vels(self.target_lin_vel, self.target_turn_vel))
        elif key == 'x':
            self.target_lin_vel = checkLinearSpeedLimit(self.target_lin_vel - LIN_SPEED_STEP_SIZE)
            self.status += 1
            print(vels(self.target_lin_vel, self.target_turn_vel))
        elif key == 'a':
            self.target_turn_vel = checkTurningSpeedLimit(self.target_turn_vel + TURN_SPEED_STEP_SIZE)
            self.status += 1
            print(vels(self.target_lin_vel, self.target_turn_vel))
        elif key == 'd':
            self.target_turn_vel = checkTurningSpeedLimit(self.target_turn_vel - TURN_SPEED_STEP_SIZE)
            self.status += 1
            print(vels(self.target_lin_vel, self.target_turn_vel))
        elif key == 's':
            self.target_turn_vel = 0.0
            self.control_turn_vel = 0.0
            print(vels(self.target_lin_vel, self.target_turn_vel))
        elif key == ' ':
            self.target_lin_vel = 0.0
            self.control_lin_vel = 0.0
            self.target_turn_vel = 0.0
            self.control_turn_vel = 0.0
            print(vels(self.target_lin_vel, self.target_turn_vel))
        else:
            if key == '\x03':
                self.destroy_node()
                rclpy.shutdown()

        # for the case when terminal gets flooded with velocity log & you forget the keys to control the bot
        # so display the message again that shows how to control the robot
        if self.status == 20:
            print(msg)
            self.status = 0

        # creating object of message type of Joy for feeding velocity
        joy_msg = Joy()
        joy_msg.buttons = [1, 0, 0, 0, 0, 0, 0, 1]

        # calculating final velocity control velocity to be fed to the robot from the calculated velocity
        self.control_lin_vel = smooth_velocity(self.control_lin_vel, self.target_lin_vel, (LIN_SPEED_STEP_SIZE / 2.0))

        self.control_turn_vel = smooth_velocity(self.control_turn_vel, self.target_turn_vel, (TURN_SPEED_STEP_SIZE / 2.0))
        
        # publishing the final control velocity to the robot
        joy_msg.axes = [0.0, float(self.control_lin_vel), 0.0, float(self.control_turn_vel)]
        self.publisher_joy.publish(joy_msg)

    #  for shutting down the keyboard event callback features and stopping the robot
    def shutdown(self):
        msg_joy = Joy()
        msg_joy.buttons = [1, 0, 0, 0, 0, 0, 0, 1]
        msg_joy.axes = [0.0, MIN_LIN_SPEED, 0.0, MIN_TURN_SPEED]
        self.publisher_joy.publish(msg_joy)

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        

# defining main function that this node will run
def main(args=None):
		
    rclpy.init(args=args)
    teleop_node = KeyboardTeleop()

    try:
        rclpy.spin(teleop_node)
    
    except Exception as ex:
        print(e)
        print(str(ex))
    
    finally:
        teleop_node.shutdown()
        teleop_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()