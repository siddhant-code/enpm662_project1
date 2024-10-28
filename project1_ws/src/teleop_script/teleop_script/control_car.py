#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import sys
import select
import tty
import termios
#from pynput import keyboard

# Define key codes
LIN_VEL_STEP_SIZE = 10
ANG_VEL_STEP_SIZE = 0.1
MAX_VELOCITY = 30.0
MAX_STEER = 1.0

class KeyboardControlNode(Node):

    def __init__(self):
        super().__init__('keyboard_control_node')

        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.settings = termios.tcgetattr(sys.stdin)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run_keyboard_control(self):
        self.msg = """
        Control Your Car!
        ---------------------------
        Moving around:
            w
        a    s    d

        q : force stop

        Esc to quit
        """

        self.get_logger().info(self.msg)
        joint_positions = Float64MultiArray()
        wheel_velocities = Float64MultiArray()
        linear_vel=0.0
        steer_angle=0.0


        while True:
            key = self.getKey()
            if key != "":
                if key == '\x1b':  # Escape key
                    break
                elif key == 'q':  # Quit
                    linear_vel=0.0
                    steer_angle=0.0
                elif key == 'w':  # Forward
                    linear_vel += LIN_VEL_STEP_SIZE
                elif key == 's':  # Reverse
                    linear_vel -= LIN_VEL_STEP_SIZE
                elif key == 'd':  # Right
                    steer_angle =round(steer_angle - ANG_VEL_STEP_SIZE,2)
                elif key == 'a':  # Left
                    steer_angle = round(steer_angle + ANG_VEL_STEP_SIZE,2)

                # Limiting steer and velocity input to max values
                if steer_angle>MAX_STEER:
                        steer_angle=MAX_STEER
                if steer_angle<-MAX_STEER:
                    steer_angle=-1*MAX_STEER
                    
                if linear_vel > MAX_VELOCITY:
                    linear_vel = MAX_VELOCITY
                if linear_vel < -1*MAX_VELOCITY:
                    linear_vel = -1* MAX_VELOCITY

            
            # If no button is pressed we want the steering to return to original postion
            else:
                if steer_angle != 0.0:
                    if steer_angle > 0.0:
                        steer_angle = round(steer_angle - ANG_VEL_STEP_SIZE,2)
                    elif steer_angle <= 0.0:
                        steer_angle = round(steer_angle + ANG_VEL_STEP_SIZE,2)
 
            # Publish the twist message
            wheel_velocities.data = [-linear_vel]     # an angular vel of -10 makes vehivle move forward, and 10 makes it move backward
            joint_positions.data = [steer_angle,steer_angle]

            self.joint_position_pub.publish(joint_positions)
            self.wheel_velocities_pub.publish(wheel_velocities)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    node.run_keyboard_control()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()