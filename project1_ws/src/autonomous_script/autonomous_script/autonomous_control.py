import rclpy
import math
import matplotlib.pyplot as plt
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import time

class AutomaticControlNode(Node):
    def __init__(self): 
        super().__init__('automatic_control_node')
        
        # Initialization of attributes
        self.target_reached = False        # To check if target is reached
        self.target_x = 10                 # Target X coordinate
        self.target_y = 10                 # Target Y coordinate
        self.kp = 20 * math.pi / 180
        self.steer_angle = 0.0             # Steer angle
        
        # Publisher for joint position (steer angle) and wheel velocity
        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Subscriber from odometry and velocity topic
        self.measure_odom = self.create_subscription(PoseStamped, "/odom", self.update_pose, qos_profile=10)
        self.measure_velocity = self.create_subscription(Twist, "/velocity", self.update_vel, qos_profile=10)

        # Initialize plotting variables for real-time toycar pose plot
        self.x_positions = []
        self.y_positions = []
        self.t_values = []  # New time tracking list
        self.start_time = time.time()  # Track the start time

        # Set up matplotlib for real-time plotting
        plt.ion()
        self.fig, (self.ax_xy, self.ax_xt, self.ax_yt) = plt.subplots(1, 3, figsize=(15, 5))

        # Plot for X vs Y
        self.line_xy, = self.ax_xy.plot([], [], 'bo-')
        self.ax_xy.set_xlim(0, 20)
        self.ax_xy.set_ylim(0, 20)
        self.ax_xy.set_xlabel('X Position')
        self.ax_xy.set_ylabel('Y Position')
        self.ax_xy.set_title('Pose plot')

        # Plot for X vs Time
        self.line_xt, = self.ax_xt.plot([], [], 'r-')
        self.ax_xt.set_xlabel('Time (s)')
        self.ax_xt.set_ylabel('X Position')
        self.ax_xt.set_title('X vs Time')

        # Plot for Y vs Time
        self.line_yt, = self.ax_yt.plot([], [], 'g-')
        self.ax_yt.set_xlabel('Time (s)')
        self.ax_yt.set_ylabel('Y Position')
        self.ax_yt.set_title('Y vs Time')

    # Distance between two coordinates
    def euler_distance(self, point1, point2):
        return math.sqrt((point1[1] - point2[1]) ** 2 + (point1[0] - point2[0]) ** 2)
        
    # Calculating angle to the target from the current position of toycar
    def target_direction(self, position, target):
        return math.atan2(target[1] - position[1], target[0] - position[0])
        
    # Subscribing and extracting velocity data from the message
    def update_vel(self, msg):
        self.x_vel = msg.linear.x
        self.y_vel = msg.linear.y
        self.z_vel = msg.linear.z
        
        # Printing subscribing velocity in the terminal 
        self.get_logger().info(f"Subscribing Velocity: x={self.x_vel}, y={self.y_vel}, z={self.z_vel}")
        
        # Condition to check if toycar has reached target location or not
        wheel_velocities = Float64MultiArray()
        condition_to_move = self.euler_distance([self.x, self.y], [self.target_x, self.target_y]) > 1.0
        if not condition_to_move:
            wheel_velocities.data = [0.0]
            self.target_reached = True
        else:
            wheel_velocities.data = [-10.0]
        
        # Printing the published velocity to the rear wheel of toycar in the terminal
        self.get_logger().info(f"Publishing Wheel Velocities: {wheel_velocities.data}")
        self.wheel_velocities_pub.publish(wheel_velocities)

    def update_pose(self, msg):
        if self.target_reached:
            return
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z
        
        # Subscribing the position data in the terminal
        self.get_logger().info(f"Subscribing Position from /odom - x: {self.x}, y: {self.y}, z: {self.z}")
        
        # Update the plotting data
        self.x_positions.append(self.x)
        self.y_positions.append(self.y)
        self.t_values.append(time.time() - self.start_time)

        # Update X vs Y plot
        self.line_xy.set_xdata(self.x_positions)
        self.line_xy.set_ydata(self.y_positions)

        # Update X vs T plot
        self.line_xt.set_xdata(self.t_values)
        self.line_xt.set_ydata(self.x_positions)

        # Update Y vs T plot
        self.line_yt.set_xdata(self.t_values)
        self.line_yt.set_ydata(self.y_positions)

        for ax in (self.ax_xy, self.ax_xt, self.ax_yt):
            ax.relim()
            ax.autoscale_view()
        plt.pause(0.01)         # Pause to update the plot
        
        joint_positions = Float64MultiArray()
        
        # Converting quaternion to Euler angles
        self.roll_x, self.pitch_y, self.yaw_z = self.euler_from_quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        
        # Calculating error between target direction and orientation of vehicle
        error = self.target_direction([self.x, self.y], [self.target_x, self.target_y]) - self.yaw_z  
        change = self.kp * error  
        
        # Limit Steering angles
        self.steer_angle = change
        if self.steer_angle > 1.0:
            self.steer_angle = 1.0
        if self.steer_angle < -1.0:
            self.steer_angle = -1.0
           
        final_angle = self.steer_angle
        
        joint_positions.data = [final_angle, final_angle]
        
        # Printing published position in the terminal
        self.get_logger().info(f"Publishing Joint Positions: {joint_positions.data}")
        self.joint_position_pub.publish(joint_positions)
        # self.get_logger().info(str(msg.orientation))

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z  # in radians

def main(args=None):
    rclpy.init(args=args)
    node = AutomaticControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
